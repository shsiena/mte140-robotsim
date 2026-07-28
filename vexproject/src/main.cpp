/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       simon                                                     */
/*    Created:      7/28/2026, 2:50:10 PM                                     */
/*    Description:  IQ2 project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the IQ2 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here

#include "program.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "bitgrid.h"
#include "config.h"
#include "masks.h"
#include "robot.h"
#include "screen.h"
#include "vexrobot.h"


// --- linked list ---

struct Node {
    Node* next = nullptr;
    Node* prev = nullptr;
    bool headingVertical = false;
    float xPos = 0.0;
    float yPos = 0.0;
};

class LinkedList {
private:
    Node* root_ = nullptr;
    uint8_t numNodes = 0;

public:
    ~LinkedList() {
        Node* currentNode = root_;
        while (currentNode != nullptr) {
            Node* nextNode = currentNode->next;
            delete currentNode;
            currentNode = nextNode;
        }
        root_ = nullptr;
        numNodes = 0;
        }

    void add(float x, float y, bool vertical) {
        Node* newNode = new Node;
        newNode->xPos = x;
        newNode->yPos = y;
        newNode->headingVertical = vertical;

        if (root_ == nullptr) {
            root_ = newNode;
            numNodes++;
            return;
        }

        Node* currentNode = root_;
        while (currentNode->next != nullptr) {
            currentNode = currentNode->next;
        }

        currentNode->next = newNode;
        newNode->prev = currentNode;
        numNodes++;
    }
};


namespace {

// Single-precision copies of the compile-time constants. The Cortex-M4 FPU is
// single-precision only, so a double-typed constant left in a float expression
// would silently pull the whole expression into software emulation
const float SUBCELL_CM = static_cast<float>(config::SUBCELL_CM);
const float SENSOR_FORWARD_CM = static_cast<float>(config::SENSOR_FORWARD_CM);
const float SENSOR_RIGHT_CM = static_cast<float>(config::SENSOR_RIGHT_CM);
const float SENSOR_MAX_CM = static_cast<float>(config::SENSOR_MAX_CM);
const float SENSOR_HALF_CONE_DEG = static_cast<float>(config::SENSOR_HALF_CONE_DEG);
const float CLEAR_MARGIN_CM = static_cast<float>(config::CLEAR_MARGIN_CM);
const float MIN_STEP_CM = static_cast<float>(config::MIN_STEP_CM);
const float MIN_MOTION_CM = static_cast<float>(config::MIN_MOTION_CM);
const float SWEEP_STEP_DEG = static_cast<float>(config::SWEEP_STEP_DEG);
const float START_CLEAR_RADIUS_CM = static_cast<float>(masks::START_CLEAR_RADIUS_CM);

const float NORTH_DEG = 0.0f;
const float EAST_DEG = 90.0f;
const float QUARTER_TURN_DEG = 90.0f;
const float FULL_TURN_DEG = 360.0f;

const float PI = 3.14159265358979323846f;
const float DEG_TO_RAD = PI / 180.0f;
const float EPS = 1e-6f;

const int COLS = config::GRID_COLS;
const int ROWS = config::GRID_ROWS;

struct Cell {
  uint8_t cx;
  uint8_t cy;
};

typedef BitGrid<COLS, ROWS> Map;

// true means blocked or unknown, false means confirmed clear
Map occupancy;

// which body poses fit where, regenerated from occupancy map after each scan
Map reachable;   // room for a full spin
Map turn90;      // room to take a corner between north and east
Map driveUp;     // fits facing north
Map driveEast;   // fits facing east

float centreOf(int index) { return (static_cast<float>(index) + 0.5f) * SUBCELL_CM; }

int clamp(int value, int lowest, int highest) {
  if (value < lowest) return lowest;
  if (value > highest) return highest;
  return value;
}

Cell cellAt(float x, float y) {
  Cell cell;
  cell.cx = static_cast<uint8_t>(clamp(static_cast<int>(floorf(x / SUBCELL_CM)), 0, COLS - 1));
  cell.cy = static_cast<uint8_t>(clamp(static_cast<int>(floorf(y / SUBCELL_CM)), 0, ROWS - 1));
  return cell;
}

bool withinGoal(const Goal& goal, const Vec2& p) {
  return hypotf(goal.x - p.x, goal.y - p.y) <= goal.radius;
}

// --- clearance --------------------------------------------------------------

// Each mask row is one unbroken span, so a whole row of the footprint costs a
// handful of word comparisons rather than one lookup per sub-cell. Rows run
// outwards from the pivot, so an obstacle alongside the robot rejects on the
// first few spans.
bool isClear(int cx, int cy, const masks::Mask& mask) {
  for (int k = 0; k < mask.count; k++) {
    const masks::Row& row = mask.rows[k];
    if (occupancy.anySetInRange(cy + row.dy, cx + row.xMin, cx + row.xMax)) return false;
  }
  return true;
}

// this is potentially over-optimized but whatever:
// A quarter turn passes through both fixed headings and a full turn contains a
// quarter one, so the masks nest and each map implies the ones below it. That
// lets the cheaper test short-circuit the more expensive one
void computeClearance() {
  for (int cy = 0; cy < ROWS; cy++) {
    for (int cx = 0; cx < COLS; cx++) {
      const bool corners = isClear(cx, cy, masks::QUARTER_TURN);
      turn90.set(cx, cy, corners);
      reachable.set(cx, cy, corners && isClear(cx, cy, masks::FULL_TURN));
      driveUp.set(cx, cy, corners || isClear(cx, cy, masks::FACING_NORTH));
      driveEast.set(cx, cy, corners || isClear(cx, cy, masks::FACING_EAST));
    }
  }
}

// --- route planning ---------------------------------------------------------

// A monotone route can turn at most once per row and column, so the count fits
// a byte with room to spare and the top value is free to mean "unreachable".
const uint8_t NO_ROUTE = 255;

// The longest staircase across the board, plus its starting cell.
const int MAX_PATH = COLS + ROWS + 1;

struct Candidate {
  uint8_t i;
  uint8_t j;
  bool arrivedEast;
  uint8_t turns;
  float distanceCm;
};

// Turn costs for the column being swept and the one before it. The recurrence
// only ever looks one column back, so the full width-by-height tables the
// simulator builds would be 22kB of RAM to hold results that are read once.
uint8_t previousEast[ROWS + 1];
uint8_t previousNorth[ROWS + 1];
uint8_t currentEast[ROWS + 1];
uint8_t currentNorth[ROWS + 1];

// What backtracking needs from those tables once they are gone: for each cell
// and arrival heading, whether the cheapest way in turned to get there or
// carried straight on.
Map arrivedEastByTurning;
Map arrivedNorthByTurning;

Cell routePath[MAX_PATH];
Cell routeCorners[MAX_PATH];

uint8_t addTurn(uint8_t turns) {
  return turns == NO_ROUTE ? NO_ROUTE : static_cast<uint8_t>(turns + 1);
}

// Walk the predecessor flags back to the start, recovering each step's arrival
// heading as it goes.
int backtrack(const Cell& start, const Candidate& target) {
  int i = target.i;
  int j = target.j;
  bool arrivedEast = target.arrivedEast;
  int count = 0;

  for (;;) {
    routePath[count].cx = static_cast<uint8_t>(start.cx + i);
    routePath[count].cy = static_cast<uint8_t>(start.cy + j);
    count++;
    if (i == 0 && j == 0) break;
    if (arrivedEast) {
      arrivedEast = !arrivedEastByTurning.get(i, j);
      i--;
    } else {
      arrivedEast = arrivedNorthByTurning.get(i, j);
      j--;
    }
  }

  for (int front = 0, back = count - 1; front < back; front++, back--) {
    const Cell held = routePath[front];
    routePath[front] = routePath[back];
    routePath[back] = held;
  }
  return count;
}

/** Reduce a cell path to its direction changes. */
int cornerCells(const Cell* cells, int count) {
  if (count <= 2) {
    memcpy(routeCorners, cells, static_cast<size_t>(count) * sizeof(Cell));
    return count;
  }
  int corners = 0;
  routeCorners[corners++] = cells[0];
  for (int k = 1; k < count - 1; k++) {
    const bool arrivedEast = cells[k].cx != cells[k - 1].cx;
    const bool leavesEast = cells[k + 1].cx != cells[k].cx;
    if (arrivedEast != leavesEast) routeCorners[corners++] = cells[k];
  }
  routeCorners[corners++] = cells[count - 1];
  return corners;
}

struct Route {
  int cornerCount;
  bool reachesGoal;
};

// search the box between the robot and the far edge of the goal zone, and take
// the fewest-turn route into the goal. Failing that, take a route to the
// nearest vantage point the robot can sweep from.
bool planRoute(IRobot& r, Route& route) {
  const Vec2 here = r.position();
  const Cell start = cellAt(here.x, here.y);
  if (!turn90.get(start.cx, start.cy)) return false;

  const Goal goal = r.goal();
  const Cell goalCentre = cellAt(goal.x, goal.y);
  const int pad = static_cast<int>(ceilf(goal.radius / SUBCELL_CM));
  const int width = clamp(goalCentre.cx + pad, 0, COLS - 1) - start.cx;
  const int height = clamp(goalCentre.cy + pad, 0, ROWS - 1) - start.cy;
  if (width < 0 || height < 0) return false;

  Candidate bestGoal = {};
  Candidate bestVantage = {};
  bool haveGoal = false;
  bool haveVantage = false;

  for (int i = 0; i <= width; i++) {
    for (int j = 0; j <= height; j++) {
      const int cx = start.cx + i;
      const int cy = start.cy + j;

      uint8_t east = NO_ROUTE;
      uint8_t north = NO_ROUTE;
      bool eastTurned = false;
      bool northTurned = false;

      if (i == 0 && j == 0) {
        east = 0;
        north = 0;
      } else {
        if (i > 0 && driveEast.get(cx, cy)) {
          const uint8_t straight = previousEast[j];
          const uint8_t turning =
              turn90.get(cx - 1, cy) ? addTurn(previousNorth[j]) : NO_ROUTE;
          eastTurned = turning < straight;
          east = eastTurned ? turning : straight;
        }
        if (j > 0 && driveUp.get(cx, cy)) {
          const uint8_t straight = currentNorth[j - 1];
          const uint8_t turning =
              turn90.get(cx, cy - 1) ? addTurn(currentEast[j - 1]) : NO_ROUTE;
          northTurned = turning < straight;
          north = northTurned ? turning : straight;
        }
      }

      currentEast[j] = east;
      currentNorth[j] = north;
      arrivedEastByTurning.set(i, j, eastTurned);
      arrivedNorthByTurning.set(i, j, northTurned);

      const uint8_t turns = east <= north ? east : north;
      if (turns == NO_ROUTE) continue;

      Candidate candidate;
      candidate.i = static_cast<uint8_t>(i);
      candidate.j = static_cast<uint8_t>(j);
      candidate.arrivedEast = east <= north;
      candidate.turns = turns;
      candidate.distanceCm = hypotf(goal.x - centreOf(cx), goal.y - centreOf(cy));

      Vec2 centre;
      centre.x = centreOf(cx);
      centre.y = centreOf(cy);
      if (withinGoal(goal, centre)) {
        if (!haveGoal || turns < bestGoal.turns) {
          bestGoal = candidate;
          haveGoal = true;
        }
        continue;
      }

      if (cx > goalCentre.cx || cy > goalCentre.cy) continue;
      if (!turn90.get(cx, cy)) continue;
      if (hypotf(centre.x - here.x, centre.y - here.y) < MIN_STEP_CM) continue;
      if (!haveVantage || candidate.distanceCm < bestVantage.distanceCm - EPS ||
          (candidate.distanceCm < bestVantage.distanceCm + EPS &&
           turns < bestVantage.turns)) {
        bestVantage = candidate;
        haveVantage = true;
      }
    }

    memcpy(previousEast, currentEast, sizeof(previousEast));
    memcpy(previousNorth, currentNorth, sizeof(previousNorth));
  }

  if (!haveGoal && !haveVantage) return false;
  const int pathCount = backtrack(start, haveGoal ? bestGoal : bestVantage);
  route.reachesGoal = haveGoal;
  route.cornerCount = cornerCells(routePath, pathCount);
  return true;
}

// --- sensing ----------------------------------------------------------------

// two rows of the sub-cell corner lattice, reused between calls. Each lattice
// point is shared by four cells, so sweeping row pairs tests it once.
bool cornersBelow[COLS + 2];
bool cornersAbove[COLS + 2];

// helper to get the position of the IR sensor in world space
Vec2 sensorOrigin(IRobot& r) {
  const Vec2 p = r.position();
  const float radians = r.heading() * DEG_TO_RAD;
  const float forwardX = sinf(radians);
  const float forwardY = cosf(radians);
  Vec2 origin;
  origin.x = p.x + forwardX * SENSOR_FORWARD_CM + forwardY * SENSOR_RIGHT_CM;
  origin.y = p.y + forwardY * SENSOR_FORWARD_CM - forwardX * SENSOR_RIGHT_CM;
  return origin;
}

// test if point lies in the sensor zone
// this is called on sub-cell corners during a scan
bool inWedge(float dx, float dy, float forwardX, float forwardY, float reachSq,
             float cosHalfConeSq) {
  const float distanceSq = dx * dx + dy * dy;
  if (distanceSq > reachSq) return false;
  const float along = dx * forwardX + dy * forwardY;
  if (along <= 0.0f) return distanceSq == 0.0f;
  return along * along >= cosHalfConeSq * distanceSq;
}

// clear every sub-cell that lies entirely in the sensor zone (all four corners in sensor zone)
void markFree(IRobot& r) {
  const Vec2 origin = sensorOrigin(r);
  const float measured = r.distance();
  const float reach = isfinite(measured) ? measured - CLEAR_MARGIN_CM : SENSOR_MAX_CM;
  if (reach <= 0.0f) return;

  const float radians = r.heading() * DEG_TO_RAD;
  const float forwardX = sinf(radians);
  const float forwardY = cosf(radians);
  const float reachSq = reach * reach;
  const float cosHalfCone = cosf(SENSOR_HALF_CONE_DEG * DEG_TO_RAD);
  const float cosHalfConeSq = cosHalfCone * cosHalfCone;

  const Cell lo = cellAt(origin.x - reach, origin.y - reach);
  const Cell hi = cellAt(origin.x + reach, origin.y + reach);
  const int width = hi.cx - lo.cx + 2;  // one more corner column than cells

  const int firstColumn = lo.cx;
  bool* below = cornersBelow;
  bool* above = cornersAbove;

  const auto fillRow = [&](bool* row, int latticeRow) {
    const float dy = static_cast<float>(latticeRow) * SUBCELL_CM - origin.y;
    for (int k = 0; k < width; k++) {
      const float dx = static_cast<float>(firstColumn + k) * SUBCELL_CM - origin.x;
      row[k] = inWedge(dx, dy, forwardX, forwardY, reachSq, cosHalfConeSq);
    }
  };

  fillRow(below, lo.cy);
  for (int cy = lo.cy; cy <= hi.cy; cy++) {
    fillRow(above, cy + 1);
    for (int k = 0; k + 1 < width; k++) {
      if (below[k] && below[k + 1] && above[k] && above[k + 1]) {
        occupancy.set(lo.cx + k, cy, false);
      }
    }
    bool* reused = below;
    below = above;
    above = reused;
  }
}

// clear the area around the start, assumes the robot has clearance for a full spin
void clearStartZone(IRobot& r) {
  const Vec2 p = r.position();
  const Cell lo = cellAt(p.x - START_CLEAR_RADIUS_CM, p.y - START_CLEAR_RADIUS_CM);
  const Cell hi = cellAt(p.x + START_CLEAR_RADIUS_CM, p.y + START_CLEAR_RADIUS_CM);
  const float radiusSq = START_CLEAR_RADIUS_CM * START_CLEAR_RADIUS_CM;
  for (int cy = lo.cy; cy <= hi.cy; cy++) {
    for (int cx = lo.cx; cx <= hi.cx; cx++) {
      const float dx = centreOf(cx) - p.x;
      const float dy = centreOf(cy) - p.y;
      if (dx * dx + dy * dy <= radiusSq) occupancy.set(cx, cy, false);
    }
  }
}

// --- map view ---------------------------------------------------------------

// The internal maps drawn live on the brain screen, one pixel per sub-cell,
// with the robot and the goal overlaid so the pose can be read against what
// the robot believes about the board.
//
// Rows are painted as runs of equal cells rather than pixel by pixel. The maps
// are mostly large uniform regions, a swept-clear cone against unknown space,
// so a row usually costs a handful of line draws instead of 120 pixel draws.

enum MapId {
  MAP_OCCUPANCY = 0,
  MAP_REACHABLE,
  MAP_TURN90,
  MAP_DRIVE_UP,
  MAP_DRIVE_EAST,
  MAP_COUNT
};

// Short enough for the strip beside the map.
const char* const MAP_LABELS[MAP_COUNT] = {"occ", "rch", "t90", "dUp", "dEa"};

MapId shownMap = MAP_OCCUPANCY;
uint32_t lastMapDrawMs = 0;

const Map& mapById(MapId id) {
  switch (id) {
    case MAP_REACHABLE: return reachable;
    case MAP_TURN90: return turn90;
    case MAP_DRIVE_UP: return driveUp;
    case MAP_DRIVE_EAST: return driveEast;
    default: return occupancy;
  }
}

// The brain's two navigation buttons cycle through the maps. Edge triggered,
// since this is polled far faster than a button can be released.
void serviceMapButtons() {
  static bool upWasPressed = false;
  static bool downWasPressed = false;

  const bool up = Brain.buttonUp.pressing();
  const bool down = Brain.buttonDown.pressing();
  if (up && !upWasPressed) {
    shownMap = static_cast<MapId>((shownMap + 1) % MAP_COUNT);
  }
  if (down && !downWasPressed) {
    shownMap = static_cast<MapId>((shownMap + MAP_COUNT - 1) % MAP_COUNT);
  }
  upWasPressed = up;
  downWasPressed = down;
}

// Screen y grows downwards and the map's cy grows upwards.
int screenYOf(int cy) { return screen::MAP_Y + (ROWS - 1 - cy); }

void drawMapRows(const Map& map, const vex::color& setColor) {
  Brain.Screen.setPenWidth(1);
  for (int cy = 0; cy < ROWS; cy++) {
    const int y = screenYOf(cy);
    int runStart = 0;
    bool runValue = map.get(0, cy);
    // One past the last column, so the final run is always flushed.
    for (int cx = 1; cx <= COLS; cx++) {
      const bool value = cx < COLS ? map.get(cx, cy) : !runValue;
      if (value == runValue) continue;
      Brain.Screen.setPenColor(runValue ? setColor : vex::color::black);
      Brain.Screen.drawLine(screen::MAP_X + runStart, y, screen::MAP_X + cx - 1,
                            y);
      runStart = cx;
      runValue = value;
    }
  }
}

void drawMapOverlays(IRobot& r) {
  Brain.Screen.setFillColor(vex::color::transparent);

  const Goal goal = r.goal();
  const Cell goalCell = cellAt(goal.x, goal.y);
  Brain.Screen.setPenColor(vex::color::yellow);
  Brain.Screen.drawCircle(screen::MAP_X + goalCell.cx, screenYOf(goalCell.cy),
                          static_cast<int>(goal.radius / SUBCELL_CM));

  // The pivot, plus a stub in the direction the robot is facing.
  const Vec2 p = r.position();
  const Cell here = cellAt(p.x, p.y);
  const int x = screen::MAP_X + here.cx;
  const int y = screenYOf(here.cy);
  const float radians = r.heading() * DEG_TO_RAD;
  const int noseLength = 6;

  Brain.Screen.setPenColor(vex::color::red);
  Brain.Screen.drawCircle(x, y, 2);
  Brain.Screen.drawLine(
      x, y, x + static_cast<int>(sinf(radians) * noseLength),
      y - static_cast<int>(cosf(radians) * noseLength));
}

// Repaints the map, throttled. Safe to call as often as is convenient.
void drawMapView(IRobot& r) {
  const uint32_t now = vex::timer::system();
  if (now - lastMapDrawMs < screen::MAP_REFRESH_MS) return;
  lastMapDrawMs = now;

  serviceMapButtons();

  // A set bit means blocked or unknown in the occupancy map, but means the
  // pose fits in the clearance maps, so the two are coloured differently to
  // keep "white is something to avoid" from reading backwards.
  const vex::color& setColor = shownMap == MAP_OCCUPANCY ? vex::color::white
                                                         : vex::color::green;
  drawMapRows(mapById(shownMap), setColor);
  drawMapOverlays(r);

  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.printAt(screen::SIDE_X, screen::SIDE_NAME_Y, true, "%s",
                       MAP_LABELS[shownMap]);
}

// --- driving ----------------------------------------------------------------

void whileMoving(IRobot& r) {
  while (r.isMoving() && !r.finished()) {
    r.step();
    markFree(r);
    drawMapView(r);
  }
}

void driveScanning(IRobot& r, float cm) {
  if (cm <= MIN_MOTION_CM) return;
  r.startDrive(cm);
  whileMoving(r);
}

void turnScanning(IRobot& r, float headingDeg) {
  r.startTurnTo(headingDeg);
  whileMoving(r);
}

// A settled turn leaves a small residual error, and the heading wraps, so an
// exact comparison would report a turn on every leg of the route.
const float HEADING_EPS_DEG = 1.0f;

bool headingIs(float headingDeg, float targetDeg) {
  float diff = fmodf(fabsf(headingDeg - targetDeg), FULL_TURN_DEG);
  if (diff > FULL_TURN_DEG * 0.5f) diff = FULL_TURN_DEG - diff;
  return diff <= HEADING_EPS_DEG;
}

// The largest arc that can be commanded as a single turn. A turn is commanded
// as an absolute heading, so "here plus a full circle" would be a command to
// stay where it is; anything up to half a circle is unambiguous.
const float SWEEP_LEG_DEG = 90.0f;

// scan an arbitrary number of degrees
//
// One continuous rotation, sampled as it goes, rather than a rotation to each
// heading in turn: the robot is never asked to stop, so the arc costs one
// spin-up instead of hundreds, and whileMoving() reads the sensor throughout.
// Long arcs are split into legs only because of the half-circle limit above,
// and every leg runs the same way round so the robot never doubles back.
void sweep(IRobot& r, float arcDeg) {
  for (float turned = 0.0f; turned < arcDeg - EPS; turned += SWEEP_LEG_DEG) {
    if (r.finished()) return;
    const float legDeg = fminf(SWEEP_LEG_DEG, arcDeg - turned);
    r.startTurnTo(r.heading() + legDeg);
    whileMoving(r);
  }
}

// scan as wide as the current sub-cell has clearance for (based on bitmaps)
void scan(IRobot& r) {
  markFree(r);
  const Vec2 p = r.position();
  const Cell here = cellAt(p.x, p.y);
  if (reachable.get(here.cx, here.cy)) {
    sweep(r, FULL_TURN_DEG);
  } else if (turn90.get(here.cx, here.cy)) {
    turnScanning(r, NORTH_DEG);
    sweep(r, QUARTER_TURN_DEG);
  }
}

// Drive the route, stopping at the first cell a full sweep is possible from so
// as to reveal more of the map. Corners that only permit a quarter turn can be
// traversed on the way to a sweepable position.
void driveRoute(IRobot& r, int cornerCount, LinkedList& turns) {
  for (int k = 1; k < cornerCount; k++) {
    const Cell from = routeCorners[k - 1];
    const Cell to = routeCorners[k];
    const bool drivesEast = to.cx != from.cx;
    const float targetDeg = drivesEast ? EAST_DEG : NORTH_DEG;

    // Record where the robot changes direction, not where it carries straight on.
    if (!headingIs(r.heading(), targetDeg)) {
      const Vec2 p = r.position();
      turns.add(p.x, p.y, !drivesEast);
    }

    turnScanning(r, targetDeg);
    driveScanning(r, drivesEast ? centreOf(to.cx) - r.position().x
                                : centreOf(to.cy) - r.position().y);
    if (r.finished()) return;
    if (reachable.get(to.cx, to.cy)) return;
  }
}

}  // namespace

void run(IRobot& r) {
  occupancy.fill(true);
  clearStartZone(r);
  computeClearance();

  // Every direction change the robot makes while driving a route, in order.
  LinkedList turns;

  // main loop
  for (int step = 0; step < config::MAX_STEPS; step++) {
    scan(r);
    if (r.finished()) return;
    computeClearance();
    // The clearance maps only change here, so this is the one point where
    // showing one of them is worth forcing a repaint for.
    lastMapDrawMs = 0;
    drawMapView(r);
    if (withinGoal(r.goal(), r.position())) {
      r.log("goal reached");
      return;
    }

    Route route;
    if (!planRoute(r, route)) {
      r.log("stuck: no route to the goal or to a new vantage");
      return;
    }
    r.log(route.reachesGoal ? "route to goal" : "route to vantage");
    driveRoute(r, route.cornerCount, turns);
    if (r.finished()) return;
  }
  r.log("step budget exhausted");
}


// --- run setup ---------------------------------------------------------------

// Where the robot is placed on the board and where it is being sent. The
// algorithm has no way to discover either, so both are measured off the board
// beforehand. World cm, origin bottom-left, heading 0 = north.
//
// The route search only ever moves up and to the right, so the goal has to sit
// up and to the right of the start.
namespace {

const float START_X_CM = 10.0f;       // TODO: measure
const float START_Y_CM = 10.0f;       // TODO: measure
const float START_HEADING_DEG = 0.0f; // TODO: measure

const float GOAL_X_CM = 70.0f;        // TODO: measure
const float GOAL_Y_CM = 50.0f;        // TODO: measure
const float GOAL_RADIUS_CM = 5.0f;    // TODO: measure

}  // namespace

int main() {

    Brain.Screen.printAt( 2, 30, "Hello IQ2" );

    Vec2 start;
    start.x = START_X_CM;
    start.y = START_Y_CM;

    Goal goal;
    goal.x = GOAL_X_CM;
    goal.y = GOAL_Y_CM;
    goal.radius = GOAL_RADIUS_CM;

    VexRobot robot(start, START_HEADING_DEG, goal);

    // Calibrates the inertial sensor, so the robot has to be still and on the
    // board by this point.
    robot.begin();

    robot.log("running");
    run(robot);
    robot.log("run over");

    while(1) {

        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}