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

// --- map display ------------------------------------------------------------

// Draws what the algorithm believes about the board, live, so its internal
// state can be watched rather than inferred. One sub-cell is one pixel: the
// screen is 160x128 and the grid is 120x90, so the largest whole-pixel scale
// that fits is 1:1, centred.

const int SCREEN_W = 160;
const int SCREEN_H = 128;
const int MAP_X0 = (SCREEN_W - COLS) / 2;
const int MAP_Y0 = (SCREEN_H - ROWS) / 2;

// Redrawing the whole map costs far more than a polling tick does, so the view
// is throttled rather than redrawn on every one.
const uint32_t MAP_REFRESH_MS = 250;
uint32_t lastMapDrawMs = 0;

// World y runs up the board, screen y runs down the screen.
int screenXOf(int cx) { return MAP_X0 + cx; }
int screenYOf(int cy) { return MAP_Y0 + (ROWS - 1 - cy); }

// Every map the algorithm keeps, drawn at once, each in its own colour.
//
// They nest: room for a full spin implies room to corner, and room to corner
// implies both driving poses fit. A pixel can only be one colour, so each
// sub-cell is drawn as the most specific class it belongs to, most specific
// first.
//
//   black   nothing known yet, or blocked          occupancy set
//   blue    sensor cleared it, no pose fits
//   cyan    fits facing north only                 driveUp
//   purple  fits facing east only                  driveEast
//   orange  both driving poses fit, cannot corner
//   yellow  room to corner                         turn90
//   white   room for a full spin                   reachable
//
// Red is the robot and green the goal, so neither appears in the map palette.
const int CLASS_UNKNOWN = 0;
const int CLASS_CLEAR = 1;
const int CLASS_NORTH = 2;
const int CLASS_EAST = 3;
const int CLASS_BOTH = 4;
const int CLASS_CORNER = 5;
const int CLASS_SPIN = 6;

int classOf(int cx, int cy) {
  if (occupancy.get(cx, cy)) return CLASS_UNKNOWN;
  if (reachable.get(cx, cy)) return CLASS_SPIN;
  if (turn90.get(cx, cy)) return CLASS_CORNER;
  const bool north = driveUp.get(cx, cy);
  const bool east = driveEast.get(cx, cy);
  if (north && east) return CLASS_BOTH;
  if (north) return CLASS_NORTH;
  if (east) return CLASS_EAST;
  return CLASS_CLEAR;
}

const color& inkFor(int cellClass) {
  switch (cellClass) {
    case CLASS_SPIN: return color::white;
    case CLASS_CORNER: return color::yellow;
    case CLASS_BOTH: return color::orange;
    case CLASS_EAST: return color::purple;
    case CLASS_NORTH: return color::cyan;
    default: return color::blue;
  }
}

void drawClassRun(int cellClass, int cx, int cy, int width) {
  // Unknown is the background, so it is never drawn: on a board the robot has
  // barely seen, almost nothing costs anything to render.
  if (cellClass == CLASS_UNKNOWN) return;
  const color& ink = inkFor(cellClass);
  Brain.Screen.setPenColor(ink);
  Brain.Screen.setFillColor(ink);
  Brain.Screen.drawRectangle(screenXOf(cx), screenYOf(cy), width, 1);
}

void drawRobot(IRobot& r) {
  const Vec2 p = r.position();
  const Cell here = cellAt(p.x, p.y);
  const int x = screenXOf(here.cx);
  const int y = screenYOf(here.cy);

  // A tick showing which way it is facing, drawn from the pivot outwards.
  const float radians = r.heading() * DEG_TO_RAD;
  const int tipX = x + static_cast<int>(sinf(radians) * 8.0f);
  const int tipY = y - static_cast<int>(cosf(radians) * 8.0f);
  Brain.Screen.setPenColor(color::yellow);
  Brain.Screen.drawLine(x, y, tipX, tipY);

  Brain.Screen.setPenColor(color::red);
  Brain.Screen.setFillColor(color::red);
  Brain.Screen.drawCircle(x, y, 2);
}

void drawGoal(IRobot& r) {
  const Goal goal = r.goal();
  const Cell centre = cellAt(goal.x, goal.y);
  Brain.Screen.setPenColor(color::green);
  Brain.Screen.setFillColor(color::transparent);
  Brain.Screen.drawCircle(screenXOf(centre.cx), screenYOf(centre.cy),
                          static_cast<int>(goal.radius / SUBCELL_CM));
}

void drawMap(IRobot& r) {
  const uint32_t now = vex::timer::system();
  if (now - lastMapDrawMs < MAP_REFRESH_MS) return;
  lastMapDrawMs = now;

  Brain.Screen.clearScreen(color::black);

  // Runs of equal class go out as one rectangle each. A map row is 120 pixels
  // wide, and drawing it a pixel at a time would be tens of thousands of calls
  // a frame.
  for (int cy = 0; cy < ROWS; cy++) {
    int runStart = 0;
    int runClass = classOf(0, cy);
    for (int cx = 1; cx <= COLS; cx++) {
      // One past the last column closes whatever run is open.
      const int cellClass = cx < COLS ? classOf(cx, cy) : CLASS_UNKNOWN;
      if (cellClass == runClass) continue;
      drawClassRun(runClass, runStart, cy, cx - runStart);
      runStart = cx;
      runClass = cellClass;
    }
  }

  drawGoal(r);
  drawRobot(r);
  Brain.Screen.render();
}

// --- driving ----------------------------------------------------------------

void whileMoving(IRobot& r) {
  while (r.isMoving() && !r.finished()) {
    r.step();
    markFree(r);
    drawMap(r);
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
  drawMap(r);

  // Every direction change the robot makes while driving a route, in order.
  LinkedList turns;

  // main loop
  for (int step = 0; step < config::MAX_STEPS; step++) {
    scan(r);
    if (r.finished()) return;
    computeClearance();
    // The derived maps have just been rebuilt, so this is the one redraw that
    // shows the planner's view rather than the sensor's.
    lastMapDrawMs = 0;
    drawMap(r);
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