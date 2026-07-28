// Explores an unknown board and drives to the goal using only northward and
// eastward travel.
//
// Three layers sit on top of each other:
//
//   1. occupancy    True means blocked OR unknown; markFree only ever clears,
//                   so knowledge grows and never retracts.
//   2. clearance    four maps saying which body poses fit where, each built by
//                   testing occupancy against a precomputed swept-footprint
//                   mask.
//   3. route        a dynamic program over the clearance maps that minimises
//                   90 degree turns, since every monotone route is the same
//                   length and turns are the only cost that varies.
//

#include "program.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "bitgrid.h"
#include "config.h"
#include "masks.h"
#include "robot.h"

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

// --- driving ----------------------------------------------------------------

void whileMoving(IRobot& r) {
  while (r.isMoving() && !r.finished()) {
    r.step();
    markFree(r);
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

// scan an arbitrary number of degrees
void sweep(IRobot& r, float arcDeg) {
  for (float turned = 0.0f; turned < arcDeg; turned += SWEEP_STEP_DEG) {
    if (r.finished()) return;
    r.turn(SWEEP_STEP_DEG);
    markFree(r);
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
void driveRoute(IRobot& r, int cornerCount) {
  for (int k = 1; k < cornerCount; k++) {
    const Cell from = routeCorners[k - 1];
    const Cell to = routeCorners[k];
    if (to.cx != from.cx) {
      turnScanning(r, EAST_DEG);
      driveScanning(r, centreOf(to.cx) - r.position().x);
    } else {
      turnScanning(r, NORTH_DEG);
      driveScanning(r, centreOf(to.cy) - r.position().y);
    }
    if (r.finished()) return;
    if (reachable.get(to.cx, to.cy)) return;
  }
}

}  // namespace

void run(IRobot& r) {
  occupancy.fill(true);
  clearStartZone(r);
  computeClearance();

  // main loop
  for (int step = 0; step < config::MAX_STEPS; step++) {
    scan(r);
    if (r.finished()) return;
    computeClearance();
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
    driveRoute(r, route.cornerCount);
    if (r.finished()) return;
  }
  r.log("step budget exhausted");
}
