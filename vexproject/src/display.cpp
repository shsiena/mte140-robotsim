// draws what the algorithm believes about the board, live. watching the maps
// build up beats trying to infer them from how the robot moves.

#include "display.h"

#include <math.h>
#include <stdint.h>

#include "grid.h"
#include "linkedlist.h"
#include "robot.h"
#include "vex.h"

using namespace grid;

namespace {

// one sub-cell is one pixel, centred. the screen is 160x128 and the grid is
// smaller than that in both axes, so 1:1 is the only whole-pixel scale that
// fits without cropping.
const int SCREEN_W = 160;
const int SCREEN_H = 128;
const int MAP_X0 = (SCREEN_W - COLS) / 2;
const int MAP_Y0 = (SCREEN_H - ROWS) / 2;

// repainting the whole map costs far more than a polling tick does, so this
// throttles on a timer.
const uint32_t MAP_REFRESH_MS = 250;
uint32_t lastMapDrawMs = 0;

// print the waypoint list at the end of the program
const bool SHOW_WAYPOINTS = true;
const int WAYPOINT_FIRST_ROW = 1;
const int WAYPOINT_LAST_ROW = 6;

const float PI = 3.14159265358979323846f;
const float DEG_TO_RAD = PI / 180.0f;

// world y runs up the board, screen y runs down the screen.
int screenXOf(int cx) { return MAP_X0 + cx; }
int screenYOf(int cy) { return MAP_Y0 + (ROWS - 1 - cy); }

// every map at once, each sub-cell in the color of the most specific class it
// belongs to:
//
//   black   nothing known yet, or blocked
//   blue    sensor cleared it, but no pose fits
//   cyan    fits facing north only
//   purple  fits facing east only
//   orange  both driving poses fit, but it cannot corner here
//   yellow  room to corner
//   white   room for a full spin
//
// red is the robot and green the goal


// all my homies love C enums
enum CellClass {
  CLASS_UNKNOWN,
  CLASS_CLEAR,
  CLASS_NORTH,
  CLASS_EAST,
  CLASS_BOTH,
  CLASS_CORNER,
  CLASS_SPIN
};

CellClass classOf(int cx, int cy) {
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

const vex::color& inkFor(CellClass cellClass) {
  switch (cellClass) {
    case CLASS_SPIN: return vex::color::white;
    case CLASS_CORNER: return vex::color::yellow;
    case CLASS_BOTH: return vex::color::orange;
    case CLASS_EAST: return vex::color::purple;
    case CLASS_NORTH: return vex::color::cyan;
    default: return vex::color::blue;
  }
}

void drawClassRun(CellClass cellClass, int cx, int cy, int width) {
  // unknown is already the background color
  if (cellClass == CLASS_UNKNOWN) return;
  const vex::color& ink = inkFor(cellClass);
  Brain.Screen.setPenColor(ink);
  Brain.Screen.setFillColor(ink);
  Brain.Screen.drawRectangle(screenXOf(cx), screenYOf(cy), width, 1);
}

void drawRobot(IRobot& r) {
  const Vec2 p = r.position();
  const Cell here = cellAt(p.x, p.y);
  const int x = screenXOf(here.cx);
  const int y = screenYOf(here.cy);

  // short indicator coming out of the pivot showing which way it is facing
  const float radians = r.heading() * DEG_TO_RAD;
  const int tipX = x + static_cast<int>(sinf(radians) * 8.0f);
  const int tipY = y - static_cast<int>(cosf(radians) * 8.0f);
  Brain.Screen.setPenColor(vex::color::yellow);
  Brain.Screen.drawLine(x, y, tipX, tipY);

  Brain.Screen.setPenColor(vex::color::red);
  Brain.Screen.setFillColor(vex::color::red);
  Brain.Screen.drawCircle(x, y, 2);
}

void drawGoal(IRobot& r) {
  const Goal goal = r.goal();
  const Cell centre = cellAt(goal.x, goal.y);
  Brain.Screen.setPenColor(vex::color::green);
  Brain.Screen.setFillColor(vex::color::transparent);
  Brain.Screen.drawCircle(screenXOf(centre.cx), screenYOf(centre.cy),
                          static_cast<int>(goal.radius / SUBCELL_CM));
}

}  // namespace

void invalidateMap() { lastMapDrawMs = 0; }

void drawMap(IRobot& r) {
  const uint32_t now = vex::timer::system();
  if (now - lastMapDrawMs < MAP_REFRESH_MS) return;
  lastMapDrawMs = now;

  Brain.Screen.clearScreen(vex::color::black);

  // runs of equal class go out as one rectangle each. a pixel per draw call
  // puts the frame over 20k calls.
  for (int cy = 0; cy < ROWS; cy++) {
    int runStart = 0;
    CellClass runClass = classOf(0, cy);
    for (int cx = 1; cx <= COLS; cx++) {
      // one past the last column closes whatever run is still open.
      const CellClass cellClass = cx < COLS ? classOf(cx, cy) : CLASS_UNKNOWN;
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

void drawWaypoints(const LinkedList& waypoints) {
  if (!SHOW_WAYPOINTS) return;

  Brain.Screen.clearScreen(vex::color::black);
  Brain.Screen.setPenColor(vex::color::white);

  // printf_float is off for this project, so positions go out in millimetres
  // and headings in whole degrees rather than as floats.
  int row = WAYPOINT_FIRST_ROW;
  for (const Node* node = waypoints.first(); node != nullptr;
       node = node->next) {
    Brain.Screen.setCursor(row, 1);
    Brain.Screen.print("%d,%d %ddeg", static_cast<int>(node->xPos * 10.0f),
                       static_cast<int>(node->yPos * 10.0f),
                       static_cast<int>(node->headingDeg));
    if (++row > WAYPOINT_LAST_ROW) break;
  }
  Brain.Screen.render();
}
