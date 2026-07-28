#ifndef ROBOT_SCREEN_H
#define ROBOT_SCREEN_H

#include <stdint.h>

// How the 160x128 brain screen is divided while debugging. Shared because two
// files draw on it: main.cpp paints the sub-cell maps, robot.cpp writes the
// robot's own status, and neither can see the other's output to know it has
// been overwritten.
//
//   +------------------------+------+
//   |                        | occ  |  which map is shown
//   |   map, 120 x 90        | step |  last entry point called
//   |   one pixel per        | S842 |  polling ticks
//   |   sub-cell             | r437 |  range in mm
//   |                        |      |
//   +------------------------+------+
//   | h452>844 t1m1                 |  heading, turn target, motion flags
//   | route to goal                 |  last log line
//   +-------------------------------+
//
// Text is positioned by pixel rather than by row, since the rows a brain
// screen prints in are taller than the bands left over around the map.

namespace screen {

// The sub-cell grid is config::GRID_COLS by config::GRID_ROWS, which is 120 by
// 90 and so fits the screen at one pixel per sub-cell with room to spare.
const int MAP_X = 0;
const int MAP_Y = 0;

// Strip to the right of the map. Roughly four characters wide.
const int SIDE_X = 123;
const int SIDE_NAME_Y = 16;
const int SIDE_WHERE_Y = 40;
const int SIDE_STEPS_Y = 62;
const int SIDE_RANGE_Y = 84;

// Full-width lines under the map. These are text baselines, so each one sits
// above the coordinate given.
const int STATUS_Y = 108;
const int LOG_Y = 126;

// Redrawing every sub-cell costs far more than a polling tick, so the map is
// repainted on a timer rather than on every call.
const uint32_t MAP_REFRESH_MS = 250;

}  // namespace screen

#endif  // ROBOT_SCREEN_H
