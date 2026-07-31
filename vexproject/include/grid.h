#ifndef ROBOT_GRID_H
#define ROBOT_GRID_H

#include <math.h>
#include <stdint.h>

#include "bitgrid.h"
#include "config.h"

// the sub-cell grid the robot thinks in, plus what it currently believes about
// the board. program.cpp writes these, display.cpp reads them.

namespace grid {

const int COLS = config::GRID_COLS;
const int ROWS = config::GRID_ROWS;

const float SUBCELL_CM = static_cast<float>(config::SUBCELL_CM);

typedef BitGrid<COLS, ROWS> Map;

struct Cell {
  uint8_t cx;
  uint8_t cy;
};


// I know I know, global variables bad...
// however, consider the following:
//  -> we are writing C-style C++ for an extremely memory-limited embedded
//     platform
//  -> extra classes and heap allocated objects must be avoided at all costs
//
//  ...also clean code is a myth, it either works or it doesn't ¯\_(ツ)_/¯
//
//  hot take: global variable bugs are a skill issue, I paid for the whole
//  language, I'm going to use the whole language
//  - Simon

// true means blocked or unknown, false means confirmed clear. the map starts
// fully obstructed and markFree clears open space as the sensor sees it.
extern Map occupancy;

// which body poses fit where, rebuilt from occupancy after every scan.
extern Map reachable;  // room for a full spin
extern Map turn90;     // room to take a corner between north and east
extern Map driveUp;    // fits facing north
extern Map driveEast;  // fits facing east

// inlined to reduce call stack frames
inline int clamp(int value, int lowest, int highest) {
  if (value < lowest) return lowest;
  if (value > highest) return highest;
  return value;
}

// inlined to reduce call stack frames
inline float centreOf(int index) {
  return (static_cast<float>(index) + 0.5f) * SUBCELL_CM;
}

// this clamps. it does not report out of range, which means a pose that drifts
// off the modelled board quietly reads as the nearest edge cell. this could be
// a problem but it is what it is for now
inline Cell cellAt(float x, float y) {
  Cell cell;
  cell.cx = static_cast<uint8_t>(
      clamp(static_cast<int>(floorf(x / SUBCELL_CM)), 0, COLS - 1));
  cell.cy = static_cast<uint8_t>(
      clamp(static_cast<int>(floorf(y / SUBCELL_CM)), 0, ROWS - 1));
  return cell;
}

}  // namespace grid

#endif  // ROBOT_GRID_H
