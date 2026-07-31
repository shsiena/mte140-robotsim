#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// centimetres and degrees throughout, so every value maps 1:1 onto the real
// robot. doubles because the swept-footprint traces in masks.h run at compile
// time and have to land on the same sub-cells the simulator's double-precision
// arithmetic did. none of them reach the robot as a double: they fold into
// constants, and program.cpp takes float copies for anything evaluated at
// runtime.

namespace config {

constexpr double CELL_CM = 2.0;

// subdivisions along each edge of a board cell in the robot's internal maps.
// map memory scales as the square of this and a clearance sweep as the fourth
// power, so it is fixed here rather than tunable at runtime.
constexpr int GRID_RESOLUTION_PER_CELL = 3;
constexpr double SUBCELL_CM = CELL_CM / GRID_RESOLUTION_PER_CELL;

// we sized these off an estimate of the cell dimensions, since we never had a
// ruler on the board. they came out smaller than the real thing, so obstacles
// past the modelled edge are outside every map the robot keeps. bitgrid reads
// off-grid as false, and false means clear, so the clearance masks sweep
// straight over anything out there.
constexpr int BOARD_COLS = 30;
constexpr int BOARD_ROWS = 20;
constexpr int GRID_COLS = BOARD_COLS * GRID_RESOLUTION_PER_CELL;
constexpr int GRID_ROWS = BOARD_ROWS * GRID_RESOLUTION_PER_CELL;

constexpr double ROBOT_LENGTH_CM = 22.0;
constexpr double ROBOT_WIDTH_CM = 16.0;

// we estimated the robot turns about a point two thirds of the way forward rather than about its
// centre, so the rear overhang sets the turning circle. that pivot point is what every coordinate refers to
constexpr double PIVOT_FROM_REAR_CM = ROBOT_LENGTH_CM * 2.0 / 3.0;
constexpr double PIVOT_TO_FRONT_CM = ROBOT_LENGTH_CM - PIVOT_FROM_REAR_CM;

// where the ir sensor sits in the robot's frame, relative to the pivot.
constexpr double SENSOR_FORWARD_CM = PIVOT_TO_FRONT_CM;
constexpr double SENSOR_RIGHT_CM = 0.0;
constexpr double SENSOR_CONE_DEG = 24.19;
constexpr double SENSOR_HALF_CONE_DEG = SENSOR_CONE_DEG / 2.0;

// how far out we trust the sensor. the hardware reaches roughly 80in, but we deliberately capped this 
// to try and account for larger error at longer distances 
constexpr double SENSOR_TRUST_CM = 80.0;

// padding added to the body on every side before it is tested against the map,
// to give a margin of error before the robot bumps an obstacle
constexpr double SAFETY_CM = 1.0;

// trimmed off a range reading before the space in front of it is cleared
// this is deliberately conservative, but we would rather be overly cautious than bump into an obstacle
constexpr double CLEAR_MARGIN_CM = 1.5;

// shortest move worth planning
constexpr double MIN_STEP_CM = CELL_CM;
constexpr double MIN_MOTION_CM = 0.01;
constexpr int MAX_STEPS = 500;

}  // namespace config

#endif  // ROBOT_CONFIG_H
