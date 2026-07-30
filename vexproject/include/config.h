#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// Mirrors src/config.ts. Centimetres and degrees throughout, so every value
// maps 1:1 onto the physical VEX IQ robot.
//
// These are double precision because the swept-footprint traces in masks.h run
// at compile time and have to land on the same sub-cells the simulator's
// double-precision arithmetic does. Nothing here reaches the robot as a double:
// they all fold into constants, and program.cpp takes single-precision working
// copies for the expressions it evaluates at runtime.

namespace config {

constexpr double CELL_CM = 2.0;

// Subdivisions along each edge of a board cell in the robot's internal maps.
// Map memory scales as the square of this and a clearance sweep as the fourth
// power, so it is fixed here rather than being configurable at runtime.
constexpr int GRID_RESOLUTION_PER_CELL = 3;
constexpr double SUBCELL_CM = CELL_CM / GRID_RESOLUTION_PER_CELL;

constexpr int BOARD_COLS = 30;
constexpr int BOARD_ROWS = 20;
constexpr int GRID_COLS = BOARD_COLS * GRID_RESOLUTION_PER_CELL;
constexpr int GRID_ROWS = BOARD_ROWS * GRID_RESOLUTION_PER_CELL;

// A rectangle with a heading. "Length" runs along the heading.
constexpr double ROBOT_LENGTH_CM = 22.0;
constexpr double ROBOT_WIDTH_CM = 16.0;

// The robot turns in place about a point two thirds of the way forward, not
// about the centre of its body, so the rear overhang sets the turning circle.
// That pivot is the pose every coordinate here refers to.
constexpr double PIVOT_FROM_REAR_CM = ROBOT_LENGTH_CM * 2.0 / 3.0;
constexpr double PIVOT_TO_FRONT_CM = ROBOT_LENGTH_CM - PIVOT_FROM_REAR_CM;

// IR sensor mounting in the robot's frame, relative to the pivot.
constexpr double SENSOR_FORWARD_CM = PIVOT_TO_FRONT_CM;
constexpr double SENSOR_RIGHT_CM = 0.0;
constexpr double SENSOR_CONE_DEG = 24.19;
constexpr double SENSOR_HALF_CONE_DEG = SENSOR_CONE_DEG / 2.0;
constexpr double SENSOR_MAX_CM = 80;

// Clearance the body is padded by on every side before it is tested against
// the map, absorbing odometry drift and sensor quantisation.
constexpr double SAFETY_CM = 1.0;

// Trimmed off a range reading before the space in front of it is marked free,
// so a sub-cell straddling the reflecting surface is never cleared.
constexpr double CLEAR_MARGIN_CM = 1.5;

constexpr double SWEEP_STEP_DEG = 1.0;
constexpr double MIN_STEP_CM = CELL_CM;
constexpr double MIN_MOTION_CM = 0.01;
constexpr int MAX_STEPS = 500;

}  // namespace config

#endif  // ROBOT_CONFIG_H
