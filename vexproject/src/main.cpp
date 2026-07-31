/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       Group 26                                                  */
/*    Description:  MTE 140 grid traversal, IQ2                               */
/*----------------------------------------------------------------------------*/

#include "vex.h"

#include "program.h"
#include "robot.h"
#include "vexrobot.h"

vex::brain Brain;

namespace {

// where the robot is placed on the board and where it is being sent. it has no
// way to discover either, so both get measured off the board beforehand. world
// cm, origin bottom-left, heading 0 = north.
//
// the route search only ever moves up and to the right, so the goal has to sit
// up and to the right of the start.
//
// none of these were ever measured properly. the goal below is outside the
// board as config.h has it sized, so cellAt() clamps it to the top right corner
// and withinGoal() can never be satisfied. that needs a tape measure and a
// board, which is what we ran out of.
const float START_X_CM = 10.0f;
const float START_Y_CM = 10.0f;
const float START_HEADING_DEG = 0.0f;

const float GOAL_X_CM = 70.0f;
const float GOAL_Y_CM = 50.0f;
const float GOAL_RADIUS_CM = 5.0f;

}  // namespace

int main() {
  Vec2 start;
  start.x = START_X_CM;
  start.y = START_Y_CM;

  Goal goal;
  goal.x = GOAL_X_CM;
  goal.y = GOAL_Y_CM;
  goal.radius = GOAL_RADIUS_CM;

  VexRobot robot(start, START_HEADING_DEG, goal);

  // calibrates the inertial sensor, the robot has to be still and on the
  // board by the time this runs.
  robot.begin();

  robot.log("running");
  run(robot);
  robot.log("run over");

  while (true) {
    vex::this_thread::sleep_for(10);
  }
}
