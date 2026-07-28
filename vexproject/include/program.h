#ifndef ROBOT_PROGRAM_H
#define ROBOT_PROGRAM_H

class IRobot;

// Explore an unknown board and drive to the goal, returning once the goal is
// reached, the robot is stuck, or the step budget runs out. All of the maps it
// works with are statically allocated inside program.cpp; nothing is allocated
// here or on the caller's stack.
void run(IRobot& robot);

#endif  // ROBOT_PROGRAM_H
