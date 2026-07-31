#ifndef ROBOT_PROGRAM_H
#define ROBOT_PROGRAM_H

class IRobot;

// explore an unknown board and drive to the goal, returning once the goal is
// reached, the robot is stuck, or the step budget runs out. every map it works
// with is statically allocated in program.cpp, so the only thing this ever puts
// on the heap is the waypoint list.
void run(IRobot& robot);

#endif  // ROBOT_PROGRAM_H
