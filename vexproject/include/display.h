#ifndef ROBOT_DISPLAY_H
#define ROBOT_DISPLAY_H

class IRobot;
class LinkedList;

// draws the robot's internal maps on the brain screen. throttled internally, so
// it is safe to call as often as is convenient from inside a motion loop
void drawMap(IRobot& robot);

// forces the next drawMap() to repaint
void invalidateMap();

// prints the waypoint trail as text. off by default, see SHOW_WAYPOINTS
void drawWaypoints(const LinkedList& waypoints);

#endif  // ROBOT_DISPLAY_H
