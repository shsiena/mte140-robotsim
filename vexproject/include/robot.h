#ifndef ROBOT_ROBOT_H
#define ROBOT_ROBOT_H

// the hardware contract the algorithm is written against, so run() never talks
// to a motor directly.
//
// nothing here is called per sub-cell, only on the order of hundreds of times a
// run, which is what makes virtual dispatch affordable. the occupancy and
// clearance maps deliberately do not appear: they are the algorithm's own
// memory, not something the hardware provides.

struct Vec2 {
  float x;
  float y;
};

struct Goal {
  float x;
  float y;
  float radius;
};

class IRobot {
 public:
  virtual ~IRobot() {}

  // --- sensing --------------------------------------------------------------
  // pivot point in world cm. origin bottom-left, +x right, +y up.
  virtual Vec2 position() = 0;
  // degrees, 0 = north (+y), clockwise positive.
  virtual float heading() = 0;
  // ir range in cm, or INFINITY when nothing is in the cone.
  virtual float distance() = 0;
  // known from the start. success is the pivot within radius of the centre.
  virtual Goal goal() = 0;

  // --- motion ---------------------------------------------------------------
  // both of these return immediately so the caller can keep reading the sensor
  // while the robot moves. drive runs along the current heading, + forward.
  virtual void startDrive(float cm) = 0;
  // absolute heading, shortest way round.
  virtual void startTurnTo(float headingDeg) = 0;
  virtual bool isMoving() = 0;
  // advance one polling tick, refreshing pose and range.
  virtual void step() = 0;

  // true once the run has ended underneath the algorithm, by collision or by
  // the goal being captured mid-motion. every motion loop checks it.
  virtual bool finished() = 0;

  virtual void log(const char* message) = 0;
};

#endif  // ROBOT_ROBOT_H
