#ifndef ROBOT_ROBOT_H
#define ROBOT_ROBOT_H

// The hardware contract the algorithm is written against. The simulator bridge
// and the VEX implementation both satisfy it, so run() is compiled once and
// linked against whichever one the binary needs.
//
// Every method here is called on the order of hundreds of times per run, never
// per sub-cell, which is why virtual dispatch is affordable. The occupancy and
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
  // Pivot point in world cm. Origin bottom-left, +x right, +y up.
  virtual Vec2 position() = 0;
  // Degrees, 0 = north (+y), clockwise positive.
  virtual float heading() = 0;
  // IR range in cm, or INFINITY when nothing is in the cone.
  virtual float distance() = 0;
  // Known from the start. Success is the pivot within radius of the centre.
  virtual Goal goal() = 0;

  // --- blocking motion ------------------------------------------------------
  // Rotate in place by a relative amount, + clockwise, returning once settled.
  virtual void turn(float deltaDeg) = 0;

  // --- polled motion --------------------------------------------------------
  // Begin driving along the current heading, + forward. Returns immediately so
  // the caller can keep reading the sensor while the robot moves.
  virtual void startDrive(float cm) = 0;
  // Begin rotating in place towards an absolute heading, shortest direction.
  virtual void startTurnTo(float headingDeg) = 0;
  virtual bool isMoving() = 0;
  // Advance one polling tick, refreshing pose and range.
  virtual void step() = 0;

  // True once the run has ended underneath the algorithm, by collision or by
  // the goal being captured mid-motion. Every motion loop checks it.
  virtual bool finished() = 0;

  virtual void log(const char* message) = 0;
};

#endif  // ROBOT_ROBOT_H
