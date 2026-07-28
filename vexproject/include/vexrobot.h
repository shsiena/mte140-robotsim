#ifndef ROBOT_VEXROBOT_H
#define ROBOT_VEXROBOT_H

#include "robot.h"
#include "vex.h"

// The IRobot the algorithm actually runs on: a VEX IQ2 gyro drivetrain plus a
// distance sensor.
//
// The hardware exposes no absolute position, so the pose returned by
// position() is dead reckoned here and nowhere else. Wheel travel comes from
// the drive encoders and heading from the brain's inertial sensor, and the two
// are integrated on every polling tick. Encoder travel is averaged across the
// two sides, so a turn in place contributes nothing to the position: the sides
// spin equal and opposite and cancel.
//
// World frame matches robot.h: origin bottom-left, +x right, +y up, heading in
// degrees with 0 = north (+y) and clockwise positive. That is also the
// convention the inertial sensor reports in, so heading() is a direct reading
// rather than an integration, and it does not accumulate error.
//
// Hardware ports and geometry live in the configuration block at the top of
// src/robot.cpp.

class VexRobot : public IRobot {
 public:
  // The start pose is the algorithm's only fixed point, so it has to be
  // measured off the board and passed in. The goal is known from the start.
  VexRobot(const Vec2& startPosition, float startHeadingDeg, const Goal& goal);

  // Calibrates the inertial sensor and zeroes the odometry against the start
  // pose. Blocks for a couple of seconds and must be called, with the robot
  // held still, before run().
  void begin();

  // --- IRobot ---------------------------------------------------------------
  virtual Vec2 position();
  virtual float heading();
  virtual float distance();
  virtual Goal goal();

  virtual void turn(float deltaDeg);

  virtual void startDrive(float cm);
  virtual void startTurnTo(float headingDeg);
  virtual bool isMoving();
  virtual void step();

  virtual bool finished();

  virtual void log(const char* message);

 private:
  // Total signed wheel travel since begin(), averaged over both sides.
  float odometerCm();
  // Fold the travel and heading accrued since the last call into the pose.
  void updatePose();
  // Drive the live turn one tick closer to its target, or stop it if it has
  // arrived. Turns are closed against the inertial sensor here rather than by
  // the drivetrain: see the note on TURN_CRAWL_PCT in src/robot.cpp.
  void serviceTurn();
  // Poll until the current motion settles, without scanning.
  void settle();
  // Logs anything missing. False if what is missing makes the run pointless.
  bool devicesReady();
  void endRun(const char* reason);

  // Debug. Every entry point the algorithm can call records where it is, so a
  // frozen status line names the last thing that ran: a hang inside the
  // algorithm's own map code shows a stale marker from position() or
  // distance(), while a hang in a motion loop shows one from step().
  void beat(const char* where);
  void drawStatus();
  // A drive that commands motion but produces none has hit something.
  void checkForCollision();
  void checkForGoal();

  // Declaration order matters: the drivetrain binds references to the motors
  // and the sensor, so they have to be constructed first.
  vex::motor leftMotor_;
  vex::motor rightMotor_;
  vex::inertial imu_;
  vex::smartdrive drive_;
  vex::distance range_;
  vex::brain brain_;

  Vec2 position_;
  float headingDeg_;
  Goal goal_;

  float odometerCm_;
  // Travel accrued over the last polling tick, which is what stall detection
  // watches.
  float tickTravelCm_;

  // Absolute heading the live turn is aiming at, whether there is one, and
  // which way round it is going. The direction is latched when the turn is
  // commanded and never revisited: re-deriving it from the live error means
  // the first overshoot reverses the motors, and the robot sits there
  // oscillating around the target instead of settling on it.
  float turnTargetDeg_;
  bool turning_;
  float turnSign_;

  // Odometer reading the live drive is aiming at. A drive is finished when the
  // odometer arrives, which is a reading this class owns, rather than when the
  // drivetrain says so.
  float driveTargetCm_;

  // When the current motion was commanded. The drivetrain does not report
  // itself as moving until the motors actually spin up, so a motion is treated
  // as live for a short grace period regardless of what it reports.
  uint32_t motionStartMs_;
  // Whether the live motion is a drive. Turns are exempt from stall detection,
  // since a turn in place is expected to produce no travel.
  bool motionIsDrive_;
  int stalledTicks_;

  bool finished_;
  int logRow_;

  const char* where_;
  uint32_t turnCount_;
  uint32_t stepCount_;
  uint32_t rangeCount_;
  uint32_t lastDebugMs_;
};

#endif  // ROBOT_VEXROBOT_H
