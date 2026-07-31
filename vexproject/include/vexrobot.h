#ifndef ROBOT_VEXROBOT_H
#define ROBOT_VEXROBOT_H

#include "robot.h"
#include "vex.h"

// the IRobot the algorithm actually runs on: an IQ2 gyro drivetrain plus a
// distance sensor.
//
// the hardware exposes no absolute position. the pose behind position() is dead
// reckoned here and nowhere else: wheel travel from the drive encoders, heading
// from the inertial sensor, both integrated on every polling tick. encoder
// travel is averaged across the two sides, which makes a turn in place
// contribute nothing, since the sides spin equal and opposite and cancel.
//
// heading 0 = north, clockwise positive. that is also what the inertial sensor
// reports, which makes heading() a reading rather than a running total. it does
// not accumulate error.
//
// ports and geometry live in the block at the top of robot.cpp.

class VexRobot : public IRobot {
  public:
    VexRobot(const Vec2& startPosition, float startHeadingDeg, const Goal& goal);

    // calibrates the inertial sensor and zeroes the odometry against the start
    // pose. blocks for a couple of seconds and has to be called with the robot
    // held still, before run().
    void begin();

    virtual Vec2 position();
    virtual float heading();
    virtual float distance();
    virtual Goal goal();

    virtual void startDrive(float cm);
    virtual void startTurnTo(float headingDeg);
    virtual bool isMoving();
    virtual void step();

    virtual bool finished();

    virtual void log(const char* message);

  private:
    float odometerCm();
    void updatePose();
    // nudge the live turn one tick closer to its target, or stop it if it has
    // arrived. see the note on TURN_CRAWL_PCT in robot.cpp for why turns are
    // closed here rather than by the drivetrain.
    void serviceTurn();
    bool devicesReady();
    void endRun(const char* reason);
    void checkForCollision();
    void checkForGoal();

    // declaration order matters. the drivetrain binds references to the motors
    // and the sensor, which have to exist by the time it is constructed.
    vex::motor leftMotor_;
    vex::motor rightMotor_;
    vex::inertial imu_;
    vex::smartdrive drive_;
    vex::distance range_;

    Vec2 position_;
    float headingDeg_;
    Goal goal_;

    float odometerCm_;
    float tickTravelCm_;

    // absolute heading the live turn is aiming at, whether there is one, and
    // which way round it is going. the direction is latched when the turn is
    // commanded and never revisited: re-deriving it from the live error means the
    // first overshoot reverses the motors and it oscillates instead of settling.
    float turnTargetDeg_;
    bool turning_;
    float turnSign_;

    float driveTargetCm_;

    uint32_t motionStartMs_;
    // whether the live motion is a drive. turns are exempt from stall detection,
    // since a turn in place is supposed to produce no travel.
    bool motionIsDrive_;
    int stalledTicks_;

    bool finished_;
    int logRow_;
};

#endif  // ROBOT_VEXROBOT_H
