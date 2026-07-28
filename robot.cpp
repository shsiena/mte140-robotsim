#ifndef ROBOT_ROBOT_H
#define ROBOT_ROBOT_H

#include <math.h>

#include "robot.h"

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
    // startX/startY are the robot pivot's known starting coordinates in cm.
    IRobot(float startX = 0.0f, float startY = 0.0f)
        : position_{startX, startY},
          lastLeftDeg_(0.0f),
          lastRightDeg_(0.0f),
          lastHeadingDeg_(0.0f),
          odometryInitialized_(false) {}

    virtual ~IRobot() {}

    // --- sensing --------------------------------------------------------------
    // Pivot point in world cm. Origin bottom-left, +x right, +y up.
    virtual Vec2 position() {
        const float leftDeg = LeftDriveSmart.position(degrees);
        const float rightDeg = RightDriveSmart.position(degrees);
        const float currentHeadingDeg = BrainInertial.heading();

        // The first call establishes the encoder reference without inventing
        // movement that may have happened before this object was constructed.
        if (!odometryInitialized_) {
            lastLeftDeg_ = leftDeg;
            lastRightDeg_ = rightDeg;
            lastHeadingDeg_ = currentHeadingDeg;
            odometryInitialized_ = true;
            return position_;
        }

        const float leftDeltaDeg = leftDeg - lastLeftDeg_;
        const float rightDeltaDeg = rightDeg - lastRightDeg_;
        const float distanceCm =
            0.5f * (leftDeltaDeg + rightDeltaDeg) *
            (WHEEL_TRAVEL_CM / 360.0f);

        // Use the heading halfway through this encoder interval. This is more
        // accurate than using only the old or new heading while driving an arc.
        float headingDeltaDeg = currentHeadingDeg - lastHeadingDeg_;
        if (headingDeltaDeg > 180.0f) {
            headingDeltaDeg -= 360.0f;
        } else if (headingDeltaDeg < -180.0f) {
            headingDeltaDeg += 360.0f;
        }

        const float midHeadingRad =
            (lastHeadingDeg_ + 0.5f * headingDeltaDeg) * DEG_TO_RAD;

        // Heading 0 is north (+y), and positive headings turn clockwise.
        position_.x += distanceCm * sinf(midHeadingRad);
        position_.y += distanceCm * cosf(midHeadingRad);

        lastLeftDeg_ = leftDeg;
        lastRightDeg_ = rightDeg;
        lastHeadingDeg_ = currentHeadingDeg;
        return position_;
    }
    // Degrees, 0 = north (+y), clockwise positive.
    virtual float heading() {
        return BrainInertial.heading();
    }
    // IR range in cm, or INFINITY when nothing is in the cone.
    virtual float distance() {
        return Distance4.objectDistance(mm)/10;
    };
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

private:
    // VEXcode's standard IQ drivetrain setting uses 200 mm of wheel travel
    // (wheel circumference), which is 20 cm. Change this if yours is different.
    static constexpr float WHEEL_TRAVEL_CM = 20.0f;
    static constexpr float DEG_TO_RAD = 0.01745329251994329577f;

    Vec2 position_;
    float lastLeftDeg_;
    float lastRightDeg_;
    float lastHeadingDeg_;
    bool odometryInitialized_;
};

#endif  // ROBOT_ROBOT_H
