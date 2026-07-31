#include "vexrobot.h"

#include <math.h>

#include "config.h"

namespace {

// --- hardware ----------------------------------------------------------------
// edit this block to match the built robot. everything else in this file is
// written against these values. none of them were ever confirmed against the
// real build, so if the robot sits there doing nothing, start here: begin()
// gives up when a device does not report itself installed.

const int32_t LEFT_MOTOR_PORT = vex::PORT1;
const int32_t RIGHT_MOTOR_PORT = vex::PORT6;
const int32_t DISTANCE_SENSOR_PORT = vex::PORT4;

// one side is mounted facing the other way, so its encoder and command sense
// both flip for "forward" to mean the same thing on both.
const bool LEFT_MOTOR_REVERSED = false;
const bool RIGHT_MOTOR_REVERSED = true;

// how far one wheel rolls in a full turn, the distance between the driven
// wheels, and the front-to-rear wheel spacing. estimated, never measured.
const double WHEEL_TRAVEL_CM = 20.0;
const double TRACK_WIDTH_CM = 20.0;
const double WHEEL_BASE_CM = 5.0;

// motor revolutions per wheel revolution. 1.0 for a direct drive, which is also
// the case where the drivetrain's gear-ratio convention cannot disagree with
// odometerCm() below. if the build is geared and driveFor() distances come out
// scaled, invert this in odometerCm().
const double EXTERNAL_GEAR_RATIO = 1.0;

// slow enough that the sub-cell map is not outrun between ticks.
//
// turn velocity also sets how finely a sweep is sampled, since the sensor is
// read once per tick throughout: halving it doubles the readings per degree of
// arc. if a turn ever fails to start from rest, this has gone under what it
// takes to break static friction.
const double DRIVE_VELOCITY_PCT = 40.0;
const double TURN_VELOCITY_PCT = 12.0;

// ceiling on any one leg. without it a jammed wheel hangs the run indefinitely.
// the longest leg the planner can produce is the board diagonal.
const int32_t MOTION_TIMEOUT_SEC = 10;

// turns are closed against the inertial sensor here rather than handed to
// smartdrive::turnToHeading, which scales motor velocity by the heading error.
// near the target that comes out under what it takes to break static friction:
// the motors are commanded, nothing rotates, and the turn sits there until it
// times out. a fixed velocity with a floor under it does not stall.
const double TURN_CRAWL_PCT = 9.0;
// error below which the crawl velocity is used instead of the full one.
const double TURN_APPROACH_DEG = 10.0;
const float TURN_TOLERANCE_DEG = 0.5f;

// generous, because it has to clear the slowest legitimate turn, which is a 90
// degree sweep leg at TURN_VELOCITY_PCT. tripping this ends the run outright,
// so it is set to catch a jammed robot rather than to bound a slow one.
const uint32_t TURN_TIMEOUT_MS = 12000;

// the inertial sensor keeps reporting the previous calibration state for a
// moment after one is requested, so the wait has to start after a short delay
// or it falls straight through.
const uint32_t IMU_SETTLE_MS = 100;
const uint32_t IMU_CALIBRATE_TIMEOUT_MS = 5000;

// --- polling -----------------------------------------------------------------

// the sampling rate of the whole system. one sensor reading, one pose update
// and one map update per tick.
//
// this is what we ask for, not what we get. markFree() costs far more per call
// than the sleep does, so the real tick period is however long that takes.
const uint32_t POLL_MS = 5;

// the drivetrain reports itself stopped in the window between a motion being
// commanded and the motors spinning up, which would end a poll loop before it
// began.
const uint32_t MOTION_GRACE_MS = 60;

// a drive making less headway than this, for this long, is pushing against
// something. neither value is critical: the point is to end the run rather than
// grind, and to stay well clear of what the drivetrain produces while it is
// still accelerating.
const float STALL_SPEED_CM_PER_S = 2.0f;
const uint32_t STALL_WINDOW_MS = 250;

const float STALL_TRAVEL_CM =
    STALL_SPEED_CM_PER_S * static_cast<float>(POLL_MS) / 1000.0f;
const int STALL_TICKS = static_cast<int>(STALL_WINDOW_MS / POLL_MS);

// --- logging -----------------------------------------------------------------

// the map display owns the screen, so log lines go out over usb only. turn this
// on and it draws over the map. worse, drawMap() calls render(), which puts the
// screen into double buffering: text written between frames waits for the next
// one before it appears at all.
const bool LOG_TO_SCREEN = false;
const int LOG_FIRST_ROW = 1;
const int LOG_LAST_ROW = 6;

// last-resort backstop. nothing the algorithm asks for legitimately takes this
// long: the drivetrain gives up on a drive after MOTION_TIMEOUT_SEC and a turn
// after TURN_TIMEOUT_MS. past this the motion gets declared over and the poll
// loop ends. it sits above TURN_TIMEOUT_MS deliberately, otherwise a stuck turn
// would trip this instead of being reported as the stuck turn it is.
const uint32_t MOTION_CAP_MS = 20000;

// how near the commanded odometer reading a drive has to land to count as
// arrived. under one sub-cell, which keeps the map and the pose agreeing about
// which cell the robot is in.
const float DRIVE_TOLERANCE_CM = 0.3f;

// --- helpers -----------------------------------------------------------------

const float PI = 3.14159265358979323846f;
const float DEG_TO_RAD = PI / 180.0f;
const float FULL_TURN_DEG = 360.0f;

const float SENSOR_TRUST_CM = static_cast<float>(config::SENSOR_TRUST_CM);

float normalizeDeg(float deg) {
  deg = fmodf(deg, FULL_TURN_DEG);
  if (deg < 0.0f) deg += FULL_TURN_DEG;
  return deg;
}

// shortest signed way round from one heading to another. averaging two headings
// either side of north would otherwise swing the pose halfway round the board.
float shortestDeltaDeg(float from, float to) {
  return fmodf(to - from + 540.0f, FULL_TURN_DEG) - 180.0f;
}

}  // namespace

VexRobot::VexRobot(const Vec2& startPosition, float startHeadingDeg,
                   const Goal& goal)
    : leftMotor_(LEFT_MOTOR_PORT, LEFT_MOTOR_REVERSED),
      rightMotor_(RIGHT_MOTOR_PORT, RIGHT_MOTOR_REVERSED),
      imu_(),
      drive_(leftMotor_, rightMotor_, imu_, WHEEL_TRAVEL_CM, TRACK_WIDTH_CM,
             WHEEL_BASE_CM, vex::distanceUnits::cm, EXTERNAL_GEAR_RATIO),
      range_(DISTANCE_SENSOR_PORT),
      position_(startPosition),
      headingDeg_(normalizeDeg(startHeadingDeg)),
      goal_(goal),
      odometerCm_(0.0f),
      tickTravelCm_(0.0f),
      turnTargetDeg_(0.0f),
      turning_(false),
      turnSign_(1.0f),
      driveTargetCm_(0.0f),
      motionStartMs_(0),
      motionIsDrive_(false),
      stalledTicks_(0),
      finished_(false),
      logRow_(LOG_FIRST_ROW) {}

bool VexRobot::devicesReady() {
  bool driveable = true;
  if (!leftMotor_.installed()) {
    log("no left motor");
    driveable = false;
  }
  if (!rightMotor_.installed()) {
    log("no right motor");
    driveable = false;
  }
  if (!imu_.installed()) {
    log("no inertial sensor");
    driveable = false;
  }
  // it can still be driven without a range sensor. it just never learns
  // anything. warning, not a reason to refuse to start.
  if (!range_.installed()) log("warn: no distance sensor");
  return driveable;
}

void VexRobot::begin() {
  drive_.setDriveVelocity(DRIVE_VELOCITY_PCT, vex::percentUnits::pct);
  drive_.setTurnVelocity(TURN_VELOCITY_PCT, vex::percentUnits::pct);
  drive_.setStopping(vex::brakeType::brake);
  drive_.setTimeout(MOTION_TIMEOUT_SEC, vex::timeUnits::sec);

  if (!devicesReady()) {
    // run() checks finished() before it moves anything. it comes straight back
    // out and the reason stays on screen.
    finished_ = true;
    return;
  }

  // calibration is what makes heading() trustworthy for the rest of the run,
  // and it is only valid if the robot is still while it happens.
  log("calibrating");
  imu_.calibrate();
  vex::this_thread::sleep_for(IMU_SETTLE_MS);
  const uint32_t calibrateStartMs = vex::timer::system();
  while (imu_.isCalibrating()) {
    if (vex::timer::system() - calibrateStartMs > IMU_CALIBRATE_TIMEOUT_MS) {
      // better to run on a questionable heading than to hang here with no
      // indication of why.
      log("warn: imu calibration timed out");
      break;
    }
    vex::this_thread::sleep_for(20);
  }
  // anchor the sensor to the start heading. heading() is then a reading, not a
  // running total, and cannot drift away from the pose.
  imu_.setHeading(headingDeg_, vex::rotationUnits::deg);

  leftMotor_.resetPosition();
  rightMotor_.resetPosition();
  odometerCm_ = 0.0f;
  tickTravelCm_ = 0.0f;
  stalledTicks_ = 0;
  turning_ = false;
  finished_ = false;
  log("ready");
}

// --- sensing -----------------------------------------------------------------

Vec2 VexRobot::position() { return position_; }

float VexRobot::heading() { return headingDeg_; }

float VexRobot::distance() {
  // read live rather than caching: the algorithm samples the range between
  // polling ticks as well as on them, and a stale reading would clear sub-cells
  // the robot has already turned away from.
  if (!range_.isObjectDetected()) return INFINITY;
  const float cm =
      static_cast<float>(range_.objectDistance(vex::distanceUnits::mm)) * 0.1f;
  // anything past the trust horizon is reported as nothing there, which is what
  // markFree() already does with a miss.
  if (!(cm > 0.0f) || cm >= SENSOR_TRUST_CM) return INFINITY;
  return cm;
}

Goal VexRobot::goal() { return goal_; }

// --- pose --------------------------------------------------------------------

float VexRobot::odometerCm() {
  const double revs = 0.5 * (leftMotor_.position(vex::rotationUnits::rev) +
                             rightMotor_.position(vex::rotationUnits::rev));
  return static_cast<float>(revs * WHEEL_TRAVEL_CM / EXTERNAL_GEAR_RATIO);
}

void VexRobot::updatePose() {
  const float odometer = odometerCm();
  tickTravelCm_ = odometer - odometerCm_;
  odometerCm_ = odometer;

  const float previousDeg = headingDeg_;
  headingDeg_ =
      normalizeDeg(static_cast<float>(imu_.heading(vex::rotationUnits::deg)));

  // travel over a tick is attributed to the heading halfway through it, which
  // costs nothing and keeps the pose honest if it is ever driving and turning
  // at the same time.
  const float midDeg =
      previousDeg + 0.5f * shortestDeltaDeg(previousDeg, headingDeg_);
  const float radians = midDeg * DEG_TO_RAD;
  position_.x += tickTravelCm_ * sinf(radians);
  position_.y += tickTravelCm_ * cosf(radians);
}

// --- motion ------------------------------------------------------------------

void VexRobot::startDrive(float cm) {
  if (finished_) return;
  updatePose();
  turning_ = false;
  motionIsDrive_ = true;
  stalledTicks_ = 0;
  motionStartMs_ = vex::timer::system();
  driveTargetCm_ = odometerCm_ + cm;
  drive_.driveFor(cm, vex::distanceUnits::cm, false);
}

void VexRobot::startTurnTo(float headingDeg) {
  if (finished_) return;
  updatePose();
  motionIsDrive_ = false;
  stalledTicks_ = 0;
  motionStartMs_ = vex::timer::system();
  // absolute, not relative. each turn corrects whatever error the last one left
  // behind instead of carrying it forward.
  turnTargetDeg_ = normalizeDeg(headingDeg);
  turnSign_ =
      shortestDeltaDeg(headingDeg_, turnTargetDeg_) >= 0.0f ? 1.0f : -1.0f;
  turning_ = true;
  // kick the motors now, not on the first poll. a caller that checks isMoving()
  // before stepping needs to see a turn that is genuinely underway.
  serviceTurn();
}

void VexRobot::serviceTurn() {
  if (!turning_) return;

  // how much of the arc is left in the direction the turn set out in. once this
  // goes negative the target has been passed, which counts as done rather than
  // a reason to come back for it: the next turn is commanded against an
  // absolute heading and absorbs the overshoot.
  const float remainingDeg =
      turnSign_ * shortestDeltaDeg(headingDeg_, turnTargetDeg_);
  if (remainingDeg <= TURN_TOLERANCE_DEG) {
    drive_.stop();
    turning_ = false;
    return;
  }
  if (vex::timer::system() - motionStartMs_ > TURN_TIMEOUT_MS) {
    endRun("collision: turn stalled");
    return;
  }

  const double velocity =
      remainingDeg > TURN_APPROACH_DEG ? TURN_VELOCITY_PCT : TURN_CRAWL_PCT;
  drive_.turn(turnSign_ > 0.0f ? vex::turnType::right : vex::turnType::left,
              velocity, vex::velocityUnits::pct);
}

// whether the robot is still working on the last motion it was given.
//
// the drivetrain's own isMoving() is not the authority here. once a turn has
// been issued as a velocity command it reports itself as moving for the rest of
// the run, even after stop(), so trusting it left every sweep leg polling until
// it timed out. it is only consulted where it is trustworthy: a drive reporting
// itself done really is done, it is the never-done answer that has to be
// ignored.
bool VexRobot::isMoving() {
  if (finished_) return false;

  const uint32_t elapsedMs = vex::timer::system() - motionStartMs_;

  // the algorithm polls this in loops it owns. a motion that never reports
  // itself done would hang the run outright.
  if (elapsedMs > MOTION_CAP_MS) {
    if (turning_ || motionIsDrive_) {
      drive_.stop();
      turning_ = false;
      log("warn: motion cap hit");
    }
    return false;
  }

  // serviceTurn() owns a turn start to finish. its flag is the whole answer.
  if (!motionIsDrive_) return turning_;

  if (elapsedMs < MOTION_GRACE_MS) return true;
  return fabsf(driveTargetCm_ - odometerCm_) > DRIVE_TOLERANCE_CM &&
         drive_.isMoving();
}

void VexRobot::step() {
  vex::this_thread::sleep_for(POLL_MS);
  updatePose();
  serviceTurn();
  checkForCollision();
  checkForGoal();
}

// --- run state ---------------------------------------------------------------

bool VexRobot::finished() { return finished_; }

void VexRobot::endRun(const char* reason) {
  drive_.stop();
  turning_ = false;
  finished_ = true;
  log(reason);
}

void VexRobot::checkForCollision() {
  if (finished_ || !motionIsDrive_) return;
  if (vex::timer::system() - motionStartMs_ < MOTION_GRACE_MS) return;
  // only meaningful while there is still distance left to cover: a drive that
  // has arrived is supposed to stop producing travel.
  if (fabsf(driveTargetCm_ - odometerCm_) <= DRIVE_TOLERANCE_CM) return;

  if (fabsf(tickTravelCm_) < STALL_TRAVEL_CM) {
    stalledTicks_++;
  } else {
    stalledTicks_ = 0;
  }
  if (stalledTicks_ >= STALL_TICKS) {
    endRun("collision: drive stalled");
  }
}

void VexRobot::checkForGoal() {
  if (finished_) return;
  if (hypotf(goal_.x - position_.x, goal_.y - position_.y) <= goal_.radius) {
    endRun("goal reached");
  }
}

void VexRobot::log(const char* message) {
  if (LOG_TO_SCREEN) {
    Brain.Screen.clearLine(logRow_);
    Brain.Screen.setCursor(logRow_, 1);
    Brain.Screen.print("%s", message);
    // wrap within the log rows so the block does not walk off the screen.
    if (++logRow_ > LOG_LAST_ROW) logRow_ = LOG_FIRST_ROW;
  }
  printf("%s\n", message);
}
