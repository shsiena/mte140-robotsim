#include "vexrobot.h"

#include <math.h>

#include "config.h"

namespace {

// --- hardware configuration --------------------------------------------------
// EDIT THIS BLOCK to match the built robot. Everything else in this file is
// written against these values.

const int32_t LEFT_MOTOR_PORT = vex::PORT1;   // TODO: confirm
const int32_t RIGHT_MOTOR_PORT = vex::PORT6;  // TODO: confirm
const int32_t DISTANCE_SENSOR_PORT = vex::PORT4;  // TODO: confirm

// One side is mounted facing the other way, so its encoder and command sense
// both have to be flipped for "forward" to mean the same thing on both.
const bool LEFT_MOTOR_REVERSED = false;
const bool RIGHT_MOTOR_REVERSED = true;

// Distance one wheel rolls in a full turn of the wheel, the distance between
// the two driven wheels, and the front-to-rear wheel spacing.
const double WHEEL_TRAVEL_CM = 20.0;  // TODO: measure (200mm wheel = 20.0)
const double TRACK_WIDTH_CM = 20.0;   // TODO: measure
const double WHEEL_BASE_CM = 5.0;     // TODO: measure

// Motor revolutions per wheel revolution. 1.0 for a direct drive, which is
// also the case where the drivetrain's own gear-ratio convention cannot
// disagree with the odometry below. If the build is geared and driveFor()
// distances come out scaled, invert this value in odometerCm().
const double EXTERNAL_GEAR_RATIO = 1.0;  // TODO: confirm

// Slow enough that the sub-cell map is not outrun between polling ticks: at
// 10ms a tick, a fast IQ drivetrain covers most of a 0.67cm sub-cell per tick.
const double DRIVE_VELOCITY_PCT = 40.0;
const double TURN_VELOCITY_PCT = 25.0;

// Ceiling on any one leg, so a jammed wheel cannot hang the run indefinitely.
// The longest leg the planner can produce is the board diagonal.
const int32_t MOTION_TIMEOUT_SEC = 10;

// Turns are closed against the inertial sensor here instead of being handed to
// smartdrive::turnToHeading, which scales motor velocity by the heading error.
// The algorithm sweeps in 1 degree steps, and at 1 degree of error that comes
// out below what it takes to break static friction: the motors are commanded,
// nothing rotates, and the turn sits there until it times out. A fixed
// velocity with a floor under it turns the same 1 degree in a couple of ticks.
const double TURN_CRAWL_PCT = 12.0;
// Error below which the crawl velocity is used instead of the full one.
const double TURN_APPROACH_DEG = 10.0;
// Close enough to stop. Has to stay under the 1 degree sweep step, or a sweep
// step would be considered already arrived and the robot would never rotate.
const float TURN_TOLERANCE_DEG = 0.5f;
const uint32_t TURN_TIMEOUT_MS = 4000;

// The inertial sensor keeps reporting the previous calibration state for a
// moment after one is requested, so the wait has to start after a short delay
// or it falls straight through.
const uint32_t IMU_SETTLE_MS = 100;
const uint32_t IMU_CALIBRATE_TIMEOUT_MS = 5000;

// --- polling -----------------------------------------------------------------

const uint32_t POLL_MS = 10;

// The drivetrain reports itself stopped in the window between a motion being
// commanded and the motors spinning up, which would end a poll loop before it
// began.
const uint32_t MOTION_GRACE_MS = 60;

// A drive that produces less than this much travel per tick, for this many
// consecutive ticks, is pushing against something. Neither value is critical:
// the point is to end the run rather than grind, and to be well clear of what
// the drivetrain produces while it is accelerating.
const float STALL_TRAVEL_CM = 0.02f;
const int STALL_TICKS = 25;

// --- debug -------------------------------------------------------------------

// Most of the screen is the live map view drawn by main.cpp, so everything
// written here goes in the bands screen.h reserves around it. Only the last
// log line survives on screen; the full history goes to the serial console.

const bool DEBUG_TO_SCREEN = true;

// Redrawing the screen costs far more than a polling tick, so the status is
// throttled rather than drawn on every one.
const uint32_t DEBUG_REFRESH_MS = 150;

// Nothing the algorithm asks for legitimately takes this long: the drivetrain
// gives up on a drive after MOTION_TIMEOUT_SEC and a turn after
// TURN_TIMEOUT_MS. Past it, the motion is declared over so that a poll loop
// ends rather than running forever. Note that the algorithm's own motion loops
// are not ours to bound, which is why this lives in isMoving() rather than in
// settle().
const uint32_t MOTION_CAP_MS = 12000;

// Bound on a blocking turn, which is ours to end.
const uint32_t SETTLE_CAP_MS = 6000;

// How near the commanded odometer reading a drive has to land to count as
// arrived. Under one sub-cell, so the map never disagrees with the pose about
// which cell the robot is in.
const float DRIVE_TOLERANCE_CM = 0.3f;

// --- helpers -----------------------------------------------------------------

const float PI = 3.14159265358979323846f;
const float DEG_TO_RAD = PI / 180.0f;
const float FULL_TURN_DEG = 360.0f;

const float SENSOR_MAX_CM = static_cast<float>(config::SENSOR_MAX_CM);

float normalizeDeg(float deg) {
  deg = fmodf(deg, FULL_TURN_DEG);
  if (deg < 0.0f) deg += FULL_TURN_DEG;
  return deg;
}

// Shortest signed way round from one heading to another, so that averaging two
// headings either side of north does not swing the pose halfway round the
// board.
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
      brain_(),
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
      logRow_(LOG_FIRST_ROW),
      where_("init"),
      turnCount_(0),
      stepCount_(0),
      rangeCount_(0),
      lastDebugMs_(0) {}

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
  // The robot can still be driven without a range sensor, it just never learns
  // anything, so this is a warning rather than a reason not to start.
  if (!range_.installed()) log("warn: no distance sensor");
  return driveable;
}

void VexRobot::begin() {
  drive_.setDriveVelocity(DRIVE_VELOCITY_PCT, vex::percentUnits::pct);
  drive_.setTurnVelocity(TURN_VELOCITY_PCT, vex::percentUnits::pct);
  drive_.setStopping(vex::brakeType::brake);
  drive_.setTimeout(MOTION_TIMEOUT_SEC, vex::timeUnits::sec);

  if (!devicesReady()) {
    // run() checks finished() before it moves anything, so it returns straight
    // away and the reason stays on the screen.
    finished_ = true;
    return;
  }

  // Calibration is what makes heading() trustworthy for the rest of the run,
  // and it is only valid if the robot is still while it happens.
  log("calibrating");
  imu_.calibrate();
  vex::this_thread::sleep_for(IMU_SETTLE_MS);
  const uint32_t calibrateStartMs = vex::timer::system();
  while (imu_.isCalibrating()) {
    if (vex::timer::system() - calibrateStartMs > IMU_CALIBRATE_TIMEOUT_MS) {
      // Better to run on a questionable heading than to hang here with no
      // indication of why.
      log("warn: imu calibration timed out");
      break;
    }
    vex::this_thread::sleep_for(20);
  }
  // Anchoring the sensor to the start heading makes heading() a reading rather
  // than a running total, so it cannot drift away from the pose.
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

Vec2 VexRobot::position() {
  beat("pos");
  return position_;
}

float VexRobot::heading() {
  beat("hdg");
  return headingDeg_;
}

float VexRobot::distance() {
  rangeCount_++;
  beat("rng");
  // Read live rather than caching: the algorithm samples the range between
  // polling ticks as well as on them, and a stale reading would clear
  // sub-cells the robot has already turned away from.
  if (!range_.isObjectDetected()) return INFINITY;
  const float cm =
      static_cast<float>(range_.objectDistance(vex::distanceUnits::mm)) * 0.1f;
  if (!(cm > 0.0f) || cm >= SENSOR_MAX_CM) return INFINITY;
  return cm;
}

Goal VexRobot::goal() { return goal_; }

// --- pose --------------------------------------------------------------------

float VexRobot::odometerCm() {
  const double revs =
      0.5 * (leftMotor_.position(vex::rotationUnits::rev) +
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

  // Travel over a tick is attributed to the heading halfway through it, which
  // costs nothing and keeps the pose honest if the robot is ever driving and
  // turning at once.
  const float midDeg =
      previousDeg + 0.5f * shortestDeltaDeg(previousDeg, headingDeg_);
  const float radians = midDeg * DEG_TO_RAD;
  position_.x += tickTravelCm_ * sinf(radians);
  position_.y += tickTravelCm_ * cosf(radians);
}

// --- motion ------------------------------------------------------------------

void VexRobot::turn(float deltaDeg) {
  if (finished_) return;
  turnCount_++;
  beat("turn");
  updatePose();
  startTurnTo(headingDeg_ + deltaDeg);
  settle();
}

void VexRobot::startDrive(float cm) {
  if (finished_) return;
  beat("drive");
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
  // Absolute rather than relative, so a turn corrects whatever error the last
  // one left behind instead of carrying it forward.
  turnTargetDeg_ = normalizeDeg(headingDeg);
  turnSign_ = shortestDeltaDeg(headingDeg_, turnTargetDeg_) >= 0.0f ? 1.0f : -1.0f;
  turning_ = true;
  // Kick the motors now rather than on the first poll, so that a caller which
  // checks isMoving() before stepping sees a turn that is genuinely underway.
  serviceTurn();
}

void VexRobot::serviceTurn() {
  if (!turning_) return;

  // How much of the arc is left in the direction the turn set out in. Once
  // this goes negative the target has been passed, which is done rather than a
  // reason to come back for it: the next turn is commanded against an absolute
  // heading and absorbs the overshoot.
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

// Whether the robot is still working on the last motion it was given.
//
// The drivetrain's own isMoving() is not the authority here. Once a turn has
// been issued as a velocity command it reports itself as moving for the rest
// of the run, even after stop(), so trusting it left every 1 degree sweep step
// polling until it timed out. It is only consulted where it is trustworthy: a
// drive reporting itself done really is done, it is the never-done answer that
// has to be ignored.
bool VexRobot::isMoving() {
  if (finished_) return false;

  const uint32_t elapsedMs = vex::timer::system() - motionStartMs_;

  // The algorithm polls this in loops it owns, so a motion that never reports
  // itself done would hang the run outright.
  if (elapsedMs > MOTION_CAP_MS) {
    if (turning_ || motionIsDrive_) {
      drive_.stop();
      turning_ = false;
      log("warn: motion cap hit");
    }
    return false;
  }

  // A turn is owned start to finish by serviceTurn(), so its flag is the whole
  // answer.
  if (!motionIsDrive_) return turning_;

  if (elapsedMs < MOTION_GRACE_MS) return true;
  return fabsf(driveTargetCm_ - odometerCm_) > DRIVE_TOLERANCE_CM &&
         drive_.isMoving();
}

void VexRobot::step() {
  stepCount_++;
  beat("step");
  vex::this_thread::sleep_for(POLL_MS);
  updatePose();
  serviceTurn();
  checkForCollision();
  checkForGoal();
}

void VexRobot::settle() {
  const uint32_t startMs = vex::timer::system();
  while (isMoving() && !finished_) {
    step();
    if (vex::timer::system() - startMs > SETTLE_CAP_MS) {
      drive_.stop();
      turning_ = false;
      log("warn: settle cap hit");
      return;
    }
  }
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
  // Only meaningful while there is still distance left to cover: a drive that
  // has arrived is expected to stop producing travel.
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
  // Printed opaquely and padded, so the tail of a longer previous message is
  // covered rather than left behind. Clearing the line instead would take the
  // map with it.
  brain_.Screen.setPenColor(vex::color::white);
  brain_.Screen.printAt(screen::MAP_X, screen::LOG_Y, true, "%-20s", message);
  printf("%s\n", message);
}

// --- debug -------------------------------------------------------------------

void VexRobot::beat(const char* where) {
  where_ = where;
  if (!DEBUG_TO_SCREEN) return;
  const uint32_t now = vex::timer::system();
  if (now - lastDebugMs_ < DEBUG_REFRESH_MS) return;
  lastDebugMs_ = now;
  drawStatus();
}

void VexRobot::drawStatus() {
  // printf_float is off for this project, so every value is scaled to an
  // integer rather than printed as a float: headings in tenths of a degree,
  // positions and ranges in millimetres.
  const int headingDeci = static_cast<int>(headingDeg_ * 10.0f);
  const int targetDeci = static_cast<int>(turnTargetDeg_ * 10.0f);
  const int xMm = static_cast<int>(position_.x * 10.0f);
  const int yMm = static_cast<int>(position_.y * 10.0f);

  const bool detected = range_.isObjectDetected();
  const int rangeMm =
      detected ? static_cast<int>(range_.objectDistance(vex::distanceUnits::mm))
               : -1;

  for (int row = 1; row <= STATUS_ROWS; row++) {
    brain_.Screen.clearLine(row);
  }

  // Which entry point ran last, and how many times each has. If the screen
  // freezes, whatever these say is where it stopped.
  brain_.Screen.setCursor(1, 1);
  brain_.Screen.print("%s T%d S%d R%d", where_, static_cast<int>(turnCount_),
                      static_cast<int>(stepCount_),
                      static_cast<int>(rangeCount_));

  // Heading against the turn it is chasing, whether a turn is live, and
  // whether the drivetrain agrees anything is moving. Kept short: the screen
  // truncates at roughly sixteen characters.
  brain_.Screen.setCursor(2, 1);
  brain_.Screen.print("h%d>%d t%d m%d", headingDeci, targetDeci,
                      turning_ ? 1 : 0, drive_.isMoving() ? 1 : 0);

  brain_.Screen.setCursor(3, 1);
  brain_.Screen.print("x%d y%d r%d", xMm, yMm, rangeMm);
}
