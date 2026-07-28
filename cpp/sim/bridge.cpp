// An IRobot backed by the TypeScript simulator on the other end of a pipe.
//
// The simulator stays authoritative for physics, so the algorithm being tested
// here is the same translation unit that goes to the robot, exercised against
// the same raycasting and collision the browser build uses. Nothing about the
// board, the sensor model, or the motion integration is reimplemented in C++,
// so a disagreement between this and the simulator is a disagreement about the
// algorithm rather than about geometry.
//
// One line out, one line back. Every reply but GOAL carries the full robot
// state, which is why the pose accessors below can answer from a cache: the
// pose can only change on a command that returns a fresh one.

#include <stdio.h>
#include <string.h>

#include "program.h"
#include "robot.h"

namespace {

class BridgeRobot : public IRobot {
 public:
  BridgeRobot() : heading_(0.0f), distance_(0.0f), moving_(false), finished_(false) {
    position_.x = 0.0f;
    position_.y = 0.0f;
    goal_.x = 0.0f;
    goal_.y = 0.0f;
    goal_.radius = 0.0f;

    exchange("SENSE");

    char line[128];
    writeLine("GOAL");
    if (readLine(line, sizeof(line))) {
      sscanf(line, "%f %f %f", &goal_.x, &goal_.y, &goal_.radius);
    }
  }

  Vec2 position() override { return position_; }
  float heading() override { return heading_; }
  float distance() override { return distance_; }
  Goal goal() override { return goal_; }

  void turn(float deltaDeg) override { command("TURN", deltaDeg); }
  void startDrive(float cm) override { command("STARTDRIVE", cm); }
  void startTurnTo(float headingDeg) override { command("STARTTURNTO", headingDeg); }

  bool isMoving() override { return moving_; }
  void step() override { exchange("STEP"); }
  bool finished() override { return finished_; }

  void log(const char* message) override {
    char line[256];
    snprintf(line, sizeof(line), "LOG %s", message);
    exchange(line);
  }

  void disconnect() { writeLine("DONE"); }

 private:
  void command(const char* name, float value) {
    char line[64];
    snprintf(line, sizeof(line), "%s %.6f", name, value);
    exchange(line);
  }

  void exchange(const char* request) {
    writeLine(request);
    char line[128];
    if (!readLine(line, sizeof(line))) {
      // The simulator hung up. Reporting the run as over unwinds every motion
      // loop in the algorithm rather than leaving it spinning on a dead pipe.
      finished_ = true;
      return;
    }
    int moving = 0;
    int finished = 0;
    sscanf(line, "%f %f %f %f %d %d", &position_.x, &position_.y, &heading_, &distance_,
           &moving, &finished);
    moving_ = moving != 0;
    finished_ = finished != 0;
  }

  static void writeLine(const char* line) {
    fputs(line, stdout);
    fputc('\n', stdout);
    fflush(stdout);
  }

  static bool readLine(char* buffer, size_t size) {
    return fgets(buffer, static_cast<int>(size), stdin) != NULL;
  }

  Vec2 position_;
  float heading_;
  float distance_;
  Goal goal_;
  bool moving_;
  bool finished_;
};

}  // namespace

int main() {
  // Static so the maps inside program.cpp are not the only large thing kept off
  // a stack that, on the robot, is measured in kilobytes.
  static BridgeRobot robot;
  run(robot);
  robot.disconnect();
  return 0;
}
