#include "vex.h"
#include "./include/control.h"
#include "./include/engine.h"
#include "./include/driver.h"
#include "./include/spec/robot_factory.h"
#include "./include/actions.h"

using namespace vex;

brain Brain;

namespace {
void loadPrototypeRobot() {
  // Drivetrain configuration
  // Motor ports (in order): 1, 10 (left), 2, 9 (right)
  // Right motors auto-reversed for correct rotation
  // Track width: 14 inches = 355.6 mm
  // Wheelbase: 12.5 inches = 317.5 mm
  // Gear ratio: 2:33 = 0.0606
  double wheelbase_mm = 12.5 * 25.4;
  double track_width_mm = 14.0 * 25.4;
  double gear_ratio = 2.0 / 33.0;

  RobotFactory::setDrivetrain(gear_ratio, 1, 10, 2, 9, track_width_mm, wheelbase_mm);

  // Intake motors
  RobotFactory::setIntake(5, ROBOT_GEAR_18_1, false);
  RobotFactory::setUpTake(7, ROBOT_GEAR_18_1, false);
  RobotFactory::setMidTake(8, ROBOT_GEAR_18_1, false);

  // Piston configuration
  // Scoop: not implemented yet
  vex::triport::port arm_port = Brain.ThreeWirePort.A;
  RobotFactory::setArm(arm_port);
}

void loadPrototypeButtons() {
  set_movement_type(AXIS);
}
}  // namespace

void auton() {
  driveAction(100);
  turnAction(90);
  driveAction(100);
  uptakeAction(1, 1000);
  midtakeAction(1, 1000);
  intakeAction(1, 1000);
  turnAction(-90);
  driveAction(100);
  armActivate();
  armDeactivate();
}

int main() {
  pre_auton();
  loadPrototypeRobot();
  loadPrototypeButtons();
  auton();
  logProgramHalt();

  while (true) {
    wait(100, msec);
  }
}
