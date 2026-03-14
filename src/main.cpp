#include "vex.h"
#include "./include/control.h"
#include "./include/engine.h"
#include "./include/driver.h"
#include "./include/spec/robot_factory.h"

namespace {
void loadPrototypeRobot() {
  RobotMotor leftDrive[2] = {
      {1, ROBOT_GEAR_18_1, false},
      {2, ROBOT_GEAR_18_1, false},
  };
  RobotMotor rightDrive[2] = {
      {9, ROBOT_GEAR_18_1, true},
      {10, ROBOT_GEAR_18_1, true},
  };
  RobotMotor intake[1] = {
      {3, ROBOT_GEAR_18_1, false},
  };

  RobotFactory::setController(ROBOT_CONTROLLER_PRIMARY);
  RobotFactory::setDrivetrain(
      leftDrive, 2, rightDrive, 2, 319.19, 320.0, 130.0, ROBOT_DISTANCE_MM, 1.0);
  RobotFactory::setIntake(intake, 1);
}

void loadPrototypeButtons() {
  set_movement_type(AXIS);
  setControllerButtons(CONTROLLER_BUTTON_L1, CONTROLLER_BUTTON_L2);
}

void loadPrototypeRoute() {
  addwaypoint(24.0f, 0, 4, 0);
  addwaypoint(18.0f, 90, 2, 1);
  addwaypoint(12.0f, 180, 3, 0);
}
}  // namespace

int main() {
  pre_auton();
  loadPrototypeRobot();
  loadPrototypeButtons();
  loadPrototypeRoute();

  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);

  while (true) {
    wait(100, msec);
  }
}
