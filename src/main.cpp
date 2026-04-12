#include "vex.h"
#include "./include/control.h"
#include "./include/engine.h"
#include "./include/driver.h"
#include "./include/spec/robot_factory.h"

namespace {
void loadPrototypeRobot() {}

void loadPrototypeButtons() {
  set_movement_type(AXIS);
  setControllerButtons(CONTROLLER_BUTTON_L1, CONTROLLER_BUTTON_L2);
}

void loadPrototypeRoute() {}
}  // namespace

int main() {
  pre_auton();
  loadPrototypeRobot();
  loadPrototypeButtons();
  loadPrototypeRoute();
  auton();
  logProgramHalt();

  while (true) {
    wait(100, msec);
  }
}
