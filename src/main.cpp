#include "vex.h"
#include "./include/engine.h"
#include "./include/driver.h"
#include "./include/spec/robot_factory.h"

namespace {
void loadPrototypeRobot() {
  RobotFactory::reset();
}

void loadPrototypeRoute() {
  setBrainVerbose(true);
}
}  // namespace

int main() {
  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);

  loadPrototypeRobot();
  pre_auton();
  loadPrototypeRoute();

  while (true) {
    wait(100, msec);
  }
}
