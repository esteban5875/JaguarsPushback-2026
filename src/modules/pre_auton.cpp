#include "../include/driver.h"
#include "../include/control/private.h"
#include "../include/engine.h"
#include "../include/spec/robot_factory.h"

void pre_auton(void) {
  RobotFactory::reset();
  resetControllerProgramInternal();
  setBrainVerbose(true);
}
