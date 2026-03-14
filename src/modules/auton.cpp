#include "../include/engine/private.h"
#include "../include/driver.h"

void auton(void) {
  // Autonomous stays intentionally tiny: the route was already declared in
  // main, so the competition callback only has to tell the engine to execute.
  engineRunInternal();
}
