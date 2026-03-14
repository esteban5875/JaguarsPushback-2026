#include "../include/driver.h"
#include "../include/control/private.h"

void usercontrol(void) {
  getControllerProgramInternal().execute();
}
