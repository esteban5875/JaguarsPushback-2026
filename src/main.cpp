#include "vex.h"
#include "./include/engine.h"
#include "./include/driver.h"

int main() {
  competition(auton);
  competition(usercontrol);

    pre_auton();

  while (true) {
    wait(100, msec);
  }
}

