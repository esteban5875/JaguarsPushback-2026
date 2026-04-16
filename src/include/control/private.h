#pragma once

#include "../control.h"
#include "vex.h"

class ControllerProgram {
 public:
  ControllerProgram();
  void reset();
  void configureMovementType(MovementType movementType);
  void execute();

 private:
  MovementType movementType_;
};

ControllerProgram& getControllerProgramInternal(void);
void resetControllerProgramInternal(void);
