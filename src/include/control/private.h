#pragma once

#include "../control.h"

class ControllerProgram {
 public:
  ControllerProgram();
  void reset();
  void configureButtons(ControllerButton intakeInButton,
                        ControllerButton intakeForwardButton);
  void configureMovementType(MovementType movementType);
  void execute();

 private:
  ControllerButton intakeInButton_;
  ControllerButton intakeForwardButton_;
  MovementType movementType_;
};

ControllerProgram& getControllerProgramInternal(void);
void resetControllerProgramInternal(void);
