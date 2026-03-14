#include "../../include/control/private.h"
#include "../../include/spec/private.h"

namespace {
const int kDriveDeadbandPct = 5;
const double kIntakeDriverVelocityPct = 100.0;
const int kButtonDriveVelocityPct = 80;
const int kButtonTurnVelocityPct = 60;
const vex::directionType kIntakeInwardDirection = vex::directionType::fwd;
const vex::directionType kIntakeForwardDirection = vex::directionType::rev;

ControllerProgram ActiveControllerProgram;

Robot& activeRobot() {
  return getRobotInternal();
}

bool robotHasController() {
  return activeRobot().hasController && activeRobot().controller != NULL;
}

bool robotHasDrivetrain() {
  return activeRobot().hasDrivetrain && activeRobot().leftDrive != NULL &&
         activeRobot().rightDrive != NULL;
}

bool robotHasIntake() {
  return activeRobot().hasIntake && activeRobot().intake != NULL;
}

int applyDeadband(int valuePct) {
  if (valuePct > -kDriveDeadbandPct && valuePct < kDriveDeadbandPct) {
    return 0;
  }

  return valuePct;
}

int clampVelocity(int valuePct) {
  if (valuePct > 100) {
    return 100;
  }

  if (valuePct < -100) {
    return -100;
  }

  return valuePct;
}

bool isButtonPressed(ControllerButton button) {
  if (!robotHasController()) {
    return false;
  }

  switch (button) {
    case CONTROLLER_BUTTON_L1:
      return activeRobot().controller->ButtonL1.pressing();
    case CONTROLLER_BUTTON_L2:
      return activeRobot().controller->ButtonL2.pressing();
    case CONTROLLER_BUTTON_R1:
      return activeRobot().controller->ButtonR1.pressing();
    case CONTROLLER_BUTTON_R2:
      return activeRobot().controller->ButtonR2.pressing();
    case CONTROLLER_BUTTON_UP:
      return activeRobot().controller->ButtonUp.pressing();
    case CONTROLLER_BUTTON_DOWN:
      return activeRobot().controller->ButtonDown.pressing();
    case CONTROLLER_BUTTON_LEFT:
      return activeRobot().controller->ButtonLeft.pressing();
    case CONTROLLER_BUTTON_RIGHT:
      return activeRobot().controller->ButtonRight.pressing();
    case CONTROLLER_BUTTON_X:
      return activeRobot().controller->ButtonX.pressing();
    case CONTROLLER_BUTTON_B:
      return activeRobot().controller->ButtonB.pressing();
    case CONTROLLER_BUTTON_Y:
      return activeRobot().controller->ButtonY.pressing();
    case CONTROLLER_BUTTON_A:
      return activeRobot().controller->ButtonA.pressing();
    case CONTROLLER_BUTTON_NONE:
    default:
      return false;
  }
}

void stopDrive() {
  if (!robotHasDrivetrain()) {
    return;
  }

  activeRobot().leftDrive->stop(vex::brakeType::coast);
  activeRobot().rightDrive->stop(vex::brakeType::coast);
}

void spinDriveGroup(vex::motor_group* driveGroup, int velocityPct) {
  if (driveGroup == NULL) {
    return;
  }

  if (velocityPct == 0) {
    driveGroup->stop(vex::brakeType::coast);
    return;
  }

  if (velocityPct < 0) {
    driveGroup->spin(vex::directionType::rev, -velocityPct, vex::velocityUnits::pct);
    return;
  }

  driveGroup->spin(vex::directionType::fwd, velocityPct, vex::velocityUnits::pct);
}

void stopIntake() {
  if (!robotHasIntake()) {
    return;
  }

  activeRobot().intake->stop(vex::brakeType::coast);
}

void runAxisDriveMode() {
  const int forwardVelocityPct =
      applyDeadband(activeRobot().controller->Axis2.position(vex::percentUnits::pct));
  const int turnVelocityPct =
      applyDeadband(activeRobot().controller->Axis1.position(vex::percentUnits::pct));
  const int leftVelocityPct = clampVelocity(forwardVelocityPct + turnVelocityPct);
  const int rightVelocityPct = clampVelocity(forwardVelocityPct - turnVelocityPct);

  spinDriveGroup(activeRobot().leftDrive, leftVelocityPct);
  spinDriveGroup(activeRobot().rightDrive, rightVelocityPct);
}

void runButtonDriveMode() {
  int forwardVelocityPct = 0;
  int turnVelocityPct = 0;
  int leftVelocityPct = 0;
  int rightVelocityPct = 0;

  if (activeRobot().controller->ButtonUp.pressing()) {
    forwardVelocityPct += kButtonDriveVelocityPct;
  }

  if (activeRobot().controller->ButtonDown.pressing()) {
    forwardVelocityPct -= kButtonDriveVelocityPct;
  }

  if (activeRobot().controller->ButtonLeft.pressing()) {
    turnVelocityPct -= kButtonTurnVelocityPct;
  }

  if (activeRobot().controller->ButtonRight.pressing()) {
    turnVelocityPct += kButtonTurnVelocityPct;
  }

  leftVelocityPct = clampVelocity(forwardVelocityPct + turnVelocityPct);
  rightVelocityPct = clampVelocity(forwardVelocityPct - turnVelocityPct);

  spinDriveGroup(activeRobot().leftDrive, leftVelocityPct);
  spinDriveGroup(activeRobot().rightDrive, rightVelocityPct);
}
}  // namespace

ControllerProgram::ControllerProgram() {
  reset();
}

void ControllerProgram::reset() {
  intakeInButton_ = CONTROLLER_BUTTON_NONE;
  intakeForwardButton_ = CONTROLLER_BUTTON_NONE;
  movementType_ = AXIS;
}

void ControllerProgram::configureButtons(ControllerButton intakeInButton,
                                         ControllerButton intakeForwardButton) {
  intakeInButton_ = intakeInButton;
  intakeForwardButton_ = intakeForwardButton;
}

void ControllerProgram::configureMovementType(MovementType movementType) {
  movementType_ = movementType;
}

void ControllerProgram::execute() {
  while (true) {
    if (!robotHasController()) {
      stopDrive();
      stopIntake();
      wait(20, vex::msec);
      continue;
    }

    if (robotHasDrivetrain()) {
      if (movementType_ == BUTTONS) {
        runButtonDriveMode();
      } else {
        runAxisDriveMode();
      }
    }

    if (robotHasIntake()) {
      if (isButtonPressed(intakeInButton_)) {
        activeRobot().intake->spin(kIntakeInwardDirection,
                                   kIntakeDriverVelocityPct,
                                   vex::velocityUnits::pct);
      } else if (isButtonPressed(intakeForwardButton_)) {
        activeRobot().intake->spin(kIntakeForwardDirection,
                                   kIntakeDriverVelocityPct,
                                   vex::velocityUnits::pct);
      } else {
        stopIntake();
      }
    }

    wait(20, vex::msec);
  }
}

ControllerProgram& getControllerProgramInternal(void) {
  return ActiveControllerProgram;
}

void resetControllerProgramInternal(void) {
  ActiveControllerProgram.reset();
}

void setControllerButtons(ControllerButton intakeInButton,
                          ControllerButton intakeForwardButton) {
  ActiveControllerProgram.configureButtons(intakeInButton, intakeForwardButton);
}

void set_movement_type(MovementType movementType) {
  ActiveControllerProgram.configureMovementType(movementType);
}
