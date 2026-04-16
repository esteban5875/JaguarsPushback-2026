#pragma once

// Wrapper button codes so main does not need to use raw VEX controller members.
typedef enum ControllerButton {
  CONTROLLER_BUTTON_NONE = 0,
  CONTROLLER_BUTTON_L1 = 1,
  CONTROLLER_BUTTON_L2 = 2,
  CONTROLLER_BUTTON_R1 = 3,
  CONTROLLER_BUTTON_R2 = 4,
  CONTROLLER_BUTTON_UP = 5,
  CONTROLLER_BUTTON_DOWN = 6,
  CONTROLLER_BUTTON_LEFT = 7,
  CONTROLLER_BUTTON_RIGHT = 8,
  CONTROLLER_BUTTON_X = 9,
  CONTROLLER_BUTTON_B = 10,
  CONTROLLER_BUTTON_Y = 11,
  CONTROLLER_BUTTON_A = 12
} ControllerButton;

// Wrapper movement modes for driver control.
// BUTTONS = use the arrow buttons for drive movement and turning.
// AXIS = use the right joystick for forward/backward movement and turning.
typedef enum MovementType {
  BUTTONS = 1,
  AXIS = 2
} MovementType;

// Selects how the drivetrain is controlled during driver control.
void set_movement_type(MovementType movementType);
