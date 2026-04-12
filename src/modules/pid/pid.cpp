#include "./include/private.h"

PidOutputPayload pid_run_from_motor_input(double left_motor_degrees,
                                          double right_motor_degrees,
                                          double drive_expected_inches,
                                          double turn_expected_degrees,
                                          int start_new_move) {
  PidInputPayload input;
  PidOutputPayload output;

  input.left_motor_degrees = left_motor_degrees;
  input.right_motor_degrees = right_motor_degrees;
  input.drive_expected_inches = drive_expected_inches;
  input.turn_expected_degrees = turn_expected_degrees;
  input.start_new_move = start_new_move;

  pid_engine_main(&input, &output);

  return output;
}
