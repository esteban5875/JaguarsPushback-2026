#pragma once

typedef struct PidInputPayload {
  double left_motor_degrees;
  double right_motor_degrees;
  double drive_expected_inches;
  double turn_expected_degrees;
  int start_new_move;
} PidInputPayload;

typedef struct PidOutputPayload {
  double left_command_pct;
  double right_command_pct;
  double drive_output_pct;
  double turn_output_pct;
  double drive_error_inches;
  double turn_error_degrees;
  double drive_real_inches;
  double turn_real_degrees;
  int drive_is_settled;
  int turn_is_settled;
} PidOutputPayload;

void pid_engine_main(const PidInputPayload* input, PidOutputPayload* output);

PidOutputPayload pid_run_from_motor_input(double left_motor_degrees,
                                          double right_motor_degrees,
                                          double drive_expected_inches,
                                          double turn_expected_degrees,
                                          int start_new_move);
