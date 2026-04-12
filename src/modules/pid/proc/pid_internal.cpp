#include "../include/params.h"
#include "../include/private.h"

#include <math.h>

int pid_loop_delay_msec = 20;

double pid_turn_kp = 1.0;
double pid_turn_ki = 1.0;
double pid_turn_kd = 1.0;

double pid_drive_kp = 1.0;
double pid_drive_ki = 1.0;
double pid_drive_kd = 1.0;

double pid_turn_tolerance_deg = 1.0;
double pid_drive_tolerance_in = 0.25;

int pid_turn_settle_loops = 5;
int pid_drive_settle_loops = 5;

double pid_turn_integral_limit = 100.0;
double pid_drive_integral_limit = 100.0;

double pid_turn_min_output_pct = -100.0;
double pid_turn_max_output_pct = 100.0;
double pid_drive_min_output_pct = -100.0;
double pid_drive_max_output_pct = 100.0;

double pid_drive_inches_per_motor_degree = 1.0;
double pid_turn_degrees_per_motor_degree_difference = 1.0;

double pid_left_motor_degrees = 0.0;
double pid_right_motor_degrees = 0.0;

double pid_left_motor_start_degrees = 0.0;
double pid_right_motor_start_degrees = 0.0;

double pid_average_drive_motor_degrees = 0.0;
double pid_turn_motor_degree_difference = 0.0;

double pid_drive_expected_inches = 0.0;
double pid_turn_expected_degrees = 0.0;

double pid_drive_real_inches = 0.0;
double pid_turn_real_degrees = 0.0;

double pid_drive_error_inches = 0.0;
double pid_turn_error_degrees = 0.0;

double pid_drive_previous_error_inches = 0.0;
double pid_drive_integral = 0.0;
double pid_drive_derivative = 0.0;
double pid_drive_output_pct = 0.0;
int pid_drive_settle_count = 0;

double pid_turn_previous_error_degrees = 0.0;
double pid_turn_integral = 0.0;
double pid_turn_derivative = 0.0;
double pid_turn_output_pct = 0.0;
int pid_turn_settle_count = 0;

double pid_left_command_pct = 0.0;
double pid_right_command_pct = 0.0;

static double pid_clamp(double value, double minimum, double maximum) {
  double swap = 0.0;

  if (minimum > maximum) {
    swap = minimum;
    minimum = maximum;
    maximum = swap;
  }

  if (value < minimum) {
    return minimum;
  }

  if (value > maximum) {
    return maximum;
  }

  return value;
}

static int pid_required_settle_loops(int settle_loops) {
  if (settle_loops < 1) {
    return 1;
  }

  return settle_loops;
}

static int pid_drive_is_settled(void) {
  return pid_drive_settle_count >=
         pid_required_settle_loops(pid_drive_settle_loops);
}

static int pid_turn_is_settled(void) {
  return pid_turn_settle_count >=
         pid_required_settle_loops(pid_turn_settle_loops);
}

static void pid_zero_outputs(void) {
  pid_drive_output_pct = 0.0;
  pid_turn_output_pct = 0.0;
  pid_left_command_pct = 0.0;
  pid_right_command_pct = 0.0;
}

static void pid_reset_move_state(void) {
  pid_average_drive_motor_degrees = 0.0;
  pid_turn_motor_degree_difference = 0.0;

  pid_drive_real_inches = 0.0;
  pid_turn_real_degrees = 0.0;

  pid_drive_error_inches = 0.0;
  pid_turn_error_degrees = 0.0;

  pid_drive_previous_error_inches = 0.0;
  pid_drive_integral = 0.0;
  pid_drive_derivative = 0.0;
  pid_drive_settle_count = 0;

  pid_turn_previous_error_degrees = 0.0;
  pid_turn_integral = 0.0;
  pid_turn_derivative = 0.0;
  pid_turn_settle_count = 0;

  pid_zero_outputs();
}

static void pid_load_input_payload(const PidInputPayload* input) {
  pid_left_motor_degrees = input->left_motor_degrees;
  pid_right_motor_degrees = input->right_motor_degrees;
  pid_drive_expected_inches = input->drive_expected_inches;
  pid_turn_expected_degrees = input->turn_expected_degrees;
}

static void pid_capture_move_start(void) {
  pid_left_motor_start_degrees = pid_left_motor_degrees;
  pid_right_motor_start_degrees = pid_right_motor_degrees;
}

static void pid_update_real_motion(void) {
  const double left_delta_degrees =
      pid_left_motor_degrees - pid_left_motor_start_degrees;
  const double right_delta_degrees =
      pid_right_motor_degrees - pid_right_motor_start_degrees;

  pid_average_drive_motor_degrees =
      (left_delta_degrees + right_delta_degrees) / 2.0;
  pid_turn_motor_degree_difference =
      left_delta_degrees - right_delta_degrees;

  pid_drive_real_inches =
      pid_average_drive_motor_degrees * pid_drive_inches_per_motor_degree;
  pid_turn_real_degrees =
      pid_turn_motor_degree_difference *
      pid_turn_degrees_per_motor_degree_difference;
}

static void pid_update_drive_logic(void) {
  const double integral_limit = fabs(pid_drive_integral_limit);

  pid_drive_error_inches = pid_drive_expected_inches - pid_drive_real_inches;
  pid_drive_integral += pid_drive_error_inches;
  pid_drive_integral =
      pid_clamp(pid_drive_integral, -integral_limit, integral_limit);
  pid_drive_derivative =
      pid_drive_error_inches - pid_drive_previous_error_inches;

  pid_drive_output_pct =
      (pid_drive_kp * pid_drive_error_inches) +
      (pid_drive_ki * pid_drive_integral) +
      (pid_drive_kd * pid_drive_derivative);
  pid_drive_output_pct =
      pid_clamp(pid_drive_output_pct,
                pid_drive_min_output_pct,
                pid_drive_max_output_pct);

  if (fabs(pid_drive_error_inches) <= fabs(pid_drive_tolerance_in)) {
    pid_drive_settle_count++;
  } else {
    pid_drive_settle_count = 0;
  }

  if (pid_drive_is_settled()) {
    pid_drive_output_pct = 0.0;
  }

  pid_drive_previous_error_inches = pid_drive_error_inches;
}

static void pid_update_turn_logic(void) {
  const double integral_limit = fabs(pid_turn_integral_limit);

  pid_turn_error_degrees = pid_turn_expected_degrees - pid_turn_real_degrees;
  pid_turn_integral += pid_turn_error_degrees;
  pid_turn_integral =
      pid_clamp(pid_turn_integral, -integral_limit, integral_limit);
  pid_turn_derivative =
      pid_turn_error_degrees - pid_turn_previous_error_degrees;

  pid_turn_output_pct =
      (pid_turn_kp * pid_turn_error_degrees) +
      (pid_turn_ki * pid_turn_integral) +
      (pid_turn_kd * pid_turn_derivative);
  pid_turn_output_pct =
      pid_clamp(pid_turn_output_pct,
                pid_turn_min_output_pct,
                pid_turn_max_output_pct);

  if (fabs(pid_turn_error_degrees) <= fabs(pid_turn_tolerance_deg)) {
    pid_turn_settle_count++;
  } else {
    pid_turn_settle_count = 0;
  }

  if (pid_turn_is_settled()) {
    pid_turn_output_pct = 0.0;
  }

  pid_turn_previous_error_degrees = pid_turn_error_degrees;
}

static void pid_mix_outputs(void) {
  pid_left_command_pct =
      pid_clamp(pid_drive_output_pct + pid_turn_output_pct, -100.0, 100.0);
  pid_right_command_pct =
      pid_clamp(pid_drive_output_pct - pid_turn_output_pct, -100.0, 100.0);
}

static void pid_write_output_payload(PidOutputPayload* output) {
  output->left_command_pct = pid_left_command_pct;
  output->right_command_pct = pid_right_command_pct;
  output->drive_output_pct = pid_drive_output_pct;
  output->turn_output_pct = pid_turn_output_pct;
  output->drive_error_inches = pid_drive_error_inches;
  output->turn_error_degrees = pid_turn_error_degrees;
  output->drive_real_inches = pid_drive_real_inches;
  output->turn_real_degrees = pid_turn_real_degrees;
  output->drive_is_settled = pid_drive_is_settled();
  output->turn_is_settled = pid_turn_is_settled();
}

static void pid_clear_output_payload(PidOutputPayload* output) {
  if (output == 0) {
    return;
  }

  output->left_command_pct = 0.0;
  output->right_command_pct = 0.0;
  output->drive_output_pct = 0.0;
  output->turn_output_pct = 0.0;
  output->drive_error_inches = 0.0;
  output->turn_error_degrees = 0.0;
  output->drive_real_inches = 0.0;
  output->turn_real_degrees = 0.0;
  output->drive_is_settled = 0;
  output->turn_is_settled = 0;
}

// Main PID entry point.
// Engine later fills an input payload with raw sensor data and targets,
// calls this once per PID loop, and reads the command values from output.
void pid_engine_main(const PidInputPayload* input, PidOutputPayload* output) {
  if (input == 0 || output == 0) {
    pid_zero_outputs();
    pid_clear_output_payload(output);
    return;
  }

  pid_load_input_payload(input);

  if (input->start_new_move) {
    pid_reset_move_state();
    pid_capture_move_start();
  }

  pid_update_real_motion();
  pid_update_drive_logic();
  pid_update_turn_logic();
  pid_mix_outputs();
  pid_write_output_payload(output);
}
