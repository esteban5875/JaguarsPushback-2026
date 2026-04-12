#pragma once

// Simple PID timing.
extern int pid_loop_delay_msec;

// Start each weight at 1.0, then tune on the robot.
extern double pid_turn_kp;
extern double pid_turn_ki;
extern double pid_turn_kd;

extern double pid_drive_kp;
extern double pid_drive_ki;
extern double pid_drive_kd;

// Keep the finish window small so large turns like 180 deg end close.
extern double pid_turn_tolerance_deg;
extern double pid_drive_tolerance_in;

// Require the error to stay inside tolerance for a few loops.
extern int pid_turn_settle_loops;
extern int pid_drive_settle_loops;

// Limit integral growth.
extern double pid_turn_integral_limit;
extern double pid_drive_integral_limit;

// Limit PID output before it is sent to the motors.
extern double pid_turn_min_output_pct;
extern double pid_turn_max_output_pct;
extern double pid_drive_min_output_pct;
extern double pid_drive_max_output_pct;

// Convert smart motor rotation into robot movement.
extern double pid_drive_inches_per_motor_degree;
extern double pid_turn_degrees_per_motor_degree_difference;

// Raw smart motor readings.
extern double pid_left_motor_degrees;
extern double pid_right_motor_degrees;

// Smart motor readings captured at the start of the current move.
extern double pid_left_motor_start_degrees;
extern double pid_right_motor_start_degrees;

// Direct motor-based movement measurements.
extern double pid_average_drive_motor_degrees;
extern double pid_turn_motor_degree_difference;

// What the current auton step wants.
extern double pid_drive_expected_inches;
extern double pid_turn_expected_degrees;

// What the smart motor data says the robot actually did.
extern double pid_drive_real_inches;
extern double pid_turn_real_degrees;

// Direct expected vs real error values.
extern double pid_drive_error_inches;
extern double pid_turn_error_degrees;

// Drive PID state.
extern double pid_drive_previous_error_inches;
extern double pid_drive_integral;
extern double pid_drive_derivative;
extern double pid_drive_output_pct;
extern int pid_drive_settle_count;

// Turn PID state.
extern double pid_turn_previous_error_degrees;
extern double pid_turn_integral;
extern double pid_turn_derivative;
extern double pid_turn_output_pct;
extern int pid_turn_settle_count;

// Final left and right drive commands the PID will send.
extern double pid_left_command_pct;
extern double pid_right_command_pct;
