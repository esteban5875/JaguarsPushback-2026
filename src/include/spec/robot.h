#pragma once

#include "vex.h"

const int kRobotFactoryMaxMotors = 8;

// Robot is the wrapper object that stores the real VEX hardware objects.
// For now it exposes the four high-level robot parts requested:
// drivetrain, intake, controller, and brain.
class Robot {
 public:
  Robot() {
    int index = 0;

    controller = NULL;
    intake = NULL;
    drivetrain = NULL;
    leftDrive = NULL;
    rightDrive = NULL;

    intakeMotorCount = 0;
    leftDriveMotorCount = 0;
    rightDriveMotorCount = 0;

    hasBrain = true;
    hasController = false;
    hasIntake = false;
    hasDrivetrain = false;

    for (index = 0; index < kRobotFactoryMaxMotors; index++) {
      intakeMotors[index] = NULL;
      leftDriveMotors[index] = NULL;
      rightDriveMotors[index] = NULL;
    }
  }

  ~Robot() {}

  vex::brain brain;
  vex::controller* controller;
  vex::motor_group* intake;
  vex::drivetrain* drivetrain;
  vex::motor_group* leftDrive;
  vex::motor_group* rightDrive;

  // Backing groups and motors kept alive for the VEX group/drivetrain objects.
  vex::motor* intakeMotors[kRobotFactoryMaxMotors];
  int intakeMotorCount;

  vex::motor* leftDriveMotors[kRobotFactoryMaxMotors];
  int leftDriveMotorCount;

  vex::motor* rightDriveMotors[kRobotFactoryMaxMotors];
  int rightDriveMotorCount;

  bool hasBrain;
  bool hasController;
  bool hasIntake;
  bool hasDrivetrain;
};
