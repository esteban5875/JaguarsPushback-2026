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
    intakeMotor = NULL;
    upTakeMotor = NULL;
    midTakeMotor = NULL;
    drivetrain = NULL;
    leftDrive = NULL;
    rightDrive = NULL;
    scoopPiston = NULL;
    armPiston = NULL;

    leftDriveMotorCount = 0;
    rightDriveMotorCount = 0;

    hasBrain = true;
    hasController = false;
    hasIntakeMotor = false;
    hasUpTakeMotor = false;
    hasMidTakeMotor = false;
    hasDrivetrain = false;
    hasScoop = false;
    hasArm = false;

    for (index = 0; index < kRobotFactoryMaxMotors; index++) {
      leftDriveMotors[index] = NULL;
      rightDriveMotors[index] = NULL;
    }
  }

  ~Robot() {}

  vex::brain brain;
  vex::controller* controller;
  vex::motor* intakeMotor;
  vex::motor* upTakeMotor;
  vex::motor* midTakeMotor;
  vex::drivetrain* drivetrain;
  vex::motor_group* leftDrive;
  vex::motor_group* rightDrive;
  vex::pneumatics* scoopPiston;
  vex::pneumatics* armPiston;

  // Backing groups and motors kept alive for the VEX group/drivetrain objects.
  vex::motor* leftDriveMotors[kRobotFactoryMaxMotors];
  int leftDriveMotorCount;

  vex::motor* rightDriveMotors[kRobotFactoryMaxMotors];
  int rightDriveMotorCount;

  bool hasBrain;
  bool hasController;
  bool hasIntakeMotor;
  bool hasUpTakeMotor;
  bool hasMidTakeMotor;
  bool hasDrivetrain;
  bool hasScoop;
  bool hasArm;
};
