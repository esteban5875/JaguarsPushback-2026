#include "../../include/spec/robot_factory.h"

namespace {
Robot ActiveRobot;

vex::gearSetting toVexGear(RobotGear gear) {
  switch (gear) {
    case ROBOT_GEAR_36_1:
      return vex::gearSetting::ratio36_1;
    case ROBOT_GEAR_6_1:
      return vex::gearSetting::ratio6_1;
    case ROBOT_GEAR_18_1:
    default:
      return vex::gearSetting::ratio18_1;
  }
}

vex::distanceUnits toVexDistanceUnit(RobotDistanceUnit unit) {
  switch (unit) {
    case ROBOT_DISTANCE_IN:
      return vex::distanceUnits::in;
    case ROBOT_DISTANCE_CM:
      return vex::distanceUnits::cm;
    case ROBOT_DISTANCE_MM:
    default:
      return vex::distanceUnits::mm;
  }
}

vex::controllerType toVexControllerType(RobotControllerType type) {
  switch (type) {
    case ROBOT_CONTROLLER_PARTNER:
      return vex::controllerType::partner;
    case ROBOT_CONTROLLER_PRIMARY:
    default:
      return vex::controllerType::primary;
  }
}

void clearMotorArray(vex::motor* motors[], int& motorCount) {
  int index = 0;

  for (index = 0; index < motorCount; index++) {
    if (motors[index] != NULL) {
      delete motors[index];
      motors[index] = NULL;
    }
  }

  motorCount = 0;
}

void clearIntake() {
  if (ActiveRobot.intake != NULL) {
    delete ActiveRobot.intake;
    ActiveRobot.intake = NULL;
  }

  clearMotorArray(ActiveRobot.intakeMotors, ActiveRobot.intakeMotorCount);
  ActiveRobot.hasIntake = false;
}

void clearDrivetrain() {
  if (ActiveRobot.drivetrain != NULL) {
    delete ActiveRobot.drivetrain;
    ActiveRobot.drivetrain = NULL;
  }

  if (ActiveRobot.leftDrive != NULL) {
    delete ActiveRobot.leftDrive;
    ActiveRobot.leftDrive = NULL;
  }

  if (ActiveRobot.rightDrive != NULL) {
    delete ActiveRobot.rightDrive;
    ActiveRobot.rightDrive = NULL;
  }

  clearMotorArray(ActiveRobot.leftDriveMotors, ActiveRobot.leftDriveMotorCount);
  clearMotorArray(ActiveRobot.rightDriveMotors, ActiveRobot.rightDriveMotorCount);
  ActiveRobot.hasDrivetrain = false;
}
}  // namespace

void RobotFactory::reset() {
  clearIntake();
  clearDrivetrain();

  if (ActiveRobot.controller != NULL) {
    delete ActiveRobot.controller;
    ActiveRobot.controller = NULL;
  }

  ActiveRobot.hasBrain = true;
  ActiveRobot.hasController = false;
}

void RobotFactory::setBrain() {
  ActiveRobot.hasBrain = true;
}

void RobotFactory::setController(RobotControllerType controllerType) {
  if (ActiveRobot.controller != NULL) {
    delete ActiveRobot.controller;
  }

  ActiveRobot.controller = new vex::controller(toVexControllerType(controllerType));
  ActiveRobot.hasController = true;
}

void RobotFactory::setIntake(const RobotMotor motors[], int motorCount) {
  int index = 0;

  clearIntake();

  if (motorCount < 0) {
    motorCount = 0;
  }

  if (motorCount > kRobotFactoryMaxMotors) {
    motorCount = kRobotFactoryMaxMotors;
  }

  ActiveRobot.intake = new vex::motor_group();

  for (index = 0; index < motorCount; index++) {
    ActiveRobot.intakeMotors[index] =
        new vex::motor(motors[index].port, toVexGear(motors[index].gear), motors[index].reversed);
    (*ActiveRobot.intake)(*ActiveRobot.intakeMotors[index]);
    ActiveRobot.intakeMotorCount++;
  }

  ActiveRobot.hasIntake = (ActiveRobot.intakeMotorCount > 0);
}

void RobotFactory::setDrivetrain(const RobotMotor leftMotors[],
                                 int leftMotorCount,
                                 const RobotMotor rightMotors[],
                                 int rightMotorCount,
                                 double wheelTravel,
                                 double trackWidth,
                                 double wheelBase,
                                 RobotDistanceUnit distanceUnit,
                                 double externalGearRatio) {
  int index = 0;

  clearDrivetrain();

  if (leftMotorCount < 0) {
    leftMotorCount = 0;
  }

  if (rightMotorCount < 0) {
    rightMotorCount = 0;
  }

  if (leftMotorCount > kRobotFactoryMaxMotors) {
    leftMotorCount = kRobotFactoryMaxMotors;
  }

  if (rightMotorCount > kRobotFactoryMaxMotors) {
    rightMotorCount = kRobotFactoryMaxMotors;
  }

  ActiveRobot.leftDrive = new vex::motor_group();
  ActiveRobot.rightDrive = new vex::motor_group();

  for (index = 0; index < leftMotorCount; index++) {
    ActiveRobot.leftDriveMotors[index] =
        new vex::motor(leftMotors[index].port,
                       toVexGear(leftMotors[index].gear),
                       leftMotors[index].reversed);
    (*ActiveRobot.leftDrive)(*ActiveRobot.leftDriveMotors[index]);
    ActiveRobot.leftDriveMotorCount++;
  }

  for (index = 0; index < rightMotorCount; index++) {
    ActiveRobot.rightDriveMotors[index] =
        new vex::motor(rightMotors[index].port,
                       toVexGear(rightMotors[index].gear),
                       rightMotors[index].reversed);
    (*ActiveRobot.rightDrive)(*ActiveRobot.rightDriveMotors[index]);
    ActiveRobot.rightDriveMotorCount++;
  }

  if (ActiveRobot.leftDriveMotorCount > 0 && ActiveRobot.rightDriveMotorCount > 0) {
    ActiveRobot.drivetrain = new vex::drivetrain(*ActiveRobot.leftDrive,
                                                 *ActiveRobot.rightDrive,
                                                 wheelTravel,
                                                 trackWidth,
                                                 wheelBase,
                                                 toVexDistanceUnit(distanceUnit),
                                                 externalGearRatio);
    ActiveRobot.hasDrivetrain = true;
  }
}

Robot& getRobotInternal(void) {
  return ActiveRobot;
}
