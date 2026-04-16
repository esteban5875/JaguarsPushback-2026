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

void clearIntakeMotor() {
  if (ActiveRobot.intakeMotor != NULL) {
    delete ActiveRobot.intakeMotor;
    ActiveRobot.intakeMotor = NULL;
  }

  ActiveRobot.hasIntakeMotor = false;
}

void clearUpTakeMotor() {
  if (ActiveRobot.upTakeMotor != NULL) {
    delete ActiveRobot.upTakeMotor;
    ActiveRobot.upTakeMotor = NULL;
  }

  ActiveRobot.hasUpTakeMotor = false;
}

void clearMidTakeMotor() {
  if (ActiveRobot.midTakeMotor != NULL) {
    delete ActiveRobot.midTakeMotor;
    ActiveRobot.midTakeMotor = NULL;
  }

  ActiveRobot.hasMidTakeMotor = false;
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

void clearScoopPiston() {
  if (ActiveRobot.scoopPiston != NULL) {
    delete ActiveRobot.scoopPiston;
    ActiveRobot.scoopPiston = NULL;
  }

  ActiveRobot.hasScoop = false;
}

void clearArmPiston() {
  if (ActiveRobot.armPiston != NULL) {
    delete ActiveRobot.armPiston;
    ActiveRobot.armPiston = NULL;
  }

  ActiveRobot.hasArm = false;
}
}  // namespace

void RobotFactory::reset() {
  clearIntakeMotor();
  clearUpTakeMotor();
  clearMidTakeMotor();
  clearDrivetrain();
  clearScoopPiston();
  clearArmPiston();

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

void RobotFactory::setIntake(int port, RobotGear gear, bool reversed) {
  clearIntakeMotor();

  ActiveRobot.intakeMotor = new vex::motor(port, toVexGear(gear), reversed);
  ActiveRobot.hasIntakeMotor = true;
}

void RobotFactory::setUpTake(int port, RobotGear gear, bool reversed) {
  clearUpTakeMotor();

  ActiveRobot.upTakeMotor = new vex::motor(port, toVexGear(gear), reversed);
  ActiveRobot.hasUpTakeMotor = true;
}

void RobotFactory::setMidTake(int port, RobotGear gear, bool reversed) {
  clearMidTakeMotor();

  ActiveRobot.midTakeMotor = new vex::motor(port, toVexGear(gear), reversed);
  ActiveRobot.hasMidTakeMotor = true;
}

void RobotFactory::setDrivetrain(double ratio, int port1, int port2, int port3, int port4,
                                 double trackWidth, double wheelBase) {
  clearDrivetrain();

  // Create left drive motors (forward)
  ActiveRobot.leftDriveMotors[0] = new vex::motor(port1, vex::gearSetting::ratio18_1, false);
  ActiveRobot.leftDriveMotors[1] = new vex::motor(port2, vex::gearSetting::ratio18_1, false);
  ActiveRobot.leftDriveMotorCount = 2;

  // Create right drive motors (reversed for opposite direction)
  ActiveRobot.rightDriveMotors[0] = new vex::motor(port3, vex::gearSetting::ratio18_1, true);
  ActiveRobot.rightDriveMotors[1] = new vex::motor(port4, vex::gearSetting::ratio18_1, true);
  ActiveRobot.rightDriveMotorCount = 2;

  // Add motors to groups
  ActiveRobot.leftDrive = new vex::motor_group();
  ActiveRobot.rightDrive = new vex::motor_group();
  (*ActiveRobot.leftDrive)(*ActiveRobot.leftDriveMotors[0]);
  (*ActiveRobot.leftDrive)(*ActiveRobot.leftDriveMotors[1]);
  (*ActiveRobot.rightDrive)(*ActiveRobot.rightDriveMotors[0]);
  (*ActiveRobot.rightDrive)(*ActiveRobot.rightDriveMotors[1]);

  double wheelTravelMm = 3.25 * 25.4;  // 3.25 inch wheel * 25.4 mm/inch
  double gearRatio = (56.0 / 36.0) * ratio;  // external gear ratio * motor ratio

  ActiveRobot.drivetrain = new vex::drivetrain(*ActiveRobot.leftDrive,
                                               *ActiveRobot.rightDrive,
                                               wheelTravelMm,
                                               trackWidth,
                                               wheelBase,
                                               vex::distanceUnits::mm,
                                               gearRatio);
  ActiveRobot.hasDrivetrain = true;
}

void RobotFactory::setScoop(vex::triport::port& port) {
  clearScoopPiston();

  ActiveRobot.scoopPiston = new vex::pneumatics(port);
  ActiveRobot.hasScoop = true;
}

void RobotFactory::setArm(vex::triport::port& port) {
  clearArmPiston();

  ActiveRobot.armPiston = new vex::pneumatics(port);
  ActiveRobot.hasArm = true;
}

Robot& getRobotInternal(void) {
  return ActiveRobot;
}
