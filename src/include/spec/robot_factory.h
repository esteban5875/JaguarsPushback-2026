#pragma once

#include "./robot.h"

// Wrapper enums so main does not need to use raw VEX enums.
typedef enum RobotGear {
  ROBOT_GEAR_36_1 = 1,
  ROBOT_GEAR_18_1 = 2,
  ROBOT_GEAR_6_1 = 3
} RobotGear;

typedef enum RobotDistanceUnit {
  ROBOT_DISTANCE_MM = 1,
  ROBOT_DISTANCE_IN = 2,
  ROBOT_DISTANCE_CM = 3
} RobotDistanceUnit;

typedef enum RobotControllerType {
  ROBOT_CONTROLLER_PRIMARY = 1,
  ROBOT_CONTROLLER_PARTNER = 2
} RobotControllerType;

// Minimal motor input wrapper used by the setter methods.
typedef struct RobotMotor {
  int port;
  RobotGear gear;
  bool reversed;
} RobotMotor;

// Setter-only factory for building the Robot wrapper.
class RobotFactory {
 public:
  static void reset();
  static void setBrain();
  static void setController(RobotControllerType controllerType = ROBOT_CONTROLLER_PRIMARY);
  static void setIntake(const RobotMotor motors[], int motorCount);
  static void setDrivetrain(const RobotMotor leftMotors[],
                            int leftMotorCount,
                            const RobotMotor rightMotors[],
                            int rightMotorCount,
                            double wheelTravel,
                            double trackWidth,
                            double wheelBase,
                            RobotDistanceUnit distanceUnit,
                            double externalGearRatio = 1.0);
};
