#include "../../include/spec/private.h"
#include "../../include/control/constants.h"
#include "../pid/include/private.h"
#include "../pid/include/params.h"
#include "../../include/actions.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <vex.h>

using namespace vex;

competition Competition;

const double kPi = 3.14159265358979323846;
const int kMaxBrainLogLines = 12;
const int kBrainLogHistoryLines = 64;
const int kLogBufferSize = 96;
const int kMaxPidLoopCount = 400;

// Internal robot state used for debug math and autonomous planning.
typedef struct RobotPose {
  double xInches;
  double yInches;
  double headingDegrees;
} RobotPose;

typedef struct MotionProfile {
  double driveVelocityPct;
  double turnVelocityPct;
  vex::brakeType stopMode;
} MotionProfile;


int RouteCount = 0;

bool VerboseLoggingEnabled = false;
bool VerboseLoggingConfigured = false;
char BrainLogHistory[kBrainLogHistoryLines][kLogBufferSize];
int BrainLogHistoryStart = 0;
int BrainLogHistoryCount = 0;
bool HardwareConfigured = false;

// Default verbose state used until main overrides it with setBrainVerbose(...).
const bool kVerboseLoggingDefault = true;

// Default prototype pose. If the robot needs a different starting location,
// tune it here without expanding the public engine API.
RobotPose ConfiguredStartPose = {0.0, 0.0, 0.0};
RobotPose EstimatedPose = {0.0, 0.0, 0.0};

void logToBrain(const char* format, ...); 

Robot& activeRobot() {
  return getRobotInternal();
}

vex::brain& robotBrain() {
  return activeRobot().brain;
}

bool robotHasDrivetrain() {
  return activeRobot().hasDrivetrain && activeRobot().drivetrain != NULL &&
         activeRobot().leftDrive != NULL && activeRobot().rightDrive != NULL;
}

vex::drivetrain* drivetrainFromRobot() {
  if (!robotHasDrivetrain()) {
    return NULL;
  }

  return activeRobot().drivetrain;
}

double averageMotorDegrees(vex::motor* motors[], int motorCount) {
  double totalDegrees = 0.0;
  int connectedMotors = 0;
  int index = 0;

  for (index = 0; index < motorCount; index++) {
    if (motors[index] == NULL) {
      continue;
    }

    totalDegrees += motors[index]->position(vex::rotationUnits::deg);
    connectedMotors++;
  }

  if (connectedMotors == 0) {
    return 0.0;
  }

  return totalDegrees / (double)connectedMotors;
}

double leftDriveMotorDegrees() {
  return averageMotorDegrees(activeRobot().leftDriveMotors,
                             activeRobot().leftDriveMotorCount);
}

double rightDriveMotorDegrees() {
  return averageMotorDegrees(activeRobot().rightDriveMotors,
                             activeRobot().rightDriveMotorCount);
}

double clampDriveCommand(double value, double maxMagnitude) {
  const double limitedMagnitude = fabs(maxMagnitude);

  if (limitedMagnitude <= 0.0) {
    return 0.0;
  }

  if (value > limitedMagnitude) {
    return limitedMagnitude;
  }

  if (value < -limitedMagnitude) {
    return -limitedMagnitude;
  }

  return value;
}

void spinDriveGroupAtPct(vex::motor_group* driveGroup, double velocityPct) {
  if (driveGroup == NULL) {
    return;
  }

  if (fabs(velocityPct) < 0.01) {
    driveGroup->stop(vex::brakeType::brake);
    return;
  }

  if (velocityPct < 0.0) {
    driveGroup->spin(vex::directionType::rev,
                     fabs(velocityPct),
                     vex::velocityUnits::pct);
  } else {
    driveGroup->spin(vex::directionType::fwd,
                     velocityPct,
                     vex::velocityUnits::pct);
  }
}

void applyPidDriveOutput(const PidOutputPayload& output, double maxCommandPct) {
  const double leftCommand = clampDriveCommand(output.left_command_pct, maxCommandPct);
  const double rightCommand = clampDriveCommand(output.right_command_pct, maxCommandPct);

  spinDriveGroupAtPct(activeRobot().leftDrive, leftCommand);
  spinDriveGroupAtPct(activeRobot().rightDrive, rightCommand);
}

void stopDriveBase(vex::brakeType stopMode) {
  if (!robotHasDrivetrain()) {
    return;
  }

  activeRobot().leftDrive->stop(stopMode);
  activeRobot().rightDrive->stop(stopMode);
}

int pidMoveFinished(const PidOutputPayload& output,
                    double expectedDriveInches,
                    double expectedTurnDegrees) {
  int finished = 1;

  if (fabs(expectedDriveInches) >= 0.01 && !output.drive_is_settled) {
    finished = 0;
  }

  if (fabs(expectedTurnDegrees) >= 0.01 && !output.turn_is_settled) {
    finished = 0;
  }

  return finished;
}

PidOutputPayload runPidMove(double expectedDriveInches,
                            double expectedTurnDegrees,
                            double maxCommandPct,
                            vex::brakeType stopMode) {
  PidOutputPayload output;
  int loopCount = 0;

  output = pid_run_from_motor_input(leftDriveMotorDegrees(),
                                    rightDriveMotorDegrees(),
                                    expectedDriveInches,
                                    expectedTurnDegrees,
                                    1);

  while (true) {
    applyPidDriveOutput(output, maxCommandPct);

    if (pidMoveFinished(output, expectedDriveInches, expectedTurnDegrees)) {
      break;
    }

    loopCount++;
    if (loopCount >= kMaxPidLoopCount) {
      logToBrain("PID move timeout");
      break;
    }

    wait(pid_loop_delay_msec, vex::msec);

    output = pid_run_from_motor_input(leftDriveMotorDegrees(),
                                      rightDriveMotorDegrees(),
                                      expectedDriveInches,
                                      expectedTurnDegrees,
                                      0);

    logToBrain("PID drive: p=%.1f i=%.1f d=%.1f out=%.0f%%",
               pid_drive_kp * pid_drive_error_inches,
               pid_drive_ki * pid_drive_integral,
               pid_drive_kd * pid_drive_derivative,
               pid_drive_output_pct);
    logToBrain("Drive: err %.2f in, correct %.0f%%",
               pid_drive_error_inches,
               pid_drive_output_pct);
    logToBrain("Turn: err %.1f deg, correct %.0f%%",
               pid_turn_error_degrees,
               pid_turn_output_pct);
  }

  stopDriveBase(stopMode);
  return output;
}

void resetBrainLog() {
  if (!VerboseLoggingEnabled) {
    return;
  }

  BrainLogHistoryStart = 0;
  BrainLogHistoryCount = 0;
  robotBrain().Screen.clearScreen();
}

void renderBrainLogHistory() {
  int visibleCount = 0;
  int visibleOffset = 0;
  int line = 0;

  if (!VerboseLoggingEnabled) {
    return;
  }

  robotBrain().Screen.clearScreen();

  visibleCount = BrainLogHistoryCount;
  if (visibleCount > kMaxBrainLogLines) {
    visibleCount = kMaxBrainLogLines;
  }

  visibleOffset = BrainLogHistoryCount - visibleCount;

  for (line = 0; line < visibleCount; line++) {
    const int historyIndex =
        (BrainLogHistoryStart + visibleOffset + line) % kBrainLogHistoryLines;

    robotBrain().Screen.setCursor(line + 1, 1);
    robotBrain().Screen.print("%s", BrainLogHistory[historyIndex]);
  }
}

void appendBrainLogLine(const char* text) {
  int historyIndex = 0;

  if (!VerboseLoggingEnabled) {
    return;
  }

  if (BrainLogHistoryCount < kBrainLogHistoryLines) {
    historyIndex =
        (BrainLogHistoryStart + BrainLogHistoryCount) % kBrainLogHistoryLines;
    BrainLogHistoryCount++;
  } else {
    historyIndex = BrainLogHistoryStart;
    BrainLogHistoryStart = (BrainLogHistoryStart + 1) % kBrainLogHistoryLines;
  }

  snprintf(BrainLogHistory[historyIndex], kLogBufferSize, "%s", text);
  renderBrainLogHistory();
}

void logToBrain(const char* format, ...) {
  char buffer[kLogBufferSize];
  va_list args;

  if (!VerboseLoggingEnabled) {
    return;
  }

  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  appendBrainLogLine(buffer);
  wait(50, msec);  // Small delay to make logs readable
}

double executeTurn(double signedTurnDegrees, const MotionProfile& profile) {
  PidOutputPayload output;

  if (!robotHasDrivetrain()) {
    return 0.0;
  }

  if (fabs(signedTurnDegrees) < 0.01) {
    logToBrain("Turn skipped: already aligned");
    return 0.0;
  }

  if (signedTurnDegrees < 0.0) {
    logToBrain("Turn left %.1f deg", fabs(signedTurnDegrees));
  } else {
    logToBrain("Turn right %.1f deg", fabs(signedTurnDegrees));
  }

  output = runPidMove(0.0,
                      signedTurnDegrees,
                      profile.turnVelocityPct,
                      profile.stopMode);
  logToBrain("Turn real %.1f err %.1f",
             output.turn_real_degrees,
             output.turn_error_degrees);

  return output.turn_real_degrees;
}

double executeDrive(double signedDistanceInches, const MotionProfile& profile) {
  PidOutputPayload output;

  if (!robotHasDrivetrain()) {
    return 0.0;
  }

  if (fabs(signedDistanceInches) < 0.01) {
    logToBrain("Drive skipped: zero distance");
    return 0.0;
  }

  if (signedDistanceInches < 0.0) {
    logToBrain("Drive reverse %.1f in", fabs(signedDistanceInches));
  } else {
    logToBrain("Drive forward %.1f in", signedDistanceInches);
  }

  output = runPidMove(signedDistanceInches,
                      0.0,
                      profile.driveVelocityPct,
                      profile.stopMode);
  logToBrain("Drive real %.1f err %.2f",
             output.drive_real_inches,
             output.drive_error_inches);

  return output.drive_real_inches;
}

void setBrainVerbose(bool enabled) {
  VerboseLoggingEnabled = enabled;
  VerboseLoggingConfigured = true;

  // When verbose is turned off, clear the old debug text so the Brain screen
  // does not keep stale route messages from a previous run.
  if (!VerboseLoggingEnabled) {
    BrainLogHistoryStart = 0;
    BrainLogHistoryCount = 0;
    robotBrain().Screen.clearScreen();
  }
}

void logProgramHalt(void) {
  if (!VerboseLoggingConfigured) {
    VerboseLoggingEnabled = kVerboseLoggingDefault;
  }

  logToBrain("Program Halt");
}

// Drive action: positive = forward, negative = reverse
void driveAction(double signedDistanceInches) {
  MotionProfile profile = {50.0, 35.0, vex::brakeType::brake};
  executeDrive(signedDistanceInches, profile);
}

// Turn action: positive = right, negative = left
void turnAction(double signedDegrees) {
  MotionProfile profile = {50.0, 35.0, vex::brakeType::brake};
  executeTurn(signedDegrees, profile);
}

// Intake action: direction > 0 = forward, direction < 0 = reverse, duration in milliseconds
void intakeAction(int direction, int durationMs) {
  Robot& robot = activeRobot();

  if (!robot.hasIntakeMotor || robot.intakeMotor == NULL) {
    return;
  }

  if (direction > 0) {
    robot.intakeMotor->spin(vex::directionType::fwd, TAKE_SPEED_RPM, vex::velocityUnits::rpm);
  } else if (direction < 0) {
    robot.intakeMotor->spin(vex::directionType::rev, TAKE_SPEED_RPM, vex::velocityUnits::rpm);
  } else {
    robot.intakeMotor->stop(vex::brakeType::coast);
  }

  wait(durationMs, msec);
  robot.intakeMotor->stop(vex::brakeType::coast);
}

// UpTake action: direction > 0 = forward, direction < 0 = reverse, duration in milliseconds
void uptakeAction(int direction, int durationMs) {
  Robot& robot = activeRobot();

  if (!robot.hasUpTakeMotor || robot.upTakeMotor == NULL) {
    return;
  }

  if (direction > 0) {
    robot.upTakeMotor->spin(vex::directionType::fwd, TAKE_SPEED_RPM, vex::velocityUnits::rpm);
  } else if (direction < 0) {
    robot.upTakeMotor->spin(vex::directionType::rev, TAKE_SPEED_RPM, vex::velocityUnits::rpm);
  } else {
    robot.upTakeMotor->stop(vex::brakeType::coast);
  }

  wait(durationMs, msec);
  robot.upTakeMotor->stop(vex::brakeType::coast);
}

// MidTake action: direction > 0 = forward, direction < 0 = reverse, duration in milliseconds
void midtakeAction(int direction, int durationMs) {
  Robot& robot = activeRobot();

  if (!robot.hasMidTakeMotor || robot.midTakeMotor == NULL) {
    return;
  }

  if (direction > 0) {
    robot.midTakeMotor->spin(vex::directionType::fwd, TAKE_SPEED_RPM, vex::velocityUnits::rpm);
  } else if (direction < 0) {
    robot.midTakeMotor->spin(vex::directionType::rev, TAKE_SPEED_RPM, vex::velocityUnits::rpm);
  } else {
    robot.midTakeMotor->stop(vex::brakeType::coast);
  }

  wait(durationMs, msec);
  robot.midTakeMotor->stop(vex::brakeType::coast);
}

// Scoop activate
void scoopActivate() {
  Robot& robot = activeRobot();

  if (!robot.hasScoop || robot.scoopPiston == NULL) {
    return;
  }

  robot.scoopPiston->open();
}

// Scoop deactivate
void scoopDeactivate() {
  Robot& robot = activeRobot();

  if (!robot.hasScoop || robot.scoopPiston == NULL) {
    return;
  }

  robot.scoopPiston->close();
}

// Arm activate
void armActivate() {
  Robot& robot = activeRobot();

  if (!robot.hasArm || robot.armPiston == NULL) {
    return;
  }

  robot.armPiston->open();
}

// Arm deactivate
void armDeactivate() {
  Robot& robot = activeRobot();

  if (!robot.hasArm || robot.armPiston == NULL) {
    return;
  }

  robot.armPiston->close();
}
