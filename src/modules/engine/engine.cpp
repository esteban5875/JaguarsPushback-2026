#include "../../include/engine/private.h"
#include "../../include/spec/private.h"
#include "../pid/include/private.h"
#include "../pid/include/params.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

vex::competition Competition;

namespace {
const double kPi = 3.14159265358979323846;
const int kMaxBrainLogLines = 12;
const int kBrainLogHistoryLines = 64;
const int kLogBufferSize = 96;
const int kMaxPidLoopCount = 400;
const double kIntakeVelocityPct = 80.0;

// These are prototype defaults for the two intake behaviors requested.
// If the intake spins the wrong way on a real robot, flip the intake motor
// reversed flag in loadPrototypeRobot() or tune these constants later.
const vex::directionType kIntakeInwardDirection = vex::directionType::fwd;
const vex::directionType kIntakeForwardDirection = vex::directionType::rev;

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

Waypoint* RouteHead = NULL;
Waypoint* RouteTail = NULL;
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

bool robotHasIntake() {
  return activeRobot().hasIntake && activeRobot().intake != NULL;
}

vex::drivetrain* drivetrainFromRobot() {
  if (!robotHasDrivetrain()) {
    return NULL;
  }

  return activeRobot().drivetrain;
}

vex::motor_group* intakeFromRobot() {
  if (!robotHasIntake()) {
    return NULL;
  }

  return activeRobot().intake;
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

double wrapHeadingDegrees(double degrees) {
  while (degrees < 0.0) {
    degrees += 360.0;
  }

  while (degrees >= 360.0) {
    degrees -= 360.0;
  }

  return degrees;
}

double smallestTurnDegrees(double requestedTurn) {
  while (requestedTurn <= -180.0) {
    requestedTurn += 360.0;
  }

  while (requestedTurn > 180.0) {
    requestedTurn -= 360.0;
  }

  return requestedTurn;
}

const char* waypointTypeName(WaypointType type) {
  switch (type) {
    case TARGET:
      return "TARGET";
    case OBJECT:
      return "OBJECT";
    case PARK:
      return "PARK";
    case COORD:
      return "COORD";
    default:
      return "UNKNOWN";
  }
}

const char* teamColorName(Color team) {
  switch (team) {
    case NONE:
      return "NONE";
    case BLUE:
      return "BLUE";
    case RED:
      return "RED";
    default:
      return "UNKNOWN";
  }
}

MotionProfile motionProfileForWaypoint(WaypointType type) {
  MotionProfile profile;

  // Different waypoint intents usually need different aggression levels.
  // These defaults are conservative so the prototype is easier to watch and
  // debug on the field.
  switch (type) {
    case TARGET:
      profile.driveVelocityPct = 35.0;
      profile.turnVelocityPct = 25.0;
      profile.stopMode = vex::brakeType::hold;
      break;
    case OBJECT:
      profile.driveVelocityPct = 40.0;
      profile.turnVelocityPct = 30.0;
      profile.stopMode = vex::brakeType::hold;
      break;
    case PARK:
      profile.driveVelocityPct = 60.0;
      profile.turnVelocityPct = 40.0;
      profile.stopMode = vex::brakeType::brake;
      break;
    case COORD:
    default:
      profile.driveVelocityPct = 50.0;
      profile.turnVelocityPct = 35.0;
      profile.stopMode = vex::brakeType::brake;
      break;
  }

  return profile;
}

void configureHardwareIfNeeded() {
  vex::drivetrain* driveBase = drivetrainFromRobot();

  if (HardwareConfigured) {
    return;
  }

  if (driveBase == NULL) {
    return;
  }

  // One-time drivetrain defaults that every autonomous run should inherit.
  driveBase->setTimeout(4, vex::timeUnits::sec);
  driveBase->setDriveVelocity(40, vex::percentUnits::pct);
  driveBase->setTurnVelocity(30, vex::percentUnits::pct);
  driveBase->setStopping(vex::brakeType::brake);

  activeRobot().leftDrive->setStopping(vex::brakeType::brake);
  activeRobot().rightDrive->setStopping(vex::brakeType::brake);

  HardwareConfigured = true;
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
}

Waypoint* cloneWaypoint(const Waypoint& source) {
  Waypoint* copy = (Waypoint*)malloc(sizeof(Waypoint));

  if (copy == NULL) {
    logToBrain("Route append failed: no memory");
    return NULL;
  }

  *copy = source;
  copy->next = NULL;

  return copy;
}

void resetDriveEncoders() {
  if (!robotHasDrivetrain()) {
    return;
  }

  activeRobot().leftDrive->resetPosition();
  activeRobot().rightDrive->resetPosition();
}

void copyConfiguredPoseIntoEstimate() {
  EstimatedPose = ConfiguredStartPose;
}

void updateEstimatedPoseAfterDrive(double signedDistanceInches, double headingDegrees) {
  const double radians = headingDegrees * (kPi / 180.0);

  EstimatedPose.xInches += cos(radians) * signedDistanceInches;
  EstimatedPose.yInches += sin(radians) * signedDistanceInches;
}

void stopIntake() {
  vex::motor_group* intake = intakeFromRobot();

  if (intake == NULL) {
    return;
  }

  intake->stop(vex::brakeType::coast);
}

void runIntakeForWaypoint(const Waypoint& waypoint) {
  vex::motor_group* intake = intakeFromRobot();

  if (waypoint.type != OBJECT && waypoint.type != TARGET) {
    stopIntake();
    return;
  }

  if (intake == NULL) {
    logToBrain("No intake for %s", waypointTypeName(waypoint.type));
    return;
  }

  if (waypoint.type == OBJECT) {
    logToBrain("Intake inward %.0f pct", kIntakeVelocityPct);
    intake->spin(kIntakeInwardDirection, kIntakeVelocityPct, vex::velocityUnits::pct);
    return;
  }

  logToBrain("Intake forward %.0f pct", kIntakeVelocityPct);
  intake->spin(kIntakeForwardDirection, kIntakeVelocityPct, vex::velocityUnits::pct);
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

void applyMotionProfile(const MotionProfile& profile) {
  vex::drivetrain* driveBase = drivetrainFromRobot();

  if (driveBase == NULL) {
    return;
  }

  driveBase->setDriveVelocity(profile.driveVelocityPct, vex::percentUnits::pct);
  driveBase->setTurnVelocity(profile.turnVelocityPct, vex::percentUnits::pct);
  driveBase->setStopping(profile.stopMode);
  activeRobot().leftDrive->setStopping(profile.stopMode);
  activeRobot().rightDrive->setStopping(profile.stopMode);
}

bool executeWaypoint(const Waypoint& waypoint, int waypointIndex) {
  const double targetHeading = wrapHeadingDegrees((double)waypoint.direction);
  const double requestedTurn =
      smallestTurnDegrees(targetHeading - EstimatedPose.headingDegrees);
  const MotionProfile profile = motionProfileForWaypoint(waypoint.type);
  const double signedDistance = (double)waypoint.distance;

  logToBrain("WP %d/%d %s %s",
             waypointIndex,
             RouteCount,
             waypointTypeName(waypoint.type),
             teamColorName(waypoint.team));
  logToBrain("Target heading %.1f deg", targetHeading);
  logToBrain("Profile drive %.0f turn %.0f",
             profile.driveVelocityPct,
             profile.turnVelocityPct);

  // Intake actions start before the move so the mechanism is already active
  // while the robot approaches an object or target.
  applyMotionProfile(profile);
  runIntakeForWaypoint(waypoint);
  {
    const double turnDelta = executeTurn(requestedTurn, profile);
    EstimatedPose.headingDegrees =
        wrapHeadingDegrees(EstimatedPose.headingDegrees + turnDelta);
  }

  // Drive uses the heading estimate produced by the completed PID turn.
  {
    const double driveDelta = executeDrive(signedDistance, profile);
    updateEstimatedPoseAfterDrive(driveDelta, EstimatedPose.headingDegrees);
  }

  logToBrain("Pose x=%.1f y=%.1f h=%.1f",
             EstimatedPose.xInches,
             EstimatedPose.yInches,
             EstimatedPose.headingDegrees);

  if (waypoint.type == PARK) {
    stopIntake();
    logToBrain("Park reached: stop route");
    return false;
  }

  if (waypoint.type == COORD) {
    logToBrain("Coord reached: continue");
  }

  stopIntake();
  return true;
}

WaypointType waypointTypeFromCode(int typeCode) {
  switch (typeCode) {
    case TARGET:
      return TARGET;
    case OBJECT:
      return OBJECT;
    case PARK:
      return PARK;
    case COORD:
    default:
      return COORD;
  }
}

Color colorFromCode(int colorCode) {
  switch (colorCode) {
    case BLUE:
      return BLUE;
    case RED:
      return RED;
    case NONE:
    default:
      return NONE;
  }
}
}  // namespace

void addwaypoint(float distance, int direction, int type, int color) {
  Waypoint waypoint;
  Waypoint* copy;

  waypoint.distance = distance;
  waypoint.direction = direction;
  waypoint.type = waypointTypeFromCode(type);
  waypoint.team = colorFromCode(color);
  waypoint.next = NULL;

  copy = cloneWaypoint(waypoint);

  if (copy == NULL) {
    return;
  }

  if (RouteHead == NULL) {
    RouteHead = copy;
    RouteTail = copy;
  } else {
    RouteTail->next = copy;
    RouteTail = copy;
  }

  RouteCount++;
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

void engineRunInternal(void) {
  Waypoint* current = RouteHead;
  int waypointIndex = 1;
  vex::drivetrain* driveBase = drivetrainFromRobot();

  if (!VerboseLoggingConfigured) {
    VerboseLoggingEnabled = kVerboseLoggingDefault;
  }
  HardwareConfigured = false;

  if (driveBase == NULL) {
    resetBrainLog();
    logToBrain("No drivetrain configured");
    return;
  }

  configureHardwareIfNeeded();
  resetDriveEncoders();
  copyConfiguredPoseIntoEstimate();

  if (VerboseLoggingEnabled) {
    resetBrainLog();
    logToBrain("Waypoint route start");
    logToBrain("Waypoints queued: %d", RouteCount);
    logToBrain("Start pose x=%.1f y=%.1f h=%.1f",
               EstimatedPose.xInches,
               EstimatedPose.yInches,
               EstimatedPose.headingDegrees);
  }

  if (current == NULL) {
    logToBrain("No waypoints queued");
    return;
  }

  while (current != NULL) {
    if (!executeWaypoint(*current, waypointIndex)) {
      break;
    }

    current = current->next;
    waypointIndex++;
  }

  stopIntake();
  driveBase->stop(vex::brakeType::brake);
  logToBrain("Route complete");
}
