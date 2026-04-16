#pragma once

// Drive action: positive = forward, negative = reverse
void driveAction(double signedDistanceInches);

// Turn action: positive = right, negative = left
void turnAction(double signedDegrees);

// Intake action: direction > 0 = forward, direction < 0 = reverse, duration in milliseconds
void intakeAction(int direction, int durationMs);

// UpTake action: direction > 0 = forward, direction < 0 = reverse, duration in milliseconds
void uptakeAction(int direction, int durationMs);

// MidTake action: direction > 0 = forward, direction < 0 = reverse, duration in milliseconds
void midtakeAction(int direction, int durationMs);

// Scoop activate
void scoopActivate();
// Scoop deactivate
void scoopDeactivate();

// Arm activate
void armActivate();

// Arm deactivate
void armDeactivate();