#pragma once

#include "vex.h"

// Public engine API:
// distance = inches to drive after turning to the requested heading
// direction = absolute heading in degrees
// type = 1 TARGET, 2 OBJECT, 3 PARK, 4 COORD
// color = 0 ignore, 1 BLUE, 2 RED
// pistonAction = 0 no action, 1 open piston, 2 close piston
//
// Controls whether the engine writes detailed movement logs to the VEX Brain.
void setBrainVerbose(bool enabled);

// Writes a final idle-state marker to the VEX Brain verbose log.
void logProgramHalt(void);
