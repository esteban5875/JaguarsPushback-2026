#pragma once

#include "vex.h"

#include "../engine.h"
#include "./data.h"
#include "./waypoint.h"

// Internal VEX object used by the competition stage modules.
// main should not include this file.
extern vex::competition Competition;

// Internal autonomous execution entrypoint.
void engineRunInternal(void);
