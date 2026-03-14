#pragma once

#include "./robot.h"

// Internal access for modules that need the built Robot instance.
// main should stay on the public RobotFactory setter wrappers.
Robot& getRobotInternal(void);
