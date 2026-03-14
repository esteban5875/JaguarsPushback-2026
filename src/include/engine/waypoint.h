#pragma once

#include "./data.h"

typedef struct Waypoint {
    // Linear distance, in inches, to drive once the robot is aimed correctly.
    float distance;

    // Absolute heading, in degrees, for this waypoint.
    // The engine converts this into the smaller relative turn needed from the
    // robot's current heading.
    int direction;

    // Metadata used by the engine to choose drive/turn behavior.
    WaypointType type;
    Color team;

    // Linked-list support so the route can stay lightweight and easy to inspect.
    struct Waypoint* next;
} Waypoint;
