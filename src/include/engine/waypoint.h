#pragma once

#include "./data.h"

typedef struct Waypoint
{
    float distance;
    int direction;
    WaypointType type;
    Color team;
    Waypoint* next;
};
