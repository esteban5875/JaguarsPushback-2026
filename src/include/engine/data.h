#pragma once

// High-level labels for why a waypoint exists.
// The engine uses these labels to choose a simple motion profile so we can
// tune different autonomous intents independently later on.
typedef enum WaypointType {
    TARGET = 1,
    OBJECT = 2,
    PARK = 3,
    COORD = 4
} WaypointType;

// Team metadata is carried with every waypoint so verbose logs can tell us
// which side of the field a route was intended for.
typedef enum Color {
    NONE = 0,
    BLUE = 1,
    RED = 2
} Color;

