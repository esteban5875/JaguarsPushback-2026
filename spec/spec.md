# Spec

## How to Debug

Turn Brain logging on with:

```cpp
setBrainVerbose(true);
```

Turn it off with:

```cpp
setBrainVerbose(false);
```

When verbose mode is on, the VEX Brain shows a short live log during autonomous.
This lets you see what the engine is trying to do without reading the engine code.

The log updates as the robot runs.
It is meant to answer simple questions like these:

- Did the engine see my waypoint?
- Which heading is it trying to face?
- Did it think this was an `OBJECT`, `TARGET`, `PARK`, or `COORD` step?
- Did it stop because I asked it to park?

### Brain Log Format

You will usually see lines like this:

```text
WP 1/4 OBJECT BLUE
Target heading 90.0 deg
Profile drive 40 turn 30
Intake inward 80 pct
Turn right 90.0 deg
Drive forward 18.0 in
Pose x=18.0 y=0.0 h=90.0
```

What those lines mean:

- `WP 1/4 OBJECT BLUE` = waypoint 1 out of 4, type `OBJECT`, color `BLUE`
- `Target heading 90.0 deg` = the robot wants to face 90 degrees
- `Profile drive 40 turn 30` = 40% drive speed and 30% turn speed
- `Intake inward 80 pct` = the intake is pulling inward at 80%
- `Turn right 90.0 deg` = the robot is turning right 90 degrees
- `Drive forward 18.0 in` = the robot is driving forward 18 inches
- `Pose x=18.0 y=0.0 h=90.0` = the engine's best estimate of robot position

The first line usually tells you which waypoint the robot is on.
The lines after that explain what the engine decided to do for that waypoint.

Common line meanings:

- `WP x/y ...` = which step is currently running
- `Target heading ...` = where the robot is trying to face
- `Profile drive ... turn ...` = the speed choice for this step
- `Intake inward ...` or `Intake forward ...` = intake action for this step
- `Turn left ...` or `Turn right ...` = turning action
- `Drive forward ...` or `Drive reverse ...` = drive action
- `Pose ...` = estimated robot position after the move

### Special Messages

`No drivetrain configured`

This means `loadPrototypeRobot()` did not create a drivetrain.
Add `RobotFactory::setDrivetrain(...)` before running autonomous.

`No waypoints queued`

This means `loadPrototypeRoute()` did not add any waypoints.
Add one or more `addwaypoint(...)` lines.

`No intake for OBJECT`

This means the route asked for intake behavior, but no intake was created.
Add `RobotFactory::setIntake(...)` if the robot should use an intake.

`No intake for TARGET`

This means the same thing, but for a target step instead of an object step.
The route is fine, but the robot setup is missing intake hardware.

`Park reached: stop route`

This means the robot reached a `PARK` waypoint and stopped on purpose.
This is normal.

`Coord reached: continue`

This means the robot reached a `COORD` waypoint and kept going.
This is also normal.

`Route complete`

This means the engine finished the active route.
If there was a `PARK` waypoint before the end, this message still appears after the stop.

### Quick Check

- If the robot does not move, check for `No drivetrain configured`.
- If the robot moves but intake does nothing, check for `No intake for OBJECT` or `No intake for TARGET`.
- If the route stops too early, check if a waypoint type was set to `3` for `PARK`.
- If the Brain screen is blank, make sure `setBrainVerbose(true);` was called before the route runs.

### Easy Debug Examples

If you expected the robot to keep going but the log shows:

```text
WP 2/5 PARK NONE
Park reached: stop route
Route complete
```

the route stopped because waypoint 2 was a `PARK` step.
Change that waypoint type if that was not your intention.

If you expected intake action but the log shows:

```text
WP 1/3 OBJECT RED
No intake for OBJECT
```

the route is fine, but the robot setup is missing `RobotFactory::setIntake(...)`.
Fix the robot setup, not the waypoint line.
