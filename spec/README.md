# Main

This project is built so `main.cpp` feels like a small list of commands.
You do not need to know how motors, drivetrain math, or engine code work.
In `main.cpp`, you only do two jobs:

1. Tell the project what robot you have.
2. Tell the project where the robot should go.

## How To Use The Wrappers

Think of each wrapper like a command.
You write one line, change the numbers, and the project handles the hard part.

The project already gives you two places to write those commands:

1. `loadPrototypeRobot()` for robot setup
2. `loadPrototypeRoute()` for autonomous steps

Put robot hardware lines in `loadPrototypeRobot()`.
Put route lines in `loadPrototypeRoute()`.

## Robot Setup Commands

Write these inside `loadPrototypeRobot()`.

`RobotFactory::reset();`

Use this first.
It clears the old robot setup so you start fresh.

If you forget this, old setup data can stay in memory.
That can make debugging confusing, so this should stay as the first line.

`RobotFactory::setController(ROBOT_CONTROLLER_PRIMARY);`

Use this if you want the main handheld controller.
If you want the partner controller, use `ROBOT_CONTROLLER_PARTNER`.

This is optional for autonomous movement.
It mainly tells the robot wrapper which controller belongs to the build.

`RobotFactory::setBrain();`

Use this only if you want to say clearly that the robot has a brain.
Most of the time you can skip it because the robot already starts with one.

`RobotFactory::setIntake(intake, 1);`

Use this after you make an intake motor list.
This tells the project which motor or motors are the intake.

The second number is the motor count.
If your intake uses two motors, pass `2` instead of `1`.

`RobotFactory::setDrivetrain(leftDrive, 2, rightDrive, 2, 319.19, 320.0, 130.0, ROBOT_DISTANCE_MM, 1.0);`

Use this after you make the left and right drive motor lists.
This is the command that gives the engine a drivetrain to move.

The values after the motor lists are the same kind of physical values the VEX drivetrain uses:

- wheel travel
- track width
- wheel base
- distance unit
- external gear ratio

If this command is missing, the engine cannot move the robot.
In that case the Brain log will say `No drivetrain configured`.

## Making A Motor List

Before `setIntake(...)` or `setDrivetrain(...)`, you make a small list of motors.
Each motor uses:

- port number
- gear type
- reversed or not reversed

The available gear wrappers are:

- `ROBOT_GEAR_36_1`
- `ROBOT_GEAR_18_1`
- `ROBOT_GEAR_6_1`

Example:

```cpp
RobotMotor leftDrive[2] = {
  {1, ROBOT_GEAR_18_1, false},
  {2, ROBOT_GEAR_18_1, false}
};
```

Read that like this:

- motor on port 1
- 18:1 cartridge
- normal direction

and:

- motor on port 2
- 18:1 cartridge
- normal direction

If a motor spins the wrong way on the real robot, change `false` to `true`.
If one whole side of the drivetrain drives backward, usually the right side needs `true`.

Example intake with one motor:

```cpp
RobotMotor intake[1] = {
  {3, ROBOT_GEAR_18_1, false}
};
```

Example drivetrain with two motors per side:

```cpp
RobotMotor leftDrive[2] = {
  {1, ROBOT_GEAR_18_1, false},
  {2, ROBOT_GEAR_18_1, false}
};

RobotMotor rightDrive[2] = {
  {9, ROBOT_GEAR_18_1, true},
  {10, ROBOT_GEAR_18_1, true}
};
```

After writing those lists, connect them with:

```cpp
RobotFactory::setDrivetrain(leftDrive,
                            2,
                            rightDrive,
                            2,
                            319.19,
                            320.0,
                            130.0,
                            ROBOT_DISTANCE_MM,
                            1.0);
```

## Route Commands

Write these inside `loadPrototypeRoute()`.

`setBrainVerbose(true);`

Use `true` if you want the VEX Brain to show a live movement log.
Use `false` if you want the robot to run quietly.

This only changes logging.
It does not change how the robot drives.

`addwaypoint(distance, direction, type, color);`

Use one line like this for each step in the route.
Write the steps in the same order you want the robot to follow them.

What each part means:

- `distance` = how far to drive
- `direction` = which heading to face first
- `type` = what kind of step this is
- `color` = optional team color tag

Important details:

- `distance` is in inches
- `direction` is an absolute heading in degrees
- `0` means face straight ahead from the starting direction
- `90` means face right
- `180` means face backward
- `270` means face left

Type values:

- `1` = `TARGET`
- `2` = `OBJECT`
- `3` = `PARK`
- `4` = `COORD`

Color values:

- `0` = no color
- `1` = blue
- `2` = red

Example:

```cpp
addwaypoint(24.0f, 90, 2, 1);
```

Read that like this:

- turn to 90 degrees
- drive 24 inches
- treat this as an `OBJECT` step
- tag it as blue

Another example:

```cpp
addwaypoint(12.0f, 180, 3, 0);
```

Read that like this:

- turn to face backward
- drive 12 inches
- treat it as a `PARK` step
- do not attach a color tag

Extra behavior:

- `OBJECT` makes the intake pull inward
- `TARGET` makes the intake push forward
- `PARK` ends the route when reached
- `COORD` continues normally

That means if you place a `PARK` waypoint in the middle of the list, the robot will stop there and ignore the lines after it.
If you want the whole route to run, keep `PARK` as the last meaningful step.

## Simple Pattern To Follow

If you want the shortest possible recipe, build `main.cpp` in this order:

1. Reset robot.
2. Add controller if needed.
3. Create motor lists.
4. Set drivetrain.
5. Set intake if you have one.
6. Turn Brain verbose on or off.
7. Add waypoints in order.

Small template:

```cpp
void loadPrototypeRobot() {
  RobotFactory::reset();

  RobotMotor leftDrive[2] = {
    {1, ROBOT_GEAR_18_1, false},
    {2, ROBOT_GEAR_18_1, false}
  };

  RobotMotor rightDrive[2] = {
    {9, ROBOT_GEAR_18_1, true},
    {10, ROBOT_GEAR_18_1, true}
  };

  RobotMotor intake[1] = {
    {3, ROBOT_GEAR_18_1, false}
  };

  RobotFactory::setDrivetrain(leftDrive, 2, rightDrive, 2, 319.19, 320.0, 130.0, ROBOT_DISTANCE_MM, 1.0);
  RobotFactory::setIntake(intake, 1);
}

void loadPrototypeRoute() {
  setBrainVerbose(true);
  addwaypoint(24.0f, 0, 4, 0);
  addwaypoint(18.0f, 90, 2, 1);
  addwaypoint(12.0f, 180, 3, 0);
}
```

If you can edit numbers in lines like these, you can use the project.

## Project Explanation

This project is split so each file has one job.
`main.cpp` is the command sheet, the spec layer builds the robot, and the engine turns waypoints into movement.

That means you only write simple setup and route lines in `main.cpp`.
The engine handles turning, driving, intake actions, stop rules, and Brain logs.

The flow is:

1. `main.cpp` builds the robot with wrapper commands.
2. `main.cpp` adds waypoints with wrapper commands.
3. `auton.cpp` tells the engine to start.
4. The engine reads the robot and runs the path.
5. The Brain log shows what happened if verbose mode is on.

## Tour

`src/main.cpp`

This is the main file you edit.
It is where you describe the robot and the route.

`src/include/spec/robot_factory.h`

This is the list of robot setup wrappers.
If you want to know which robot commands you can call, look here.

`src/modules/spec/robot_factory.cpp`

This file builds the real VEX robot objects from your wrapper lines.
You usually do not need to edit it when only changing ports or route steps.

`src/include/engine.h`

This is the list of public engine wrappers.
Right now the important ones are `setBrainVerbose(...)` and `addwaypoint(...)`.

`src/modules/engine/engine.cpp`

This is the movement engine.
It reads the robot, follows the waypoints, runs the intake when needed, and writes the Brain log.

It also decides special behavior:

- `OBJECT` runs intake inward
- `TARGET` runs intake forward
- `PARK` stops the route
- `COORD` keeps going

`src/modules/auton.cpp`

This starts autonomous.
It stays small because the engine does the real work.
