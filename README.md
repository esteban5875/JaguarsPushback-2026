# Main

This project is built so `main.cpp` feels like a short list of commands.
You do not need to write drivetrain math, controller loops, or engine logic.

After this update, `main.cpp` only does four jobs:

1. add robot parts
2. set controller buttons
3. choose the driver movement type
4. add waypoints

`pre_auton.cpp` now does the startup work for you.
It resets the robot state and turns Brain verbose mode on before the main setup lines are loaded.

## Where To Write Things

Write robot hardware in `loadPrototypeRobot()`.
Write controller button and movement lines in `loadPrototypeButtons()`.
Write autonomous route lines in `loadPrototypeRoute()`.

That means `main.cpp` now reads like:

- what hardware the robot has
- which buttons run which action
- how the driver moves the drivetrain
- which autonomous steps should run

## Parts You Add In `loadPrototypeRobot()`

These are the setup wrappers you use there.
Think of them like build commands for the robot.

`RobotFactory::setController(ROBOT_CONTROLLER_PRIMARY);`

This adds the handheld controller to the robot build.
Use `ROBOT_CONTROLLER_PARTNER` instead if you want the partner controller.

`RobotFactory::setIntake(intake, 1);`

This adds the intake motor group.
The second number is how many motors are in the intake list.

`RobotFactory::setDrivetrain(leftDrive, 2, rightDrive, 2, 319.19, 320.0, 130.0, ROBOT_DISTANCE_MM, 1.0);`

This adds the drivetrain.
If this line is missing, autonomous and driver control cannot move the robot.

## Making Motor Lists

Before you call `setIntake(...)` or `setDrivetrain(...)`, you create motor lists.
Each motor uses three values:

- port number
- gear type
- reversed or not reversed

Gear wrapper values:

- `ROBOT_GEAR_36_1`
- `ROBOT_GEAR_18_1`
- `ROBOT_GEAR_6_1`

Example intake list:

```cpp
RobotMotor intake[1] = {
  {3, ROBOT_GEAR_18_1, false}
};
```

Example drivetrain lists:

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

If a motor spins the wrong way on the real robot, change `false` to `true`.
If one whole side of the drive feels backward, the right side often needs `true`.

The drivetrain numbers after the motor lists are physical robot values:

- wheel travel
- track width
- wheel base
- distance unit
- external gear ratio

## Buttons You Set In `loadPrototypeButtons()`

Use this wrapper there:

`setControllerButtons(intakeInButton, intakeForwardButton);`

This tells driver control which buttons should run the intake inward and which button should run it forward.
The driver module does not need any more intake button code than that.

Available button wrappers:

- `CONTROLLER_BUTTON_NONE`
- `CONTROLLER_BUTTON_L1`
- `CONTROLLER_BUTTON_L2`
- `CONTROLLER_BUTTON_R1`
- `CONTROLLER_BUTTON_R2`
- `CONTROLLER_BUTTON_UP`
- `CONTROLLER_BUTTON_DOWN`
- `CONTROLLER_BUTTON_LEFT`
- `CONTROLLER_BUTTON_RIGHT`
- `CONTROLLER_BUTTON_X`
- `CONTROLLER_BUTTON_B`
- `CONTROLLER_BUTTON_Y`
- `CONTROLLER_BUTTON_A`

Example:

```cpp
setControllerButtons(CONTROLLER_BUTTON_L1, CONTROLLER_BUTTON_L2);
```

Read that like this:

- `L1` pulls the intake inward
- `L2` pushes the intake forward

## Movement Type You Set In `loadPrototypeButtons()`

Use this wrapper there:

`set_movement_type(type);`

This tells driver control how the drivetrain should be driven.
You choose between `BUTTONS` mode and `AXIS` mode.

`set_movement_type(BUTTONS);`

This makes the controller arrow buttons drive the robot.
The arrows work like this:

- `Up` = forward
- `Down` = backward
- `Left` = turn left
- `Right` = turn right

You can press movement and turn buttons together.
For example, `Up` and `Left` together will drive forward while turning left.

`set_movement_type(AXIS);`

This makes the right joystick drive the robot.
The right joystick works like this:

- right stick up/down = forward and backward
- right stick left/right = turn

This is the default mode after reset.
If you do not set a movement type, the controller program falls back to `AXIS`.

## Waypoints You Add In `loadPrototypeRoute()`

Use this wrapper there:

`addwaypoint(distance, direction, type, color);`

Use one line per autonomous step.
Write the lines in the same order you want the robot to follow them.

What each part means:

- `distance` = how far to drive, in inches
- `direction` = what heading to face first
- `type` = what kind of step this is
- `color` = optional team color tag

Direction examples:

- `0` = face forward
- `90` = face right
- `180` = face backward
- `270` = face left

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
- treat the step as an `OBJECT`
- tag it as blue

Special waypoint behavior:

- `OBJECT` runs intake inward during the step
- `TARGET` runs intake forward during the step
- `PARK` stops the route when reached
- `COORD` keeps the route going normally

If you place a `PARK` waypoint in the middle of the list, the robot will stop there and ignore the later waypoint lines.
If you want the whole route to run, keep `PARK` at the end.

## What `pre_auton.cpp` Does Now

You no longer need to reset the robot in `main.cpp`.
You also no longer need to turn Brain verbose on there.

`pre_auton.cpp` now does this startup work:

- resets the robot wrapper
- resets the controller program
- turns Brain verbose on

That is why `main.cpp` can stay focused on parts, buttons, movement type, and waypoints.

## Simple Pattern To Follow

Use this order:

1. In `pre_auton.cpp`, let the reset and verbose setup happen.
2. In `loadPrototypeRobot()`, add controller, drivetrain, and intake.
3. In `loadPrototypeButtons()`, set the movement type and intake buttons.
4. In `loadPrototypeRoute()`, add the waypoint lines.

Small working template:

```cpp
void loadPrototypeRobot() {
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

  RobotFactory::setController(ROBOT_CONTROLLER_PRIMARY);
  RobotFactory::setDrivetrain(leftDrive, 2, rightDrive, 2, 319.19, 320.0, 130.0, ROBOT_DISTANCE_MM, 1.0);
  RobotFactory::setIntake(intake, 1);
}

void loadPrototypeButtons() {
  set_movement_type(AXIS);
  setControllerButtons(CONTROLLER_BUTTON_L1, CONTROLLER_BUTTON_L2);
}

void loadPrototypeRoute() {
  addwaypoint(24.0f, 0, 4, 0);
  addwaypoint(18.0f, 90, 2, 1);
  addwaypoint(12.0f, 180, 3, 0);
}
```

If you can edit numbers and button names in lines like these, you can use the project.

## Project Explanation

This project is split so each file has one clear job.
`main.cpp` describes the build and route, the spec layer builds the robot, the control layer builds driver behavior, and the engine runs autonomous movement.

That means the user-facing code stays simple.
The lower-level VEX objects, loops, and motion behavior stay inside the project modules.

The flow is:

1. `pre_auton.cpp` clears startup state and turns verbose on.
2. `main.cpp` adds parts.
3. `main.cpp` sets controller buttons and movement type.
4. `main.cpp` adds waypoints.
5. `auton.cpp` tells the engine to run autonomous.
6. `usercontrol.cpp` calls the controller program's `execute()` method.

## Tour

`src/main.cpp`

This is the file you edit most.
It is where you add parts, buttons, movement type, and waypoints.

`src/modules/pre_auton.cpp`

This prepares the robot before the main setup runs.
It resets the robot and turns Brain verbose on.

`src/include/spec/robot_factory.h`

This is the list of robot part wrappers.
If you want to know how to add hardware, start here.

`src/modules/spec/robot_factory.cpp`

This file builds the real VEX robot objects from your wrapper lines.
You usually only edit it when changing the architecture, not when changing ports.

`src/include/control.h`

This is the public wrapper for controller button setup and movement type setup.
This is where `setControllerButtons(...)` and `set_movement_type(...)` come from.

`src/modules/control/control_factory.cpp`

This builds the controller program and contains its `execute()` method.
That method is the only thing driver control needs to call.

`src/include/engine.h`

This is the public autonomous wrapper list.
The important main-facing function here is `addwaypoint(...)`.

`src/modules/engine/engine.cpp`

This is the autonomous engine.
It reads the robot, follows the waypoints, runs intake actions for `OBJECT` and `TARGET`, and stops on `PARK`.

`src/modules/usercontrol.cpp`

This is the driver control entrypoint.
It stays tiny because it only calls the controller program's `execute()` method.
