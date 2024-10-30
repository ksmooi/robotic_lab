# Controller Programming in Webots: A Comprehensive Guide

**Webots User Guide R2023b**

Webots is a versatile and powerful open-source robotics simulator that facilitates the development and testing of robot controllers. This guide provides a detailed overview of controller programming within Webots, focusing on various aspects such as sensor integration, actuator control, and system synchronization. While the primary focus is on the C programming language, the principles discussed are applicable across multiple languages including C++, Java, Python, and MATLAB. For more detailed information on language-specific functions and methods, refer to the sections on Nodes and API Functions and the respective language documentation.

## Table of Contents
1. [Hello World Example](#hello-world-example)
2. [Reading Sensors](#reading-sensors)
3. [Using Actuators](#using-actuators)
4. [The "step" and "wb_robot_step" Functions](#the-step-and-wb_robot_step-functions)
5. [Using Sensors and Actuators Together](#using-sensors-and-actuators-together)
6. [Using Controller Arguments](#using-controller-arguments)
7. [Controller Termination](#controller-termination)
8. [Console Output](#console-output)
9. [Shared Libraries](#shared-libraries)
10. [Environment Variables](#environment-variables)
11. [Language Settings](#language-settings)

---

## Hello World Example

In the tradition of computer science, starting with a "Hello World!" example is essential for understanding the basics of controller programming. Below is a simple "Hello World!" controller example for Webots:

```python
from controller import Robot

robot = Robot()

while robot.step(32) != -1:
    print("Hello World!")
```

This script continuously prints "Hello World!" to the standard output stream, which Webots redirects to its console. This behavior is consistent across all supported languages, ensuring that both standard output and error streams are seamlessly integrated into the Webots environment.

### C API Initialization

When using the C API, controller code must include specific header files and initialize communication with Webots:

```c
#include <webots/robot.h>
#include <stdio.h>

int main() {
    wb_robot_init();
    
    while (wb_robot_step(32) != -1) {
        printf("Hello World!\n");
    }
    
    wb_robot_cleanup();
    return 0;
}
```

Key Points:
- **Initialization**: `wb_robot_init()` initializes communication between the controller and Webots.
- **Step Function**: `wb_robot_step(32)` synchronizes the controller's data with the simulator every 32 milliseconds of simulation time.
- **Cleanup**: `wb_robot_cleanup()` gracefully terminates the communication upon exiting the loop.

---

## Reading Sensors

Integrating sensors into your robot controller is fundamental for interaction with the simulated environment. Below is an example demonstrating how to read values from a DistanceSensor:

```python
from controller import Robot, DistanceSensor

TIME_STEP = 32

robot = Robot()

sensor = robot.getDevice("my_distance_sensor")
sensor.enable(TIME_STEP)

while robot.step(TIME_STEP) != -1:
    value = sensor.getValue()
    print("Sensor value is:", value)
```

### Key Concepts

- **Device Tag Retrieval**: Before utilizing a device, obtain its tag using `wb_robot_get_device`. The device name (e.g., "my_distance_sensor") must match the name defined in the robot's `.wbt` or `.proto` file.
- **Enabling Sensors**: Sensors must be enabled using the corresponding `wb_*_enable` function with an update delay specified in milliseconds. This delay typically matches the control step (`TIME_STEP`), ensuring synchronization between sensor updates and controller cycles.
- **Data Retrieval**: Sensor values are updated during the `wb_robot_step` function call. Use functions like `wb_distance_sensor_get_value` to access the latest sensor data.

### Vector Sensors

Certain sensors, such as GPS, Accelerometers, and Gyroscopes, return vector values. These are accessed as arrays of three floating-point numbers representing the x, y, and z axes:

```python
values = gps.getValues()
print("MY_ROBOT is at position:", values[0], values[1], values[2])
```

**Important**: In C and C++, the returned pointer should not be explicitly deleted or modified, as the memory management is handled internally by Webots.

---

## Using Actuators

Actuators, such as motors, allow your robot to perform actions within the simulation. Below is an example of making a rotational motor oscillate with a 2 Hz sine wave signal:

```python
from controller import Robot, Motor
from math import pi, sin

TIME_STEP = 32

robot = Robot()
motor = robot.getDevice("my_motor")

FREQUENCY = 2.0  # 2 Hz
t = 0.0          # Elapsed simulation time

while robot.step(TIME_STEP) != -1:
    position = sin(t * 2.0 * pi * FREQUENCY)
    motor.setPosition(position)
    t += TIME_STEP / 1000.0
```

### Key Concepts

- **Device Tag Retrieval**: Similar to sensors, actuators are accessed using `wb_robot_get_device`.
- **Setting Positions**: Use functions like `wb_motor_set_position` to define target positions. Actuation occurs during the subsequent `wb_robot_step` call.
- **Control Loop**: Typically implemented using an infinite loop, where each iteration computes and sets new actuator targets based on the desired behavior.

### Simultaneous Actuation

When controlling multiple actuators, set the target positions for each before making a single `wb_robot_step` call to ensure synchronized actuation:

```python
while robot.step(TIME_STEP) != -1:
    # Set target positions for multiple motors
    left_motor.setPosition(left_target)
    right_motor.setPosition(right_target)
```

---

## The "step" and "wb_robot_step" Functions

Webots operates using two distinct time steps:

1. **Simulation Step**: Defined in the Scene Tree under `WorldInfo.basicTimeStep`, it represents the duration of one simulation step in milliseconds.
2. **Control Step**: Specified as an argument to `wb_robot_step`, it denotes the duration of each iteration of the control loop.

### Synchronization Requirements

- **Atomic Operations**: Each simulation step is atomic and cannot be interrupted. Sensor measurements and motor actuation occur between simulation steps.
- **Multiples of Simulation Step**: The control step must be a multiple of the simulation step to ensure proper synchronization. For example, if the simulation step is 16 ms, valid control steps include 16 ms, 32 ms, 64 ms, etc.

### Step Function Usage

```python
while robot.step(32) != -1:
    # Controller logic here
```

**Note**: The `wb_robot_step` function advances the controller's time, synchronizes sensor and actuator data, and must be called regularly within the control loop.

---

## Using Sensors and Actuators Together

Webots and each robot controller run in separate processes, necessitating careful synchronization when interacting with sensors and actuators. Below are guidelines and examples to ensure coherent interaction.

### Common Pitfalls

- **Overwriting Actuator Commands**: Setting multiple target positions before a `wb_robot_step` call can lead to earlier commands being overridden.
  
  ```python
  # BAD: The first setPosition is ignored
  my_leg.setPosition(0.34)
  my_leg.setPosition(0.56)
  robot.step(40)
  ```

- **Redundant Sensor Readings**: Reading sensor values multiple times without stepping can result in identical data.

  ```python
  # WRONG: d2 will always equal d1
  d1 = sensor.getValue()
  d2 = sensor.getValue()
  if d2 > d1:
      avoidCollision()
  ```

### Recommended Approach

Implement a single `wb_robot_step` call per control loop iteration to update all sensors and actuators simultaneously:

```python
while robot.step(40) != -1:
    readSensors()
    actuateMotors()
```

### Complete Example: Differential Steering with Proximity Sensors

```python
from controller import Robot, Motor, DistanceSensor

TIME_STEP = 32

robot = Robot()

# Initialize sensors
left_sensor = robot.getDevice("left_sensor")
right_sensor = robot.getDevice("right_sensor")
left_sensor.enable(TIME_STEP)
right_sensor.enable(TIME_STEP)

# Initialize motors
left_motor = robot.getDevice("left_motor")
right_motor = robot.getDevice("right_motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

while robot.step(TIME_STEP) != -1:
    # Read sensor values
    left_dist = left_sensor.getValue()
    right_dist = right_sensor.getValue()

    # Compute motor speeds based on sensor input
    left_speed = compute_left_speed(left_dist, right_dist)
    right_speed = compute_right_speed(left_dist, right_dist)

    # Actuate motors
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
```

**Note**: Ensure that `compute_left_speed` and `compute_right_speed` are user-defined functions that determine motor speeds based on sensor inputs.

---

## Using Controller Arguments

Controller arguments allow you to pass parameters to controllers, enabling customization for different robots. These arguments are specified in the `.wbt` file within the `controllerArgs` field of the `Robot` node and are accessible as parameters in the controller's main function.

### Example Scenario

Given the following `Robot` node configuration:

```plaintext
Robot {
  ...
  controllerArgs "one two three"
  ...
}
```

A sample controller code to access these arguments:

```python
from controller import Robot
import sys

robot = Robot()

for i in range(1, len(sys.argv)):
    print(f"argv[{i}]={sys.argv[i]}")
```

**Output**:

```plaintext
argv[1]=one
argv[2]=two
argv[3]=three
```

**Note**: In MATLAB, include the optional argument `varargin` in the function declaration to handle controller arguments.

---

## Controller Termination

Controllers typically run in an infinite loop until terminated by Webots due to events such as:

- Webots quitting
- Simulation reset
- World reloading
- New simulation loading
- Controller name change

### Handling Termination

When a termination event occurs, `wb_robot_step` returns `-1`, signaling the controller to perform any necessary cleanup before exiting. Here's how to handle termination gracefully:

```python
from controller import Robot, DistanceSensor

TIME_STEP = 32

robot = Robot()

sensor = robot.getDevice("my_distance_sensor")
sensor.enable(TIME_STEP)

while robot.step(TIME_STEP) != -1:
    value = sensor.getValue()
    print("Sensor value is:", value)

# Termination detected
saveExperimentData()  # Ensure this completes within one second
```

### Manual Termination

In scenarios where the controller determines that the simulation should end (e.g., upon finding a solution), the controller should save any necessary data and terminate itself:

```python
import sys

# Condition to terminate the simulation
if finished:
    saveExperimentData()
    sys.exit(0)
```

**Note**: The exit status of a controller is ignored by Webots. Terminating the controller will freeze the simulation at the current step.

---

## Console Output

Webots redirects all standard output (`stdout`) and error (`stderr`) streams from controllers to its console. While Webots does not support `stdin` input, it does allow for basic ANSI escape codes to format console output, including:

- 3-bit color (foreground and background)
- Bold style
- Underline style
- Clear screen
- Reset styles and colors

### Example Usage

```python
from controller import AnsiCodes

print("This is " + AnsiCodes.RED_FOREGROUND + "red" + AnsiCodes.RESET + "!")
```

**Note**: If the console output is altered due to previous escape codes without a reset, recompiling, cleaning, or manually clearing the console will restore default settings.

---

## Shared Libraries

Shared libraries facilitate code reuse across multiple controllers and plugins. It is recommended to store these libraries in a subdirectory within the project's `libraries` directory. This approach ensures that the necessary paths are included in the `[[DY]LD_LIBRARY_]PATH` environment variable, depending on the operating system.

### Creating Shared Libraries

1. **Directory Structure**: Place shared libraries in a subdirectory of the `libraries` directory within your project.
2. **Compilation**: Use the main Makefile (`WEBOTS_HOME/resources/Makefile.include`) to compile shared libraries, ensuring they link correctly with Controller libraries, ODE, or the Qt framework.
3. **Environment Variable**: If libraries are stored outside the recommended directory, use the `WEBOTS_LIBRARY_PATH` environment variable to specify their locations.

### Example

The Qt utility library is a good reference for shared library implementation, located at:

```plaintext
WEBOTS_HOME/resources/projects/libraries/qt_utils
```

---

## Environment Variables

Environment variables can be crucial for configuring controller behavior without altering system-wide settings. Webots allows the definition of environment variables specific to controller processes via a `runtime.ini` configuration file.

### `runtime.ini` Configuration

- **Location**: Place the `runtime.ini` file in the controller's directory.
- **Format**: Follows the standard INI file format with key-value pairs under designated sections.
- **Sections**:
  - `[environment variables with paths]`: For variables containing relative or absolute paths.
  - `[environment variables]`: For non-path-related variables.
  - `[environment variables for Windows]`: Specific to Windows platforms.
  - `[environment variables for macOS]`: Specific to macOS platforms.
  - `[environment variables for Linux]`: Specific to Linux platforms.
  - `[environment variables for Linux 32]`: Specific to 32-bit Linux systems.
  - `[environment variables for Linux 64]`: Specific to 64-bit Linux systems.

### Example `runtime.ini`

```ini
; typical runtime.ini

[environment variables with paths]
WEBOTS_LIBRARY_PATH = lib:$(WEBOTS_LIBRARY_PATH):../../library

[environment variables]
ROS_MASTER_URI = http://localhost:11311

[environment variables for Windows]
NAOQI_LIBRARY_FOLDER = "bin;C:\Users\My Documents\Naoqi\bin"

[environment variables for macOS]
NAOQI_LIBRARY_FOLDER = lib

[environment variables for Linux]
NAOQI_LIBRARY_FOLDER = lib
```

**Key Features**:
- **Path Syntax**: Use colon `:` separators and forward slashes `/` for directories. On Windows, colons are replaced by semicolons `;` and slashes by backslashes `\`.
- **Variable References**: Environment variables can reference others using the `$(VARIABLE_NAME)` syntax.
- **Platform-Specific Settings**: Ensure variables are correctly defined for the target operating system to avoid conflicts.

---

## Language Settings

The `runtime.ini` file also supports language-specific configurations, allowing customization of the interpreter or compiler settings for different programming languages used in controllers.

### Language-Specific Sections

- **[java]**
- **[python]**
- **[matlab]**

Each section can include:
- `COMMAND`: Specifies the path to the language interpreter.
- `OPTIONS`: Defines additional options passed to the interpreter.

### Example Configurations

#### Python Controller on macOS

```ini
[python]
COMMAND = /opt/local/bin/python3.8
OPTIONS = -m package.name.given
```

**Resulting Command**:
```plaintext
/opt/local/bin/python3.8 -m package.name.given my_controller.py
```

#### Java Controller on Windows

```ini
[environment variables with paths]
CLASSPATH = ../lib/MyLibrary.jar
JAVA_LIBRARY_PATH = ../lib

[java]
COMMAND = javaw.exe
OPTIONS = -Xms6144k
```

**Notes**:
- The `CLASSPATH` environment variable automatically generates the `-classpath` option for the Java Virtual Machine, incorporating the Webots Controller libraries and any additional specified libraries.
- Do not manually add `-classpath` to the `OPTIONS` key; instead, define it within the `CLASSPATH` environment variable.

---

# Conclusion

Controller programming in Webots encompasses a range of functionalities, from basic output operations to intricate sensor-actuator integrations and environment configurations. By adhering to the guidelines and examples provided in this guide, developers can effectively create robust and efficient robot controllers tailored to their specific simulation needs. Whether utilizing C, C++, Java, Python, or MATLAB, Webots offers the flexibility and tools necessary to advance in robotics simulation and development.

For more detailed information on specific API functions or language-specific implementations, refer to the respective sections within the Webots User Guide and the language documentation.

