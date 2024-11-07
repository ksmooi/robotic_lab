# Understanding `odometry_counter.py`: A Step-by-Step Guide

Odometry is a fundamental concept in robotics, enabling a robot to estimate its position and orientation based on wheel movements. The `odometry_counter.py` script provides an object-oriented approach to tracking a robot's movement using ground sensors and motor controls. This article breaks down the script, elucidating its structure, components, and operational logic to help you understand how the robot navigates and determines when it has reached its destination.

## Table of Contents

1. [Overview](#overview)
2. [Imports and Dependencies](#imports-and-dependencies)
3. [Class Definition and Constants](#class-definition-and-constants)
4. [Initialization](#initialization)
    - [Robot and Time Step Initialization](#robot-and-time-step-initialization)
    - [Sensor and Motor Initialization](#sensor-and-motor-initialization)
    - [Odometry and Position Variables](#odometry-and-position-variables)
    - [Initial Movement](#initial-movement)
5. [Ground Sensor Configuration](#ground-sensor-configuration)
6. [Motor Configuration](#motor-configuration)
7. [Motor Velocity Control](#motor-velocity-control)
8. [Sensor Reading](#sensor-reading)
9. [Odometry Calculations](#odometry-calculations)
    - [Displacement (`delta_s`)](#displacement-delta_s)
    - [Rotation (`delta_omega`)](#rotation-delta_omega)
    - [Position Update](#position-update)
    - [Error Distance Calculation](#error-distance-calculation)
10. [Control Logic](#control-logic)
11. [Main Control Loop](#main-control-loop)
12. [Execution Entry Point](#execution-entry-point)
13. [Conclusion](#conclusion)

---

## Overview

The `OdometryCounter` class is designed to track a robot's movement by calculating its position and orientation based on wheel encoder data and ground sensor inputs. It employs a finite state machine approach to navigate the robot, adjusting motor velocities based on sensor readings to follow a line and determine when the robot has reached its destination.

---

## Imports and Dependencies

```python
import math
from controller import Robot, Motor, DistanceSensor
```

- **`math` Module**: Utilized for mathematical operations, specifically for trigonometric calculations involved in odometry.
- **`controller` Module**: Typically associated with robot simulation environments like Webots, it provides classes to interact with robot hardware components such as motors and sensors.

---

## Class Definition and Constants

```python
class OdometryCounter:
    """
    OdometryCounter handles the tracking of the robot's position and orientation
    using wheel encoders and ground sensors. It controls motor velocities based
    on sensor inputs to navigate the robot and determine when the destination
    is reached.
    """
    
    # Constants
    MAX_SPEED = 6.28  # Maximum speed in radians/second
    WHEEL_RADIUS = 0.0201  # meters
    WHEEL_DISTANCE = 0.052  # meters
    DISTANCE_THRESHOLD = 2.825  # meters
    ERROR_THRESHOLD = 0.071  # meters
```

The `OdometryCounter` class encapsulates all functionalities related to tracking and controlling the robot's movement. It defines several constants crucial for motion calculations and navigation logic:

- **`MAX_SPEED`**: Defines the maximum rotational speed of the robot's wheels in radians per second.
- **`WHEEL_RADIUS`**: The radius of the robot's wheels in meters, essential for converting rotational speed to linear displacement.
- **`WHEEL_DISTANCE`**: The distance between the two wheels in meters, used in calculating the robot's rotation based on differential speeds.
- **`DISTANCE_THRESHOLD`**: The target distance the robot aims to travel before considering its journey complete.
- **`ERROR_THRESHOLD`**: The allowable deviation from the target position, determining when the robot has effectively reached its destination.

---

## Initialization

The `__init__` method sets up the robot's hardware components, initializes sensors and motors, and prepares odometry and position tracking variables.

### Robot and Time Step Initialization

```python
def __init__(self):
    """
    Initializes the OdometryCounter by setting up the robot, sensors, motors,
    and initial position variables. It also starts the robot moving forward
    slightly to avoid immediate start line detection.
    """
    # Initialize the Robot instance and timestep
    self.robot = Robot()
    self.timestep = int(self.robot.getBasicTimeStep())
    self.time_step_seconds = self.timestep / 1000.0  # Convert to seconds
```

- **Robot Instance**: Creates an instance of the `Robot` class to interface with the robot's hardware.
- **Time Step**: Retrieves the simulation's basic time step, converting it from milliseconds to seconds for use in calculations.

### Sensor and Motor Initialization

```python
    # Initialize sensors and motors
    self.ground_sensors = self.initialize_ground_sensors(3)
    self.left_motor, self.right_motor = self.initialize_motors()
```

- **Ground Sensors**: Initializes three ground sensors (`gs0`, `gs1`, `gs2`) used for line detection.
- **Motors**: Initializes the left and right wheel motors, configuring them for velocity control.

### Odometry and Position Variables

```python
    # Initialize odometry variables
    self.total_distance = 0.0
    self.total_rotation = 0.0  # in radians

    # Initialize position variables
    self.x_pos = 0.0
    self.y_pos = 0.0
    self.z_pos = 0.0  # Will remain 0 for 2D movement
```

- **Odometry Variables**:
    - **`total_distance`**: Accumulates the total distance the robot has traveled.
    - **`total_rotation`**: Accumulates the total rotation (in radians) the robot has undergone.
- **Position Variables**:
    - **`x_pos` & `y_pos`**: Track the robot's position in a 2D plane.
    - **`z_pos`**: Remains zero as the robot operates in a 2D environment.

### Initial Movement

```python
    # Start moving forward slightly to avoid immediate start line detection
    self.set_motor_velocity(self.MAX_SPEED, self.MAX_SPEED)
    self.robot.step(1000)  # Move for 1 second
```

- **Purpose**: Prevents the robot from immediately detecting the start line by moving it slightly forward.
- **Operation**:
    - Sets both motors to `MAX_SPEED` to move the robot forward.
    - Advances the simulation by 1000 milliseconds (1 second) using `self.robot.step(1000)`, during which the robot moves forward.

---

## Ground Sensor Configuration

```python
def initialize_ground_sensors(self, count):
    """
    Initializes the specified number of ground sensors.

    Args:
        count (int): Number of ground sensors to initialize.

    Returns:
        list: A list of initialized ground sensor devices.
    """
    sensors = []
    for i in range(count):
        sensor = self.robot.getDevice(f'gs{i}')
        sensor.enable(self.timestep)
        sensors.append(sensor)
    return sensors
```

- **Purpose**: Sets up ground sensors used for detecting the line's position relative to the robot.
- **Operation**:
    - Iterates over the number of sensors specified by `count` (in this case, 3).
    - Retrieves each ground sensor device (`gs0`, `gs1`, `gs2`).
    - Enables each sensor with the simulation's time step.
    - Stores the initialized sensors in a list for later use.

---

## Motor Configuration

```python
def initialize_motors(self):
    """
    Initializes the left and right wheel motors.

    Returns:
        tuple: A tuple containing the left and right motor devices.
    """
    left_motor = self.robot.getDevice('left wheel motor')
    right_motor = self.robot.getDevice('right wheel motor')
    
    # Set the motors to run indefinitely
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    return left_motor, right_motor
```

- **Purpose**: Configures the left and right wheel motors for velocity control.
- **Operation**:
    - Retrieves the left and right wheel motor devices.
    - Sets their positions to infinity (`float('inf')`) to enable continuous rotation, allowing for velocity-based control.
    - Returns the motor objects for further manipulation.

---

## Motor Velocity Control

```python
def set_motor_velocity(self, left_velocity, right_velocity):
    """
    Sets the velocities of the left and right motors.

    Args:
        left_velocity (float): Velocity for the left motor.
        right_velocity (float): Velocity for the right motor.
    """
    self.left_motor.setVelocity(left_velocity)
    self.right_motor.setVelocity(right_velocity)
```

- **Purpose**: Provides a convenient method to set the velocities of both motors simultaneously.
- **Operation**:
    - Applies the specified velocities to the left and right motors, controlling the robot's movement.

---

## Sensor Reading

```python
def read_ground_sensors(self):
    """
    Reads the current values from all ground sensors.

    Returns:
        list: A list of sensor values.
    """
    return [sensor.getValue() for sensor in self.ground_sensors]
```

- **Purpose**: Fetches the latest readings from all ground sensors.
- **Operation**:
    - Iterates through each ground sensor and retrieves its current value.
    - Returns the sensor values as a list for further processing.

---

## Odometry Calculations

Odometry involves calculating the robot's movement based on wheel rotations. This section explains how the script calculates displacement and rotation to estimate the robot's position and orientation.

### Displacement (`delta_s`)

```python
delta_s = (self.WHEEL_RADIUS * (phildot + phirdot) / 2.0) * self.time_step_seconds
```

- **Purpose**: Calculates the linear displacement of the robot during the current timestep.
- **Calculation**:
    - **Average Wheel Speed**: \((v_{left} + v_{right}) / 2\) gives the average rotational speed of the wheels.
    - **Wheel Radius**: Multiplying by `WHEEL_RADIUS` converts rotational speed to linear speed.
    - **Time Step**: Multiplying by `time_step_seconds` gives the distance traveled during the timestep.
- **Formula**: \(\Delta s = r \times \frac{(v_{left} + v_{right})}{2} \times \Delta t\)

### Rotation (`delta_omega`)

```python
delta_omega = (self.WHEEL_RADIUS * (phirdot - phildot) / self.WHEEL_DISTANCE) * self.time_step_seconds
```

- **Purpose**: Determines the change in the robot's orientation during the current timestep.
- **Calculation**:
    - **Speed Difference**: \((v_{right} - v_{left})\) captures the differential in wheel speeds, causing rotation.
    - **Wheel Distance**: Dividing by `WHEEL_DISTANCE` normalizes the rotation based on the distance between wheels.
    - **Wheel Radius and Time Step**: These factors convert rotational speed to angular displacement over the timestep.
- **Formula**: \(\Delta \omega = r \times \frac{(v_{right} - v_{left})}{d} \times \Delta t\)

### Position Update

```python
# Calculate position change in global coordinates (swapped x and y)
delta_x = delta_s * math.cos(self.total_rotation)  # Changed from delta_y
delta_y = -delta_s * math.sin(self.total_rotation)  # Changed from delta_x and added negative

# Update position
self.y_pos += delta_y
self.x_pos += delta_x
```

- **Purpose**: Updates the robot's position in the global coordinate system based on the calculated displacement and rotation.
- **Operation**:
    - **Delta Calculations**:
        - **`delta_x`**: Determines the change in the X-coordinate based on the current orientation.
        - **`delta_y`**: Determines the change in the Y-coordinate, with a negative sign to account for the coordinate system orientation.
    - **Position Update**: Adds the calculated deltas to the current position coordinates.

### Error Distance Calculation

```python
# Compute error distance from origin
error_distance = math.sqrt(self.x_pos**2 + self.y_pos**2)
return error_distance
```

- **Purpose**: Calculates the Euclidean distance from the robot's current position to the origin (0,0,0).
- **Operation**:
    - Uses the Pythagorean theorem to determine the straight-line distance.
    - Returns the `error_distance` to assess how far the robot is from its target.

### Full Odometry Method

```python
def calculate_odometry(self, phildot, phirdot):
    """
    Calculates the robot's displacement and rotation based on wheel velocities.

    Args:
        phildot (float): Velocity of the left wheel.
        phirdot (float): Velocity of the right wheel.

    Returns:
        float: The error distance from the origin.
    """
    # Calculate displacement and rotation
    delta_s = (self.WHEEL_RADIUS * (phildot + phirdot) / 2.0) * self.time_step_seconds
    delta_omega = (self.WHEEL_RADIUS * (phirdot - phildot) / self.WHEEL_DISTANCE) * self.time_step_seconds

    # Update total rotation
    self.total_rotation += delta_omega

    # Calculate position change in global coordinates (swapped x and y)
    delta_x = delta_s * math.cos(self.total_rotation)  # Changed from delta_y
    delta_y = -delta_s * math.sin(self.total_rotation)  # Changed from delta_x and added negative

    # Update position
    self.y_pos += delta_y
    self.x_pos += delta_x

    # Update total distance
    self.total_distance += abs(delta_s)

    # Compute error distance from origin
    error_distance = math.sqrt(self.x_pos**2 + self.y_pos**2)
    return error_distance
```

This method integrates all the steps discussed above to update the robot's odometry based on current wheel velocities.

---

## Control Logic

```python
def control_logic(self, sensor_values):
    """
    Determines motor velocities based on ground sensor readings.

    Args:
        sensor_values (list): Current readings from ground sensors.

    Returns:
        tuple: Desired velocities for the left and right motors.
    """
    if sensor_values[0] > 500 and sensor_values[1] < 350 and sensor_values[2] > 500:
        # On line, drive straight
        return self.MAX_SPEED, self.MAX_SPEED
    elif sensor_values[2] < 550:
        # Line is on the right, turn right
        return 0.25 * self.MAX_SPEED, -0.1 * self.MAX_SPEED
    elif sensor_values[0] < 550:
        # Line is on the left, turn left
        return -0.1 * self.MAX_SPEED, 0.25 * self.MAX_SPEED
    else:
        # Default case, search by turning
        return 0.15 * self.MAX_SPEED, -0.15 * self.MAX_SPEED
```

- **Purpose**: Determines the appropriate motor velocities based on the current readings from the ground sensors.
- **Operation**:
    - **Conditions**:
        - **On Line**: If both side sensors (`gs0` and `gs2`) detect high values (indicating no line) and the center sensor (`gs1`) detects a low value (indicating the line is directly beneath), the robot is considered to be on the line and should move straight.
        - **Line on the Right**: If the right sensor (`gs2`) detects a low value, indicating the line is towards the right, the robot should turn right to realign.
        - **Line on the Left**: If the left sensor (`gs0`) detects a low value, indicating the line is towards the left, the robot should turn left to realign.
        - **Default Case**: If none of the above conditions are met, the robot doesn't detect the line and should search for it by turning in place.
    - **Motor Speed Adjustments**:
        - **Driving Straight**: Both motors are set to `MAX_SPEED` to move forward.
        - **Turning Right**: The left motor is set to a quarter of `MAX_SPEED` while the right motor moves in reverse at a tenth of `MAX_SPEED` to initiate a right turn.
        - **Turning Left**: The left motor moves in reverse at a tenth of `MAX_SPEED` while the right motor is set to a quarter of `MAX_SPEED` to initiate a left turn.
        - **Searching for Line**: Both motors are set to move in opposite directions at fifteen percent of `MAX_SPEED` to turn in place, scanning for the line.

---

## Main Control Loop

```python
def run(self):
    """
    The main loop that runs the odometry counter. It reads sensor values,
    updates motor velocities, calculates odometry, and checks if the
    destination is reached.
    """
    while self.robot.step(self.timestep) != -1:
        # Read ground sensors
        sensor_values = self.read_ground_sensors()

        # Determine motor velocities based on sensor input
        phildot, phirdot = self.control_logic(sensor_values)
        self.set_motor_velocity(phildot, phirdot)

        # Update odometry
        error_distance = self.calculate_odometry(phildot, phirdot)

        # Convert rotation to degrees for readability
        rotation_degrees = (self.total_rotation / math.pi) * 180.0

        # Print position, orientation, and error distance information
        if self.total_distance > self.DISTANCE_THRESHOLD:
            print(f"  Position         : ({self.x_pos:.3f}, {self.y_pos:.3f}, {self.z_pos:.3f}) meters")
            print(f"  Distance traveled: {self.total_distance:.3f} meters")
            print(f"  Current rotation : {rotation_degrees:.1f} degrees")
            print(f"  Error distance   : {error_distance:.3f} meters\n")

        # Check if the robot has reached the destination
        if (self.total_distance > self.DISTANCE_THRESHOLD) and (error_distance <= self.ERROR_THRESHOLD):
            self.set_motor_velocity(0, 0)
            print("Reached destination\n")
            print(f"  Distance traveled: {self.total_distance:.3f} meters")
            print(f"  Current rotation : {rotation_degrees:.1f} degrees")
            print(f"  Error distance   : {error_distance:.3f} meters\n")
            break
```

The `run` method embodies the robot's primary operational loop, executing repeatedly at each simulation timestep. Here's a breakdown of its components:

### Sensor Reading

```python
sensor_values = self.read_ground_sensors()
```

- **Purpose**: Retrieves the latest readings from all ground sensors.
- **Operation**: Calls the `read_ground_sensors` method to fetch current sensor values.

### Motor Velocity Determination

```python
phildot, phirdot = self.control_logic(sensor_values)
self.set_motor_velocity(phildot, phirdot)
```

- **Purpose**: Determines and sets the appropriate motor velocities based on sensor inputs.
- **Operation**:
    - **Control Logic**: Uses the `control_logic` method to determine the desired velocities (`phildot` and `phirdot`) for the left and right motors.
    - **Motor Setting**: Applies the determined velocities using the `set_motor_velocity` method.

### Odometry Update

```python
error_distance = self.calculate_odometry(phildot, phirdot)
```

- **Purpose**: Updates the robot's position and orientation based on the current motor velocities.
- **Operation**: Calls the `calculate_odometry` method with the current wheel velocities to compute the new position and rotation, returning the `error_distance` from the origin.

### Rotation Conversion

```python
rotation_degrees = (self.total_rotation / math.pi) * 180.0
```

- **Purpose**: Converts the total rotation from radians to degrees for easier interpretation.
- **Operation**: Uses the mathematical relationship between radians and degrees to perform the conversion.

### Odometry Information Display

```python
if self.total_distance > self.DISTANCE_THRESHOLD:
    print(f"  Position         : ({self.x_pos:.3f}, {self.y_pos:.3f}, {self.z_pos:.3f}) meters")
    print(f"  Distance traveled: {self.total_distance:.3f} meters")
    print(f"  Current rotation : {rotation_degrees:.1f} degrees")
    print(f"  Error distance   : {error_distance:.3f} meters\n")
```

- **Purpose**: Provides real-time feedback on the robot's position, distance traveled, current rotation, and error distance.
- **Operation**:
    - **Condition**: Only prints information if the `total_distance` exceeds `DISTANCE_THRESHOLD` to reduce verbosity during initial movement.
    - **Display**: Formats and prints the position coordinates, total distance traveled, current rotation in degrees, and the error distance from the origin.

### Destination Check

```python
if (self.total_distance > self.DISTANCE_THRESHOLD) and (error_distance <= self.ERROR_THRESHOLD):
    self.set_motor_velocity(0, 0)
    print("Reached destination\n")
    print(f"  Distance traveled: {self.total_distance:.3f} meters")
    print(f"  Current rotation : {rotation_degrees:.1f} degrees")
    print(f"  Error distance   : {error_distance:.3f} meters\n")
    break
```

- **Purpose**: Determines whether the robot has reached its intended destination.
- **Operation**:
    - **Conditions**:
        - **Distance Threshold**: Ensures the robot has traveled beyond a minimum distance (`DISTANCE_THRESHOLD`).
        - **Error Threshold**: Ensures the robot's current position is within an acceptable error margin (`ERROR_THRESHOLD`) from the origin.
    - **Action Upon Reaching Destination**:
        - **Motor Stopping**: Sets both motors' velocities to zero, halting the robot.
        - **Feedback**: Prints a confirmation message along with final odometry data.
        - **Loop Termination**: Breaks out of the main control loop, effectively ending the script.

---

## Execution Entry Point

```python
if __name__ == "__main__":
    odometry_counter = OdometryCounter()
    odometry_counter.run()
```

- **Purpose**: Ensures that the `OdometryCounter` class is instantiated and the main control loop is initiated when the script is executed directly.
- **Operation**:
    - **Instantiation**: Creates an instance of the `OdometryCounter` class, triggering the initialization process.
    - **Run Method**: Calls the `run` method to start the main control loop, enabling the robot to begin navigation and odometry tracking.

---

## Conclusion

The `odometry_counter.py` script presents a robust framework for tracking a robot's movement and orientation using wheel encoders and ground sensors. By integrating odometry calculations with sensor-based motor control, the robot can autonomously navigate towards a destination while monitoring its progress. Key aspects of the script include:

- **Modular Design**: The script's object-oriented structure promotes code organization, making it easier to manage and extend.
- **Sensor Integration**: Utilizing multiple ground sensors allows the robot to make informed decisions about its trajectory relative to the line.
- **Adaptive Motor Control**: Adjusting motor speeds based on sensor readings ensures responsive and precise navigation.
- **Odometry Tracking**: Calculating and accumulating displacement and rotation provides quantitative data on the robot's movement, aiding in navigation accuracy and system debugging.
- **Destination Determination**: Implementing distance and error thresholds ensures the robot can determine when it has successfully reached its target.

### Potential Enhancements

- **PID Control**: Implementing Proportional-Integral-Derivative (PID) controllers for smoother and more accurate line-following behavior.
- **Obstacle Avoidance**: Integrating additional sensors (e.g., ultrasonic or infrared) to detect and avoid obstacles in the robot's path.
- **Advanced Odometry**: Incorporating more sophisticated odometry techniques, such as integrating encoder ticks or using simultaneous localization and mapping (SLAM) for better position estimation.
- **Error Correction**: Implementing mechanisms to correct cumulative odometry errors over time, enhancing navigation reliability.

By understanding the components and logic of the `odometry_counter.py` script, developers and robotics enthusiasts can appreciate the intricacies of autonomous robot navigation and apply similar principles to their own projects, paving the way for more advanced and capable robotic systems.

