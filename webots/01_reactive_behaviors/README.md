# Understanding the `obstacle_avoidance_controller.py`: A Step-by-Step Guide

Robotic navigation in dynamic environments requires sophisticated algorithms to detect and circumvent obstacles efficiently. The `obstacle_avoidance_controller.py` script provides a comprehensive solution for enabling a robot to autonomously navigate by leveraging distance sensors and motor controls. This article delves into the intricacies of the script, explaining its structure, components, and operational logic step by step.

## Table of Contents

1. [Overview](#overview)
2. [Imports and Dependencies](#imports-and-dependencies)
3. [Class Definition and Constants](#class-definition-and-constants)
4. [Initialization](#initialization)
5. [Sensor Data Retrieval](#sensor-data-retrieval)
6. [Debugging Utilities](#debugging-utilities)
7. [Main Control Loop](#main-control-loop)
8. [State Handling Methods](#state-handling-methods)
    - [Handling Forward to Point A](#handling-forward-to-point-a)
    - [Handling 180-Degree Turn](#handling-180-degree-turn)
    - [Handling Forward to Point B](#handling-forward-to-point-b)
    - [Handling Clockwise Rotation](#handling-clockwise-rotation)
    - [Handling Wall Following](#handling-wall-following)
9. [Execution Entry Point](#execution-entry-point)
10. [Conclusion](#conclusion)

---

## Overview

The `ObstacleAvoidanceController` class orchestrates the robot's movements to avoid obstacles by transitioning through various states based on sensor inputs. These states include moving forward, executing turns, rotating, and following walls. The controller utilizes distance sensors to detect obstacles and adjusts motor speeds accordingly to navigate the environment effectively.

---

## Imports and Dependencies

```python
from controller import Robot, Motor, DistanceSensor
import math
```

- **`controller` Module**: Provides classes for interacting with robot hardware components like motors and sensors.
- **`math` Module**: Though imported, it isn't utilized in the current script but can be useful for future enhancements involving mathematical computations.

---

## Class Definition and Constants

```python
class ObstacleAvoidanceController:
    """
    A controller for robot obstacle avoidance using distance sensors and motor control.
    The robot navigates through different states to detect and avoid obstacles.
    """
```

This class encapsulates all functionalities related to obstacle avoidance. It defines various constants and state identifiers crucial for the robot's decision-making process.

### Constants

```python
# Constants
DISTANCE_THRESHOLD = 0.05  # 0.05 meters threshold
MAX_SPEED = 5.0            # Maximum speed for the wheels
TURN_SPEED = 2.0           # Speed for turning
TURN_180_DURATION = 4000   # milliseconds for 180-degree turn
```

- **`DISTANCE_THRESHOLD`**: The minimum distance (in meters) at which the robot considers an object as an obstacle.
- **`MAX_SPEED`**: The top speed the robot's wheels can achieve.
- **`TURN_SPEED`**: The speed used when the robot needs to turn.
- **`TURN_180_DURATION`**: The time (in milliseconds) required to execute a 180-degree turn.

### States

```python
# States
STATE_FORWARD_TO_A = 0
STATE_TURN_180 = 1
STATE_FORWARD_TO_B = 2
STATE_ROTATE_CLOCKWISE = 3
STATE_WALL_FOLLOWING = 4

state_names = {
    STATE_FORWARD_TO_A: "FORWARD_TO_A",
    STATE_TURN_180: "TURN_180",
    STATE_FORWARD_TO_B: "FORWARD_TO_B",
    STATE_ROTATE_CLOCKWISE: "ROTATE_CLOCKWISE",
    STATE_WALL_FOLLOWING: "WALL_FOLLOWING"
}
```

The robot operates through a finite state machine with the following states:

1. **`FORWARD_TO_A`**: Moving forward towards Point A.
2. **`TURN_180`**: Executing a 180-degree turn upon obstacle detection.
3. **`FORWARD_TO_B`**: Moving forward towards Point B.
4. **`ROTATE_CLOCKWISE`**: Rotating clockwise to align with a wall.
5. **`WALL_FOLLOWING`**: Following the detected wall.

The `state_names` dictionary maps state identifiers to their string representations for easier debugging and logging.

---

## Initialization

```python
def __init__(self):
    """Initialize the robot, motors, sensors, and state variables."""
    # Initialize the robot
    self.robot = Robot()
    self.timestep = int(self.robot.getBasicTimeStep())

    # Initialize motors
    self.motor_left = self.robot.getDevice('left wheel motor')
    self.motor_right = self.robot.getDevice('right wheel motor')
    self.motor_left.setPosition(float('inf'))
    self.motor_right.setPosition(float('inf'))

    # Initialize distance sensors
    self.distance_sensors = []
    for i in range(8):
        sensor = self.robot.getDevice(f'ps{i}')
        sensor.enable(self.timestep)
        self.distance_sensors.append(sensor)

    # Initialize state
    self.current_state = self.STATE_FORWARD_TO_A
    self.previous_state = None
    self.turn_start_time = 0
```

The constructor initializes the robot's hardware components and sets up the initial state:

1. **Robot Initialization**: Creates an instance of the `Robot` class and retrieves the simulation's basic time step.
2. **Motor Initialization**: Retrieves the left and right wheel motors, setting their positions to infinity to allow velocity control rather than position control.
3. **Distance Sensor Initialization**: Retrieves and enables eight distance sensors (`ps0` to `ps7`), storing them in a list for easy access.
4. **State Variables**: Sets the initial state to `FORWARD_TO_A`, with placeholders for tracking state changes and turn durations.

---

## Sensor Data Retrieval

### Front Distance

```python
def get_front_distance(self):
    """
    Retrieve the maximum distance from the front sensors (ps0 and ps7).

    Returns:
        float: The maximum front distance value.
    """
    left_front = self.distance_sensors[0].getValue()
    right_front = self.distance_sensors[7].getValue()
    print(f"Raw front sensor values - Left: {left_front}, Right: {right_front}")
    return max(left_front, right_front)
```

- **Purpose**: Obtains readings from the front-facing sensors (`ps0` and `ps7`).
- **Operation**: Fetches raw values from both sensors, logs them for debugging, and returns the maximum value to represent the front distance.

### Left Distance

```python
def get_left_distance(self):
    """
    Retrieve the distance from the left sensor (ps5).

    Returns:
        float: The left distance value.
    """
    return self.distance_sensors[5].getValue()
```

- **Purpose**: Obtains the reading from the left-side sensor (`ps5`).
- **Operation**: Returns the raw value from the specified sensor.

---

## Debugging Utilities

```python
def print_debug_info(self, front_distance, left_distance, raw_front, raw_left):
    """
    Print debug information about the current state and sensor readings.

    Args:
        front_distance (float): Processed front distance in meters.
        left_distance (float): Processed left distance in meters.
        raw_front (float): Raw front sensor value.
        raw_left (float): Raw left sensor value.
    """
    if self.current_state != self.previous_state:
        print(f"\n--- State Change: {self.state_names[self.current_state]} ---")
        self.previous_state = self.current_state

    print(f"State: {self.state_names[self.current_state]}")
    print(f"Raw Front Value: {raw_front}")
    print(f"Raw Left Value: {raw_left}")
    print(f"Front Distance: {front_distance:.3f}m")
    print(f"Left Distance: {left_distance:.3f}m")

    if self.current_state == self.STATE_TURN_180:
        elapsed_time = self.robot.getTime() * 1000 - self.turn_start_time
        turn_percentage = (elapsed_time / self.TURN_180_DURATION) * 100
        print(f"Turn Progress: {turn_percentage:.1f}%")
    print("-------------------")
```

This method serves as a logging utility to provide insights into the robot's current state and sensor readings:

- **State Change Detection**: Logs a message whenever the robot transitions to a new state.
- **Sensor Readings**: Displays both raw and processed (converted to meters) sensor values.
- **Turn Progress**: If the robot is executing a 180-degree turn, it calculates and logs the progress percentage based on elapsed time.

---

## Main Control Loop

```python
def run(self):
    """Main control loop for obstacle avoidance."""
    while self.robot.step(self.timestep) != -1:
        # Read raw sensor values
        raw_front = self.get_front_distance()
        raw_left = self.get_left_distance()
        front_distance = raw_front / 100.0
        left_distance = raw_left / 100.0

        # Print debug information
        self.print_debug_info(front_distance, left_distance, raw_front, raw_left)

        # State machine
        if self.current_state == self.STATE_FORWARD_TO_A:
            self.handle_forward_to_a(raw_front)
        elif self.current_state == self.STATE_TURN_180:
            self.handle_turn_180()
        elif self.current_state == self.STATE_FORWARD_TO_B:
            self.handle_forward_to_b(raw_front)
        elif self.current_state == self.STATE_ROTATE_CLOCKWISE:
            self.handle_rotate_clockwise(raw_left)
        elif self.current_state == self.STATE_WALL_FOLLOWING:
            self.handle_wall_following(raw_left)
```

The `run` method is the heart of the controller, executing repeatedly at each simulation timestep:

1. **Sensor Reading**: Fetches raw front and left sensor values.
2. **Distance Conversion**: Converts raw sensor values to meters by dividing by 100 (assuming sensor values are in centimeters).
3. **Debug Logging**: Calls `print_debug_info` to log current state and sensor data.
4. **State Machine Execution**: Based on the current state, invokes the corresponding handler method to determine the next action.

---

## State Handling Methods

Each state in the finite state machine is managed by a dedicated handler method. These methods encapsulate the logic required to transition between states and control the robot's movements accordingly.

### Handling Forward to Point A

```python
def handle_forward_to_a(self, raw_front):
    """
    Handle the FORWARD_TO_A state where the robot moves forward towards point A.

    Args:
        raw_front (float): Raw front sensor value.
    """
    if raw_front > 80:  # ~0.05m threshold
        print("*** DETECTED BOX A - STARTING 180 TURN ***")
        self.current_state = self.STATE_TURN_180
        self.turn_start_time = self.robot.getTime() * 1000
        self.motor_left.setVelocity(0)
        self.motor_right.setVelocity(0)
    else:
        self.motor_left.setVelocity(self.MAX_SPEED)
        self.motor_right.setVelocity(self.MAX_SPEED)
```

- **Objective**: Move the robot forward towards Point A until an obstacle is detected.
- **Logic**:
    - If the front sensor detects an object closer than the threshold (raw value > 80), transition to the `TURN_180` state.
    - Otherwise, set both motors to `MAX_SPEED` to move forward.

### Handling 180-Degree Turn

```python
def handle_turn_180(self):
    """Handle the TURN_180 state where the robot performs a 180-degree turn."""
    elapsed_time = self.robot.getTime() * 1000 - self.turn_start_time

    if elapsed_time >= self.TURN_180_DURATION:
        print("*** COMPLETED 180 TURN - MOVING TO BOX B ***")
        self.current_state = self.STATE_FORWARD_TO_B
        self.motor_left.setVelocity(0)
        self.motor_right.setVelocity(0)
    else:
        turn_multiplier = 3.0
        self.motor_left.setVelocity(-self.TURN_SPEED * turn_multiplier)
        self.motor_right.setVelocity(self.TURN_SPEED * turn_multiplier)
```

- **Objective**: Execute a 180-degree turn to navigate away from the detected obstacle.
- **Logic**:
    - **Turn Execution**: While the elapsed time is less than `TURN_180_DURATION`, rotate the robot by setting the left motor to a negative speed and the right motor to a positive speed.
    - **Turn Completion**: Once the turn duration is met, transition to the `FORWARD_TO_B` state and halt the motors.

### Handling Forward to Point B

```python
def handle_forward_to_b(self, raw_front):
    """
    Handle the FORWARD_TO_B state where the robot moves forward towards point B.

    Args:
        raw_front (float): Raw front sensor value.
    """
    if raw_front > 80:  # ~0.05m threshold
        print("*** DETECTED BOX B - STARTING CLOCKWISE ROTATION ***")
        self.current_state = self.STATE_ROTATE_CLOCKWISE
        self.motor_left.setVelocity(0)
        self.motor_right.setVelocity(0)
    else:
        self.motor_left.setVelocity(self.MAX_SPEED)
        self.motor_right.setVelocity(self.MAX_SPEED)
```

- **Objective**: Move the robot forward towards Point B until another obstacle is detected.
- **Logic**:
    - If the front sensor detects an obstacle within the threshold, transition to the `ROTATE_CLOCKWISE` state.
    - Otherwise, continue moving forward by setting both motors to `MAX_SPEED`.

### Handling Clockwise Rotation

```python
def handle_rotate_clockwise(self, raw_left):
    """
    Handle the ROTATE_CLOCKWISE state where the robot rotates clockwise until a wall is detected on the left.

    Args:
        raw_left (float): Raw left sensor value.
    """
    if raw_left > 80:  # ~0.05m threshold
        print("*** LEFT SENSOR DETECTED WALL - STARTING WALL FOLLOWING ***")
        self.current_state = self.STATE_WALL_FOLLOWING
        self.motor_left.setVelocity(self.MAX_SPEED)
        self.motor_right.setVelocity(self.MAX_SPEED)
    else:
        # Rotate clockwise until left sensor detects the wall
        self.motor_left.setVelocity(self.TURN_SPEED)
        self.motor_right.setVelocity(-self.TURN_SPEED)
```

- **Objective**: Rotate the robot clockwise until it aligns with a wall detected by the left sensor.
- **Logic**:
    - If the left sensor detects a wall within the threshold, transition to the `WALL_FOLLOWING` state and set both motors to `MAX_SPEED`.
    - Otherwise, continue rotating clockwise by setting the left motor to `TURN_SPEED` and the right motor to `-TURN_SPEED`.

### Handling Wall Following

```python
def handle_wall_following(self, raw_left):
    """
    Handle the WALL_FOLLOWING state where the robot follows a wall detected on the left.

    Args:
        raw_left (float): Raw left sensor value.
    """
    if raw_left > 80:  # ~0.05m threshold
        # Continue moving forward while wall is detected on left
        self.motor_left.setVelocity(self.MAX_SPEED)
        self.motor_right.setVelocity(self.MAX_SPEED)
    else:
        # Stop if wall is lost
        print("*** WALL LOST - STOPPING ***")
        self.motor_left.setVelocity(0)
        self.motor_right.setVelocity(0)
```

- **Objective**: Follow the detected wall by maintaining a consistent distance.
- **Logic**:
    - If the left sensor continues to detect the wall within the threshold, keep moving forward by setting both motors to `MAX_SPEED`.
    - If the wall is lost (i.e., the left sensor no longer detects it), halt the robot by setting both motors to zero.

---

## Execution Entry Point

```python
if __name__ == "__main__":
    controller = ObstacleAvoidanceController()
    controller.run()
```

This block ensures that when the script is executed directly, an instance of `ObstacleAvoidanceController` is created, and the main control loop is initiated via the `run` method.

---

## Conclusion

The `obstacle_avoidance_controller.py` script presents a well-structured approach to enabling a robot to navigate autonomously while avoiding obstacles. By employing a finite state machine and utilizing distance sensors for environmental perception, the robot can make informed decisions to maneuver effectively. Key takeaways from the script include:

- **Modular Design**: Each state is handled by a dedicated method, promoting clarity and ease of maintenance.
- **Sensor Integration**: Real-time sensor data guides the robot's actions, ensuring responsive and adaptive behavior.
- **Debugging Support**: Comprehensive logging aids in monitoring the robot's state transitions and sensor readings, facilitating troubleshooting and optimization.

This controller serves as a solid foundation for more advanced robotic navigation systems, allowing for further enhancements such as dynamic obstacle handling, path planning, and integration with other sensor modalities.

