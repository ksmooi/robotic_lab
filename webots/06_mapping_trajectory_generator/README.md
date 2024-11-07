# Understanding `trajectory_generator.py`: A Step-by-Step Comprehensive Guide

Autonomous navigation is a fundamental aspect of modern robotics, enabling robots to perform complex tasks with minimal human intervention. The `trajectory_generator.py` script serves as a robust framework for guiding a differential drive robot along a predefined set of waypoints. This script integrates various sensor inputs, motor controls, odometry calculations, occupancy mapping, and visualization techniques to achieve precise and efficient navigation. This article provides a detailed, step-by-step explanation of the `trajectory_generator.py` script, elucidating its structure, components, and operational logic.

## Table of Contents

1. [Overview](#overview)
2. [Imports and Dependencies](#imports-and-dependencies)
3. [Class Definition and Constants](#class-definition-and-constants)
4. [Initialization](#initialization)
    - [Robot and Time Step Initialization](#robot-and-time-step-initialization)
    - [Sensor and Motor Initialization](#sensor-and-motor-initialization)
    - [Display and Marker Initialization](#display-and-marker-initialization)
    - [LIDAR Initialization](#lidar-initialization)
    - [Mapping Structures Initialization](#mapping-structures-initialization)
    - [Waypoints Initialization](#waypoints-initialization)
    - [Angle Array Initialization](#angle-array-initialization)
    - [Initial Movement](#initial-movement)
5. [Motor Control Methods](#motor-control-methods)
    - [Initializing Motors](#initializing-motors)
    - [Setting Motor Velocities](#setting-motor-velocities)
6. [Sensor Reading Methods](#sensor-reading-methods)
    - [Initializing Sensors](#initializing-sensors)
    - [Retrieving Robot Pose](#retrieving-robot-pose)
7. [Odometry and Positioning](#odometry-and-positioning)
    - [World to Map Coordinate Transformation](#world-to-map-coordinate-transformation)
    - [Calculating Control Errors](#calculating-control-errors)
    - [Calculating Motor Speeds](#calculating-motor-speeds)
8. [Waypoint Management](#waypoint-management)
    - [Updating Waypoints](#updating-waypoints)
    - [Stopping the Robot](#stopping-the-robot)
9. [Occupancy Mapping and Visualization](#occupancy-mapping-and-visualization)
    - [Processing LIDAR Data](#processing-lidar-data)
    - [Drawing the Trajectory](#drawing-the-trajectory)
    - [Convolving and Displaying the Map](#convolving-and-displaying-the-map)
10. [Main Control Loop](#main-control-loop)
11. [Execution Entry Point](#execution-entry-point)
12. [Conclusion](#conclusion)

---

## Overview

The `TrajectoryGenerator` class is engineered to control a differential drive robot, enabling it to follow a predefined set of waypoints accurately. It leverages GPS and compass sensors for localization, employs a proportional controller to adjust wheel speeds based on distance and heading errors, and integrates LIDAR for environmental perception and occupancy mapping. Additionally, it provides visualization capabilities using Matplotlib to display the robot's trajectory and the occupancy map of its environment.

---

## Imports and Dependencies

```python
from controller import Robot, Supervisor
import numpy as np
from scipy import signal
from matplotlib import pyplot as plt
```

- **`controller` Module**: Associated with robot simulation environments like Webots, this module provides classes (`Robot`, `Supervisor`) to interact with robot hardware components.
    - **`Robot`**: Base class for robot instances.
    - **`Supervisor`**: Specialized class that extends `Robot`, providing higher-level control over the simulation, such as accessing and modifying node fields.
  
- **`numpy`**: A fundamental package for numerical computations in Python, used here for array operations and mathematical functions.

- **`scipy.signal`**: Utilized for signal processing tasks, such as convolution, which is employed in occupancy mapping.

- **`matplotlib.pyplot`**: A plotting library used for visualizing the occupancy map and robot trajectory.

---

## Class Definition and Constants

```python
class TrajectoryGenerator:
    """
    TrajectoryGenerator is responsible for controlling a robot's movement through a predefined set of waypoints.
    It processes sensor data (GPS, Compass, Lidar) to navigate, updates an occupancy map based on Lidar readings,
    and visualizes the robot's trajectory and environment.

    Attributes:
        MOTOR_MAX_SPEED (float): Maximum speed for the robot's motors in radians per second.
        DISTANCE_GAIN (float): Proportional gain for distance correction.
        HEADING_GAIN (float): Gain for heading correction.
        WHEEL_RADIUS (float): Radius of the robot's wheels in meters.
        AXLE_LENGTH (float): Distance between the robot's wheels in meters.
        TURN_ANGLE (float): Angle threshold (in radians) to determine when a significant turn is needed.

        HEADING_CONTROL_GAIN (float): Gain for heading correction in motor speed calculation.
        DISTANCE_CONTROL_GAIN (float): Gain for distance correction in motor speed calculation.

        MAP_WIDTH (int): Width of the occupancy grid map in pixels.
        MAP_HEIGHT (int): Height of the occupancy grid map in pixels.

        robot (Supervisor): Supervisor instance to interact with the simulation environment.
        timestep (int): Simulation time step.
        left_motor (Motor): Left wheel motor controller.
        right_motor (Motor): Right wheel motor controller.
        gps (GPS): GPS sensor for position tracking.
        compass (Compass): Compass sensor for orientation tracking.
        display (Display): Display device for visualizing the map and trajectory.
        marker (Field): Marker object in the simulation to indicate current waypoint.
        lidar (Lidar): Lidar sensor for environment scanning.
        map (ndarray): Occupancy grid map representing the environment.
        trajectory_map (ndarray): Map tracking the robot's trajectory.
        kernel (ndarray): Convolution kernel for processing the occupancy map.
        WP (list of tuples): List of waypoints (x, y) the robot should navigate through.
        index (int): Current waypoint index.
        robot_stop (bool): Flag indicating whether the robot should stop.
        angles (ndarray): Array of angles corresponding to Lidar readings.
    """

    # Robot physical constants
    MOTOR_MAX_SPEED = 6.28        # Maximum speed for TIAGo motors in radians per second
    DISTANCE_GAIN = 1.0           # Proportional constant for distance correction
    HEADING_GAIN = 5.0            # Constant for heading correction
    WHEEL_RADIUS = 0.10           # Wheel radius in meters
    AXLE_LENGTH = 0.455           # Distance between wheels in meters
    TURN_ANGLE = np.pi            # 180 degrees in radians

    # Motor control gains
    HEADING_CONTROL_GAIN = 2.8    # Gain for heading correction in motor speed calculation
    DISTANCE_CONTROL_GAIN = 2.6   # Gain for distance correction in motor speed calculation

    # Occupancy grid dimensions
    MAP_WIDTH = 250               # Width of occupancy grid in pixels
    MAP_HEIGHT = 300              # Height of occupancy grid in pixels
```

### Explanation of Constants

- **Robot Physical Constants**:
    - **`MOTOR_MAX_SPEED`**: The upper limit for the robot's motor speeds, set to 6.28 radians per second (~1 revolution per second).
    - **`DISTANCE_GAIN` & `HEADING_GAIN`**: Proportional gains used in the control algorithm to adjust motor speeds based on distance and heading errors.
    - **`WHEEL_RADIUS`**: The radius of the robot's wheels, essential for converting rotational speeds to linear velocities.
    - **`AXLE_LENGTH`**: The distance between the two wheels, crucial for calculating rotational movements based on differential wheel speeds.
    - **`TURN_ANGLE`**: A threshold angle (π radians or 180 degrees) used to determine when a significant turn is required.

- **Motor Control Gains**:
    - **`HEADING_CONTROL_GAIN`**: Determines how aggressively the robot corrects its heading based on the heading error.
    - **`DISTANCE_CONTROL_GAIN`**: Influences how the robot adjusts its speed based on the distance to the current waypoint.

- **Occupancy Grid Dimensions**:
    - **`MAP_WIDTH` & `MAP_HEIGHT`**: Define the size of the occupancy grid map in pixels, representing the robot's environment for obstacle detection and navigation.

---

## Initialization

The `__init__` method is pivotal as it sets up the robot's hardware components, initializes sensors and motors, prepares mapping structures, and defines the trajectory waypoints. Let's dissect each part:

### Robot and Time Step Initialization

```python
def __init__(self):
    """
    Initializes the TrajectoryGenerator by setting up the robot, sensors, actuators,
    mapping structures, and predefined waypoints.
    """
    # Initialize Robot and Supervisor
    self.robot = Supervisor()

    # Time step
    self.timestep = int(self.robot.getBasicTimeStep())
```

- **Robot Instance**: Creates an instance of the `Supervisor` class, which extends the `Robot` class, providing enhanced control over the simulation environment.
  
- **Time Step**: Retrieves the simulation's basic time step, converting it to an integer. This time step dictates how often the robot's control loop executes.

### Sensor and Motor Initialization

```python
    # Initialize sensors and actuators
    self._init_motors()
    self._init_sensors()
    self._init_display()
    self._init_lidar()
```

- **`_init_motors()`**: Initializes and configures the left and right wheel motors for velocity control.
  
- **`_init_sensors()`**: Sets up the GPS and compass sensors, enabling them for data retrieval.
  
- **`_init_display()`**: Initializes the display device and retrieves the marker used for visualizing the current waypoint.
  
- **`_init_lidar()`**: Initializes the LIDAR sensor, enabling it and its point cloud data for environmental scanning.

### Display and Marker Initialization

```python
def _init_display(self):
    """
    Initializes the display device and the marker used for visualizing waypoints.
    It sets the initial color to white, clears the display, and then sets it back to red
    for drawing the trajectory and map points.
    """
    self.display = self.robot.getDevice('display')
    
    # Retrieve the 'marker' object from the simulation environment
    self.marker = self.robot.getFromDef("marker").getField("translation")
```

- **Display Device**: Retrieves the display device named `'display'` from the robot's hardware, used for visualizing the occupancy map and trajectory.

- **Marker Object**: Accesses a simulation object named `'marker'`, which serves as a visual indicator of the current waypoint. It manipulates the `'translation'` field to position the marker in the simulation environment.

### LIDAR Initialization

```python
def _init_lidar(self):
    """
    Initializes the Lidar sensor, enabling it and its point cloud data with the defined timestep.
    """
    self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
    self.lidar.enable(self.timestep)
    self.lidar.enablePointCloud()
```

- **LIDAR Device**: Retrieves the LIDAR sensor named `'Hokuyo URG-04LX-UG01'` from the robot's hardware.

- **Enabling LIDAR**: Activates the LIDAR sensor with the defined time step to start receiving range data.

- **Point Cloud**: Enables the generation of point cloud data, which is essential for mapping the robot's environment and detecting obstacles.

### Mapping Structures Initialization

```python
    # Initialize mapping structures
    self.map = np.zeros((self.MAP_WIDTH, self.MAP_HEIGHT))
    self.trajectory_map = np.zeros((self.MAP_WIDTH, self.MAP_HEIGHT))
    self.kernel = np.ones((20, 20))
```

- **Occupancy Map (`self.map`)**: A 2D NumPy array representing the environment. Each cell indicates whether an obstacle is present (`1`) or free space (`0`).

- **Trajectory Map (`self.trajectory_map`)**: Another 2D NumPy array tracking the robot's path through the environment.

- **Convolution Kernel (`self.kernel`)**: A 20x20 matrix of ones used for convolution operations to process the occupancy map, enhancing obstacle detection and mapping accuracy.

### Waypoints Initialization

```python
    # Define waypoints as a list of (x, y) tuples
    self.WP = [
        (+0.641, -2.460), (+0.224, -3.072), (-0.690, -3.074), (-1.690, -2.841), (-1.703, -2.302),  #  0 -  4
        (-1.702, -1.243), (-1.542, +0.422), (-0.382, +0.503), (+0.272, +0.503), (+0.383, +0.183),  #  5 -  9
        (+0.733, -0.093), (+0.701, -0.600), (+0.732, -0.094), (+0.684, +0.152), (+0.100, +0.501),  # 10 - 14
        (-0.682, +0.502), (-1.542, +0.424), (-1.762, -0.323), (-1.690, -1.242), (-1.803, -2.303),  # 15 - 19
        (-1.683, -2.842), (-0.693, -3.072), (+0.223, -3.073), (+0.223, -3.072), (+0.643, -2.463),  # 20 - 24
        (+0.632, -2.132), (+0.552, -2.213), (+0.714, -0.791), (+0.714, -0.792), (+0.711, +0.413)   # 25 - 29
    ]
    self.index = 0  # Current waypoint index
    self.robot_stop = False  # Flag to indicate if the robot should stop
```

- **Waypoints (`self.WP`)**: A predefined list of `(x, y)` coordinates representing the path the robot should follow. These waypoints define the trajectory through which the robot navigates.

- **Waypoint Index (`self.index`)**: Tracks the current target waypoint in the list. Initialized to `0`, pointing to the first waypoint.

- **Robot Stop Flag (`self.robot_stop`)**: A boolean flag indicating whether the robot should halt its movement. Initially set to `False`.

### Angle Array Initialization

```python
    # Angles for Lidar readings
    self.angles = np.linspace(2.094395, -2.094395, 667)  # From ~120 degrees to -120 degrees
```

- **Angles Array (`self.angles`)**: Generates an array of 667 angles ranging from approximately +120 degrees to -120 degrees (in radians). These angles correspond to the directional measurements taken by the LIDAR sensor, mapping each range reading to a specific angle relative to the robot's heading.

### Initial Movement

```python
    # Start moving forward slightly to avoid immediate start line detection
    self.set_motor_velocity(self.MAX_SPEED, self.MAX_SPEED)
    self.robot.step(1000)  # Move for 1 second
```

- **Purpose**: Prevents the robot from immediately detecting the start line by moving it slightly forward.

- **Operation**:
    - **Motor Control**: Sets both motors to `MAX_SPEED`, propelling the robot forward.
    - **Simulation Step**: Advances the simulation by 1000 milliseconds (1 second) using `self.robot.step(1000)`. This duration allows the robot to move forward without triggering immediate waypoint detection.

**Note**: The use of `self.robot.step(1000)` might be specific to the simulation environment's API, advancing the simulation by a fixed duration rather than one time step. In typical control loops, `step()` is called with the time step duration to advance the simulation incrementally.

---

## Motor Control Methods

Motor control is essential for directing the robot's movement. The `TrajectoryGenerator` class provides methods to initialize and set motor velocities effectively.

### Initializing Motors

```python
def _init_motors(self):
    """
    Initializes the left and right wheel motors, setting their positions to infinity
    to allow velocity control.
    """
    self.left_motor = self.robot.getDevice('wheel_left_joint')
    self.right_motor = self.robot.getDevice('wheel_right_joint')
    self.left_motor.setPosition(float('inf'))
    self.right_motor.setPosition(float('inf'))
```

- **Purpose**: Configures the left and right wheel motors for velocity control.

- **Operation**:
    - **Motor Retrieval**: Accesses the motor devices named `'wheel_left_joint'` and `'wheel_right_joint'`.
    - **Infinite Position**: Sets the position of both motors to infinity (`float('inf')`), enabling continuous rotation. This configuration allows the motors to run indefinitely based on velocity commands rather than position commands.

### Setting Motor Velocities

```python
def set_motor_velocity(self, left_velocity, right_velocity):
    """
    Set the velocities for both wheel motors.
    
    Args:
        left_velocity (float): Desired velocity for left wheel in rad/s
        right_velocity (float): Desired velocity for right wheel in rad/s
        
    Note:
        Velocities are automatically clamped to MAX_SPEED in calculate_wheel_speeds
    """
    self.left_motor.setVelocity(left_velocity)
    self.right_motor.setVelocity(right_velocity)
```

- **Purpose**: Provides a method to set the velocities of both motors simultaneously.

- **Operation**:
    - **Velocity Control**: Applies the specified velocities (`left_velocity`, `right_velocity`) to the left and right motors, respectively, dictating the robot's movement direction and speed.

**Note**: The method assumes that velocities passed to it are already within the permissible range, as clamping is handled in the `calculate_motor_speeds` method to prevent exceeding `MAX_SPEED`.

---

## Sensor Reading Methods

Accurate sensor data is crucial for effective navigation and obstacle avoidance. The `TrajectoryGenerator` class initializes and retrieves data from GPS and compass sensors.

### Initializing Sensors

```python
def _init_sensors(self):
    """
    Initializes the GPS and Compass sensors, enabling them with the defined timestep.
    """
    self.gps = self.robot.getDevice('gps')
    self.gps.enable(self.timestep)

    self.compass = self.robot.getDevice('compass')
    self.compass.enable(self.timestep)
```

- **GPS Sensor**:
    - **Retrieval**: Accesses the GPS sensor named `'gps'`.
    - **Enabling**: Activates the GPS sensor with the defined time step, allowing the robot to retrieve its current position.

- **Compass Sensor**:
    - **Retrieval**: Accesses the compass sensor named `'compass'`.
    - **Enabling**: Activates the compass sensor with the defined time step, enabling the robot to determine its current orientation.

### Retrieving Robot Pose

```python
def get_robot_pose(self):
    """
    Get the current pose (position and orientation) of the robot.
    
    Uses GPS for position and compass for orientation measurement.
    
    Returns:
        tuple: (x, y, theta) where:
            x (float): X-coordinate in world frame
            y (float): Y-coordinate in world frame
            theta (float): Heading angle in radians [-π, π]
    """
    x = self.gps.getValues()[0]
    y = self.gps.getValues()[1]
    theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
    return x, y, theta
```

- **Purpose**: Retrieves the robot's current position and orientation.

- **Operation**:
    - **Position Retrieval**: Fetches the X and Y coordinates from the GPS sensor using `self.gps.getValues()`, which returns an array of position values.
    - **Orientation Calculation**: Determines the robot's heading angle (`theta`) using the compass sensor data. It calculates the arctangent of the first two components (`x` and `y`) of the compass reading to obtain the orientation in radians, normalized between `-π` and `π`.

- **Return**: Provides a tuple `(x, y, theta)` representing the robot's current position and orientation.

---

## Odometry and Positioning

Odometry is the process of estimating a robot's position and orientation based on sensor data, particularly wheel encoders and inertial measurements. In this script, odometry calculations are intertwined with sensor readings to maintain an accurate estimate of the robot's pose.

### World to Map Coordinate Transformation

```python
def world_to_map(self, xw, yw):
    """
    Converts world coordinates (xw, yw) to map grid coordinates (px, py).

    Args:
        xw (float): X-coordinate in the world frame.
        yw (float): Y-coordinate in the world frame.

    Returns:
        tuple: (px, py) map grid coordinates, clamped within map boundaries.
    """
    px = int(52 * xw + 124.8)
    py = int(-52 * yw + 93.834)
    # Clamp to map boundaries to prevent indexing errors
    px = np.clip(px, 0, self.MAP_WIDTH - 1)
    py = np.clip(py, 0, self.MAP_HEIGHT - 1)
    return px, py
```

- **Purpose**: Transforms the robot's real-world coordinates into corresponding grid positions on the occupancy map.

- **Operation**:
    - **Scaling and Translation**:
        - **X-axis (`px`)**: Scales the world X-coordinate by 52 and translates it by 124.8 to fit the map's pixel dimensions.
        - **Y-axis (`py`)**: Scales the world Y-coordinate by -52 (inverting the axis) and translates it by 93.834 to align with the map's pixel dimensions.
    - **Clamping**: Ensures that the resulting map coordinates do not exceed the boundaries of the occupancy grid, preventing indexing errors.
    
- **Return**: Provides a tuple `(px, py)` representing the corresponding grid coordinates on the occupancy map.

**Note**: The scaling factors and translations (`52`, `124.8`, `-52`, `93.834`) are likely calibrated based on the simulation environment's scale and origin.

### Calculating Control Errors

```python
def calculate_control_errors(self, x, y, theta):
    """
    Calculate distance and heading errors to the current target waypoint.
    
    Args:
        x (float): Current X-coordinate of the robot
        y (float): Current Y-coordinate of the robot
        theta (float): Current heading angle in radians
        
    Returns:
        tuple: (rho, alpha) where:
            rho (float): Distance to target waypoint in meters
            alpha (float): Heading error in radians [-π, π]
    """
    # Distance to waypoint
    rho = np.sqrt((x - self.WP[self.index][0])**2 + (y - self.WP[self.index][1])**2)
    
    # Desired heading angle
    absolute_error = np.arctan2(self.WP[self.index][1] - y, self.WP[self.index][0] - x)
    
    # Heading error
    alpha = absolute_error - theta
    
    # Normalize alpha to [-pi, pi]
    if alpha > np.pi:
        alpha = alpha - 2 * np.pi
    elif alpha < -np.pi:
        alpha = alpha + 2 * np.pi
        
    return rho, alpha
```

- **Purpose**: Determines how far the robot is from its current target waypoint (`rho`) and the difference in orientation (`alpha`) it needs to correct to face the waypoint directly.

- **Operation**:
    - **Distance Error (`rho`)**:
        - Calculates the Euclidean distance between the robot's current position `(x, y)` and the target waypoint's position `self.WP[self.index]`.
    - **Heading Error (`alpha`)**:
        - Computes the desired heading angle (`absolute_error`) towards the waypoint using `np.arctan2`, which returns the angle between the positive X-axis and the point `(WP_x - x, WP_y - y)`.
        - Subtracts the robot's current orientation (`theta`) from the desired heading angle to obtain the heading error.
        - Normalizes `alpha` to ensure it remains within the range `[-π, π]` radians for consistency in control.

- **Return**: Provides a tuple `(rho, alpha)` representing the distance and heading errors, respectively.

### Calculating Motor Speeds

```python
def calculate_motor_speeds(self, alpha, rho):
    """
    Calculates the velocities for the left and right motors based on the heading (alpha)
    and distance (rho) errors using a proportional control strategy.

    Args:
        alpha (float): Heading error in radians.
        rho (float): Distance error in meters.

    Returns:
        tuple: (left_speed, right_speed) velocities for the left and right motors.
    """
    if abs(alpha) > np.pi / 4:
        # Significant turn required; reduce speed to enhance turning accuracy
        left_speed = -alpha * self.HEADING_CONTROL_GAIN / 2 + rho * self.DISTANCE_CONTROL_GAIN / 8
        right_speed = alpha * self.HEADING_CONTROL_GAIN / 2 + rho * self.DISTANCE_CONTROL_GAIN / 8
    else:
        # Regular movement towards the waypoint
        left_speed = -alpha * self.HEADING_CONTROL_GAIN + rho * self.DISTANCE_CONTROL_GAIN
        right_speed = alpha * self.HEADING_CONTROL_GAIN + rho * self.DISTANCE_CONTROL_GAIN

    # Ensure motor speeds do not exceed their maximum limits
    left_speed = np.clip(left_speed, -self.MOTOR_MAX_SPEED, self.MOTOR_MAX_SPEED)
    right_speed = np.clip(right_speed, -self.MOTOR_MAX_SPEED, self.MOTOR_MAX_SPEED)

    return left_speed, right_speed
```

- **Purpose**: Determines the appropriate velocities for the left and right motors to correct the robot's heading and reduce its distance to the target waypoint.

- **Operation**:
    - **Proportional Control**:
        - **Heading Correction**: Multiplies the heading error (`alpha`) by `HEADING_CONTROL_GAIN` to compute the necessary adjustment in motor speeds.
        - **Distance Correction**: Multiplies the distance error (`rho`) by `DISTANCE_CONTROL_GAIN` to adjust the robot's forward speed.
    - **Significant Turn Handling**:
        - If the absolute heading error (`abs(alpha)`) exceeds π/4 radians (~45 degrees), the robot requires a significant turn. To enhance turning accuracy, the motor speeds are adjusted with reduced influence from distance correction.
    - **Motor Speed Clamping**:
        - Ensures that neither motor speed exceeds `MOTOR_MAX_SPEED`, preventing mechanical strain and ensuring safe operation.

- **Return**: Provides a tuple `(left_speed, right_speed)` representing the calculated velocities for the left and right motors, respectively.

**Functionality**: This method implements a simple proportional controller that adjusts motor speeds based on real-time sensor data to steer the robot towards its target waypoint efficiently.

---

## Waypoint Management

Managing waypoints is crucial for guiding the robot along the desired trajectory. The `TrajectoryGenerator` class provides methods to update the current waypoint and handle the robot's stop condition upon reaching the final waypoint.

### Updating Waypoints

```python
def update_waypoints(self, rho):
    """
    Updates the current waypoint index if the robot is within a certain distance (rho)
    of the target waypoint. If all waypoints are reached, it stops the robot and processes
    the final occupancy map.

    Args:
        rho (float): Current distance error to the target waypoint.
    """
    if rho < 0.5:
        if self.index >= len(self.WP) - 1:  # Check if the last waypoint is reached
            print(" = Final waypoint reached")
            self.stop_robot()
            self.show_convolved_map()
            return

        self.index += 1
        print(f" > Waypoint index: {self.index}, position: {self.WP[self.index]}")

        if self.index >= len(self.WP):
            print(f" = Waypoint reached, index: {self.index}")
            self.stop_robot()
            self.show_convolved_map()
```

- **Purpose**: Advances the robot to the next waypoint once it is sufficiently close (`rho < 0.5` meters) to the current target. If all waypoints are completed, it stops the robot and processes the final occupancy map.

- **Operation**:
    - **Waypoint Proximity Check**: If the robot is within `0.5` meters of the current waypoint (`rho < 0.5`), it considers the waypoint as reached.
    - **Final Waypoint Handling**:
        - If the current waypoint is the last in the list (`self.index >= len(self.WP) - 1`), it prints a message, stops the robot, and processes the occupancy map.
    - **Advancing to Next Waypoint**:
        - Increments the waypoint index (`self.index += 1`) to target the next waypoint.
        - Prints the updated waypoint index and its coordinates.
    - **Boundary Check**:
        - If the waypoint index exceeds the list length (`self.index >= len(self.WP)`), it confirms waypoint completion, stops the robot, and processes the occupancy map.

- **Functionality**: This method ensures that the robot progresses through the predefined waypoints sequentially and halts upon completing the final waypoint, triggering necessary post-navigation procedures.

### Stopping the Robot

```python
def stop_robot(self):
    """
    Stops the robot by setting both motor velocities to zero.
    """
    self.left_motor.setVelocity(0)
    self.right_motor.setVelocity(0)
```

- **Purpose**: Halts the robot's movement by setting both wheel motor velocities to zero.

- **Operation**:
    - **Motor Control**: Applies a velocity of `0` to both the left and right motors, effectively stopping the robot.

- **Functionality**: Ensures that the robot ceases all movement when it has completed its trajectory or upon encountering specific conditions requiring a stop.

---

## Occupancy Mapping and Visualization

Occupancy mapping involves creating a grid-based representation of the environment, indicating the presence or absence of obstacles. The `TrajectoryGenerator` class integrates LIDAR data to update the occupancy map and provides visualization capabilities for both the map and the robot's trajectory.

### Processing LIDAR Data

```python
def process_lidar(self, xw, yw, theta):
    """
    Processes Lidar data to update the occupancy map. It transforms Lidar points from the
    robot's frame to the world frame, updates the occupancy grid, and visualizes the points
    on the display.

    Args:
        xw (float): Current X position of the robot in the world frame.
        yw (float): Current Y position of the robot in the world frame.
        theta (float): Current orientation of the robot in radians.
    """
    # Check if the display device is initialized
    if self.display is None:
        print("Warning: Display device not initialized properly")
        return

    # Retrieve and preprocess Lidar range data
    ranges = np.array(self.lidar.getRangeImage())
    ranges[ranges == np.inf] = 100  # Replace infinite readings with a large value

    # Exclude the first and last 80 readings to avoid edge artifacts
    valid_ranges = ranges[80:-80]
    valid_angles = self.angles[80:-80]

    # Transform Lidar readings to the robot's coordinate system
    X_r = np.vstack((
        valid_ranges * np.cos(valid_angles),
        valid_ranges * np.sin(valid_angles),
        np.ones(len(valid_angles))
    ))

    # Define the transformation matrix from robot to world coordinates
    w_T_r = np.array([
        [np.cos(theta), -np.sin(theta), xw],
        [np.sin(theta),  np.cos(theta), yw],
        [            0,              0,  1]
    ])

    # Apply the transformation to obtain world coordinates of Lidar points
    D = w_T_r @ X_r

    # Update the occupancy map with the transformed Lidar points
    for point in D.T:
        px, py = self.world_to_map(point[0], point[1])
        self.map[px, py] = min(self.map[px, py] + 0.01, 1.0)  # Increment occupancy value
        color_byte = int(self.map[px, py] * 255)
        color = (color_byte << 16) | (color_byte << 8) | color_byte  # Grayscale color
        self.display.setColor(color)
        self.display.drawPixel(px, py)
```

- **Purpose**: Converts raw LIDAR data into occupancy grid updates, mapping detected obstacles onto the environment grid.

- **Operation**:
    - **Display Check**: Ensures that the display device is initialized before attempting to draw.
    - **LIDAR Data Retrieval**: Fetches the range data from the LIDAR sensor using `self.lidar.getRangeImage()`. Converts the data to a NumPy array for efficient processing.
    - **Infinite Values Handling**: Replaces any infinite readings (indicating no detection) with a large value (`100`) to standardize the data.
    - **Data Slicing**: Excludes the first and last 80 readings to eliminate edge artifacts or irrelevant data points.
    - **Coordinate Transformation**:
        - **Robot Frame**: Converts polar LIDAR data (ranges and angles) into Cartesian coordinates `(X_r, Y_r)` in the robot's frame.
        - **World Frame**: Applies a transformation matrix (`w_T_r`) based on the robot's current position and orientation to map these points into the world frame coordinates `(D)`.
    - **Occupancy Map Update**:
        - Iterates through each transformed LIDAR point.
        - Transforms world coordinates to map grid coordinates using `world_to_map()`.
        - Increments the occupancy value at the corresponding grid cell, capping it at `1.0` to denote maximum occupancy.
        - Converts the occupancy value to a grayscale color for visualization.
        - Draws the occupancy point on the display.

- **Functionality**: This method effectively maps the robot's immediate environment, allowing it to recognize obstacles and free spaces, which is essential for navigation and obstacle avoidance.

### Drawing the Trajectory

```python
def draw_trajectory(self, xw, yw):
    """
    Draws the robot's current position on the trajectory map and visualizes it in red on the display.

    Args:
        xw (float): Current X position of the robot in the world frame.
        yw (float): Current Y position of the robot in the world frame.
    """
    # Check if the display device is initialized
    if self.display is None:
        print("Warning: Display device not initialized properly")
        return

    # Convert world coordinates to map grid coordinates
    px, py = self.world_to_map(xw, yw)
    self.trajectory_map[px, py] = 1.0  # Mark the trajectory point
    self.display.setColor(0xFF0000)    # Set color to red for the trajectory
    self.display.drawPixel(px, py)     # Draw the trajectory point
```

- **Purpose**: Visualizes the robot's path by marking its current position on the trajectory map and displaying it on the simulation's display device.

- **Operation**:
    - **Display Check**: Ensures that the display device is initialized before attempting to draw.
    - **Coordinate Transformation**: Converts the robot's current world coordinates `(xw, yw)` to map grid coordinates `(px, py)` using `world_to_map()`.
    - **Trajectory Marking**: Updates the trajectory map by setting the corresponding grid cell to `1.0`, indicating the robot has traversed that point.
    - **Visualization**:
        - Sets the drawing color to red (`0xFF0000`) to differentiate the trajectory from the occupancy map.
        - Draws a pixel at the transformed grid coordinates to represent the robot's current position.

- **Functionality**: This method provides a visual trail of the robot's movement, aiding in monitoring and debugging the navigation process.

### Convolving and Displaying the Map

```python
def show_convolved_map(self):
    """
    Processes the occupancy map using convolution with a predefined kernel to identify
    significant obstacles or features. It then displays the convolved map using matplotlib.
    """
    cmap = signal.convolve2d(self.map, self.kernel, mode='same')
    cspace = cmap > 0.9  # Thresholding to create a binary occupancy grid
    plt.imshow(cspace, cmap='gray')
    plt.title("Convolved Map")
    plt.show()
```

- **Purpose**: Enhances the occupancy map by applying convolution to identify significant obstacles and visualizes the processed map.

- **Operation**:
    - **Convolution**: Applies a 2D convolution operation between the occupancy map (`self.map`) and the kernel (`self.kernel`). The `mode='same'` parameter ensures the output map has the same dimensions as the input.
    - **Thresholding**: Creates a binary occupancy grid (`cspace`) by thresholding the convolved map, marking cells with values greater than `0.9` as occupied (`True`) and others as free (`False`).
    - **Visualization**:
        - Uses `matplotlib.pyplot` to display the convolved and thresholded occupancy map.
        - Sets the colormap to grayscale (`'gray'`) for clarity.
        - Adds a title "Convolved Map" for context.
        - Calls `plt.show()` to render the visualization.

- **Functionality**: This method provides a processed view of the environment, highlighting significant obstacles and features, which is essential for informed navigation and decision-making.

---

## Main Control Loop

The `run` method encapsulates the robot's primary operational logic, executing continuously to process sensor data, calculate control commands, update waypoints, and manage occupancy mapping and visualization.

```python
def run(self):
    """
    Executes the main control loop for the trajectory generator. It continuously reads sensor data,
    calculates control commands, updates waypoints, processes Lidar data, and visualizes the trajectory
    until the simulation is terminated or all waypoints are reached.
    """
    while self.robot.step(self.timestep) != -1:
        # Read sensor values
        gps_values = self.gps.getValues()
        compass_values = self.compass.getValues()
        xw, yw = gps_values[0], gps_values[1]
        theta = np.arctan2(compass_values[0], compass_values[1])

        # Update the marker's position to the current waypoint
        self.marker.setSFVec3f([*self.WP[self.index], 0])

        # Calculate distance (rho) and heading (alpha) errors
        rho = np.hypot(xw - self.WP[self.index][0], yw - self.WP[self.index][1])
        alpha = np.arctan2(self.WP[self.index][1] - yw, self.WP[self.index][0] - xw) - theta
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]

        # Compute motor speeds based on errors
        left_speed, right_speed = self.calculate_motor_speeds(alpha, rho)

        # Update waypoint index if the current waypoint is reached
        self.update_waypoints(rho)

        # Set the computed velocities to the motors
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

        # Process Lidar data to update the occupancy map
        self.process_lidar(xw, yw, theta)

        # Draw the robot's trajectory on the display
        self.draw_trajectory(xw, yw)
```

### Step-by-Step Breakdown

1. **Simulation Step**:
    ```python
    while self.robot.step(self.timestep) != -1:
    ```
    - **Purpose**: Advances the simulation by one time step (`self.timestep`). The loop continues as long as `step()` does not return `-1`, which would indicate the simulation has ended.
  
2. **Sensor Data Retrieval**:
    ```python
    gps_values = self.gps.getValues()
    compass_values = self.compass.getValues()
    xw, yw = gps_values[0], gps_values[1]
    theta = np.arctan2(compass_values[0], compass_values[1])
    ```
    - **GPS Data**: Retrieves the robot's current position `(xw, yw)` in the world frame from the GPS sensor.
    - **Compass Data**: Retrieves the robot's orientation vector from the compass sensor. Calculates the heading angle (`theta`) using the arctangent of the compass readings, yielding the orientation in radians.
  
3. **Marker Position Update**:
    ```python
    self.marker.setSFVec3f([*self.WP[self.index], 0])
    ```
    - **Purpose**: Updates the simulation marker to the current target waypoint's position.
    - **Operation**: Sets the translation field of the marker object to the coordinates of the current waypoint (`self.WP[self.index]`) with a Z-coordinate of `0` (assuming 2D movement).

4. **Control Errors Calculation**:
    ```python
    rho = np.hypot(xw - self.WP[self.index][0], yw - self.WP[self.index][1])
    alpha = np.arctan2(self.WP[self.index][1] - yw, self.WP[self.index][0] - xw) - theta
    alpha = (alpha + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
    ```
    - **Distance Error (`rho`)**: Calculates the Euclidean distance between the robot's current position `(xw, yw)` and the target waypoint's position.
    - **Heading Error (`alpha`)**: Computes the difference between the desired heading towards the waypoint and the robot's current orientation (`theta`). Normalizes `alpha` to ensure it remains within `[-π, π]` radians for consistency.

5. **Motor Speeds Calculation and Setting**:
    ```python
    left_speed, right_speed = self.calculate_motor_speeds(alpha, rho)
    ```
    - **Purpose**: Determines the appropriate velocities for the left and right motors based on the calculated control errors.
    - **Operation**: Calls the `calculate_motor_speeds` method with `alpha` and `rho` to obtain `left_speed` and `right_speed`.

    ```python
    self.left_motor.setVelocity(left_speed)
    self.right_motor.setVelocity(right_speed)
    ```
    - **Purpose**: Applies the calculated velocities to the motors, steering the robot towards the target waypoint.
    - **Operation**: Sets the left and right motor velocities using the previously determined `left_speed` and `right_speed`.

6. **Waypoint Reached Check and Update**:
    ```python
    self.update_waypoints(rho)
    ```
    - **Purpose**: Checks if the robot is close enough to the current waypoint to consider it reached. If so, advances to the next waypoint or stops if all waypoints are completed.
    - **Operation**: Calls the `update_waypoints` method with the current distance error `rho`.

7. **Occupancy Mapping Update**:
    ```python
    self.process_lidar(xw, yw, theta)
    ```
    - **Purpose**: Updates the occupancy map based on the latest LIDAR data, mapping detected obstacles onto the grid.
    - **Operation**: Calls the `process_lidar` method with the robot's current position and orientation.

8. **Trajectory Visualization**:
    ```python
    self.draw_trajectory(xw, yw)
    ```
    - **Purpose**: Marks the robot's current position on the trajectory map and visualizes it on the display device.
    - **Operation**: Calls the `draw_trajectory` method with the robot's current position.

**Functionality**: This main control loop ensures that the robot continuously processes sensor data, adjusts its movement, updates its position, manages waypoints, and maintains an accurate occupancy map of its environment, all while visualizing its trajectory for monitoring and debugging purposes.

---

## Execution Entry Point

```python
if __name__ == "__main__":
    """
    Entry point for the script. Instantiates the TrajectoryGenerator and starts the control loop.
    """
    traj_gen = TrajectoryGenerator()
    traj_gen.run()
```

- **Purpose**: Allows the script to be executed as a standalone program.

- **Operation**:
    - **Instantiation**: Creates an instance of the `TrajectoryGenerator` class, triggering the initialization process.
    - **Run Method**: Calls the `run` method to start the main control loop, initiating the robot's autonomous navigation.

**Functionality**: This block ensures that when the script is run directly, the robot begins its trajectory following behavior without requiring external triggers or inputs.

---

## Conclusion

The `trajectory_generator.py` script offers a sophisticated and integrated approach to autonomous robot navigation. By combining sensor data processing, proportional control algorithms, odometry, occupancy mapping, and visualization, it enables a differential drive robot to follow a predefined trajectory with precision and adaptability. Here's a summary of its key components and functionalities:

- **Sensor Integration**:
    - **GPS and Compass**: Provide accurate localization and orientation data, essential for determining the robot's current pose.
    - **LIDAR**: Offers environmental perception, allowing the robot to detect obstacles and update the occupancy map dynamically.

- **Control Algorithms**:
    - **Proportional Control**: Adjusts motor speeds based on heading and distance errors to steer the robot towards waypoints effectively.
    - **Waypoint Management**: Ensures sequential navigation through predefined waypoints, handling final waypoint completion gracefully.

- **Occupancy Mapping**:
    - **Grid-Based Representation**: Maintains an occupancy grid map that reflects the robot's surroundings, crucial for obstacle avoidance and spatial awareness.
    - **Convolution and Thresholding**: Enhances map accuracy by processing occupancy data to identify significant obstacles.

- **Visualization**:
    - **Display Device**: Provides real-time visualization of the occupancy map and robot trajectory, aiding in monitoring and debugging.
    - **Matplotlib Integration**: Offers enhanced graphical representations of the convolved occupancy map for post-processing analysis.

### Potential Enhancements

1. **Advanced Control Strategies**:
    - **PID Controllers**: Implementing Proportional-Integral-Derivative (PID) controllers can provide more refined control over motor speeds, reducing overshooting and oscillations.
    - **Dynamic Gain Adjustment**: Adjusting control gains (`HEADING_CONTROL_GAIN`, `DISTANCE_CONTROL_GAIN`) based on real-time conditions can enhance adaptability.

2. **Enhanced Odometry**:
    - **Sensor Fusion**: Combining data from multiple sensors (e.g., wheel encoders, IMUs) can improve position and orientation estimates, reducing cumulative errors.
    - **Kalman Filtering**: Applying Kalman filters can provide more accurate and robust pose estimations by accounting for sensor noise and uncertainties.

3. **Obstacle Avoidance**:
    - **Reactive Behaviors**: Incorporating obstacle avoidance algorithms can enable the robot to navigate around unforeseen obstacles dynamically.
    - **Path Planning**: Implementing path planning algorithms (e.g., A*, RRT) can allow the robot to compute optimal paths in real-time, adapting to changes in the environment.

4. **Trajectory Optimization**:
    - **Smooth Trajectories**: Generating smoother trajectories by interpolating waypoints can enhance navigation efficiency and reduce mechanical strain.
    - **Adaptive Waypoints**: Allowing waypoints to be adjusted based on environmental feedback can make the navigation system more flexible.

5. **Visualization Enhancements**:
    - **Interactive Displays**: Developing interactive visualization tools can provide deeper insights into the robot's decision-making processes and environmental interactions.
    - **3D Mapping**: Extending occupancy mapping to three dimensions can enable navigation in more complex environments.

6. **Energy Efficiency**:
    - **Motor Power Management**: Optimizing motor usage based on movement requirements can enhance the robot's energy efficiency and operational lifespan.
    - **Idle Strategies**: Implementing strategies to reduce power consumption when the robot is idle or moving slowly can contribute to overall energy savings.

### Final Thoughts

The `trajectory_generator.py` script serves as a robust foundation for autonomous robot navigation, seamlessly integrating multiple sensor inputs, control algorithms, and mapping techniques to achieve precise trajectory following. By understanding its components and operational logic, developers and robotics enthusiasts can appreciate the complexities involved in autonomous navigation and leverage this knowledge to build more advanced and capable robotic systems. Whether for educational purposes, research, or practical applications, this script exemplifies the synergy between sensor data processing, control theory, and computational techniques in the realm of robotics.

