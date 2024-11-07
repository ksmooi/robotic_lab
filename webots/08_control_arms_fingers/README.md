# Understanding `tiago_control.py`: A Comprehensive Step-by-Step Guide

Autonomous robotics has revolutionized industries ranging from manufacturing to healthcare. Central to this revolution is the ability of robots to perform complex tasks with precision and reliability. The `tiago_control.py` script exemplifies this by orchestrating the movements and actions of the TIAGo robot—a versatile, humanoid robot designed for various applications. This article delves deep into the `tiago_control.py` script, providing a detailed, step-by-step explanation of its structure, components, and operational logic to help you grasp how it manages both mobility and manipulator controls, processes sensor data, and navigates through predefined waypoints.

## Table of Contents

1. [Overview](#overview)
2. [Imports and Dependencies](#imports-and-dependencies)
3. [Class Definition and Constants](#class-definition-and-constants)
4. [Initialization](#initialization)
    - [Robot and Time Step Initialization](#robot-and-time-step-initialization)
    - [Motor and Sensor Initialization](#motor-and-sensor-initialization)
    - [Display and Mapping Structures Initialization](#display-and-mapping-structures-initialization)
    - [Waypoints and Angle Array Initialization](#waypoints-and-angle-array-initialization)
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
10. [Manipulator Control](#manipulator-control)
    - [Grabbing an Object](#grabbing-an-object)
    - [Dropping an Object](#dropping-an-object)
11. [Utility Methods](#utility-methods)
    - [Waiting for Milliseconds](#waiting-for-milliseconds)
    - [Printing Debug Messages](#printing-debug-messages)
12. [Main Control Loop](#main-control-loop)
13. [Execution Entry Point](#execution-entry-point)
14. [Conclusion](#conclusion)

---

## Overview

The `TiagoController` class within `tiago_control.py` is designed to manage both the mobility and manipulator functions of the TIAGo robot. It integrates sensor data processing, motor control algorithms, occupancy mapping, and visual feedback to enable the robot to navigate through a series of predefined waypoints, manipulate objects, and interact with its environment seamlessly. The controller ensures precise movement, accurate object handling, and effective environmental mapping, making TIAGo a reliable autonomous system.

---

## Imports and Dependencies

```python
from controller import Supervisor
import numpy as np
from scipy import signal
from datetime import datetime
from matplotlib import pyplot as plt
```

- **`controller` Module**: Specific to robotics simulation environments like Webots, this module provides classes to interact with robot hardware components.
    - **`Supervisor`**: An advanced class extending the `Robot` class, offering higher-level control over the simulation, such as accessing and modifying node fields.
  
- **`numpy`**: A fundamental package for numerical computations in Python, used here for array operations and mathematical functions essential for processing sensor data and control algorithms.
  
- **`scipy.signal`**: Utilized for signal processing tasks, such as convolution, which plays a role in occupancy mapping and environmental analysis.
  
- **`datetime`**: Provides classes for manipulating dates and times, used here for timestamping debug messages.
  
- **`matplotlib.pyplot`**: A plotting library used for visualizing the occupancy map and robot trajectory, aiding in debugging and analysis.

---

## Class Definition and Constants

```python
class TiagoController:
    """
    TiagoController is a controller class for managing the movements and actions
    of the TIAGo robot. It handles both mobility (wheel movements) and manipulator
    (arm and gripper) controls, as well as sensor data processing and mapping.

    Attributes:
        robot (Supervisor): The robot supervisor instance.
        timestep (int): The simulation timestep in milliseconds.
        MAX_SPEED (float): Maximum speed for the robot's wheels.
        KP (float): Proportional gain for control algorithms.
        KA (float): Additional gain for control algorithms.
        WHEEL_RADIUS (float): Radius of the robot's wheels in meters.
        AXLE_LENGTH (float): Distance between the two wheels in meters.
        robot_init_joints (dict): Initial positions for the robot's arm and other joints.
        MAX_GRIP_FORCE (float): Maximum allowable force for the gripper.
        motor_handles (dict): Handles to the motor devices.
        map (ndarray): Occupancy map of the environment.
        trajectory_map (ndarray): Map of the robot's trajectory.
        kernel (ndarray): Kernel used for map processing.
        angles (ndarray): Array of angles corresponding to LIDAR measurements.
    """

    def __init__(self):
        ...
```

The `TiagoController` class encapsulates all functionalities required for controlling the TIAGo robot's mobility and manipulator. It defines several constants and attributes that are pivotal for motion calculations, control logic, and environmental mapping.

### Key Constants

- **Robot Physical Constants**:
    - **`MAX_SPEED`**: Defines the upper limit for the robot's wheel speeds, set to `6.28` radians per second (~1 revolution per second).
    - **`KP`**: Proportional gain used in control algorithms to adjust motor speeds based on errors.
    - **`KA`**: An additional gain factor for more nuanced control.
    - **`WHEEL_RADIUS`**: The radius of the robot's wheels in meters, essential for converting rotational speeds to linear velocities.
    - **`AXLE_LENGTH`**: The distance between the two wheels in meters, crucial for calculating rotational movements based on differential wheel speeds.
  
- **Manipulator Control Constants**:
    - **`robot_init_joints`**: A dictionary specifying the initial positions for the robot's arm joints and other movable parts.
    - **`MAX_GRIP_FORCE`**: Sets the maximum allowable force the gripper can exert, preventing damage to objects or the robot.
  
- **Mapping and Visualization Constants**:
    - **`map`**: A 2D NumPy array representing the occupancy grid of the environment.
    - **`trajectory_map`**: Tracks the robot's path through the environment.
    - **`kernel`**: A convolution kernel used for processing the occupancy map to identify significant obstacles or features.
    - **`angles`**: An array of angles corresponding to LIDAR measurements, used for mapping sensor data to spatial coordinates.

---

## Initialization

The `__init__` method sets up the robot's hardware components, initializes sensors and motors, prepares mapping structures, and defines the trajectory waypoints. This comprehensive initialization ensures that the robot is ready for autonomous operation.

### Robot and Time Step Initialization

```python
def __init__(self):
    """
    Initializes the TiagoController by setting up the robot, initializing
    devices (motors, sensors, etc.), and preparing mapping and trajectory
    data structures.
    """
    # Initialize robot
    self.robot = Supervisor()
    self.timestep = int(self.robot.getBasicTimeStep())
            
    # Movement constants
    self.MAX_SPEED = 6.28
    self.KP = 1.0
    self.KA = 5.0
    self.WHEEL_RADIUS = 0.10
    self.AXLE_LENGTH = 0.455
```

- **Robot Instance**: Creates an instance of the `Supervisor` class, enabling higher-level control over the simulation environment.
  
- **Time Step**: Retrieves the simulation's basic time step (`self.robot.getBasicTimeStep()`), converting it to an integer for use in control loops and sensor updates.
  
- **Movement Constants**: Initializes constants related to movement, including maximum speed, proportional gains (`KP`, `KA`), wheel radius, and axle length, which are essential for motion and control calculations.

### Motor and Sensor Initialization

```python
    # Arm joint positions dictionary
    self.robot_init_joints = {
        'torso_lift_joint': 0.35,            # lower back, along z-axis
        'arm_1_joint': 0.71,                 # shoulder, along z-axis (min: 0.07 rad)
        'arm_2_joint': 1.02,                 # shoulder, along x-axis (max: 1.02 rad)
        'arm_3_joint': -2.815,               # shoulder, along y-axis
        'arm_4_joint': 1.011,                # elbow, along x-axis
        'arm_5_joint': 0,                    # elbow, along z-axis
        'arm_6_joint': 0,                    # wrist, along y-axis
        'arm_7_joint': 0,                    # wrist, along x-axis
        'gripper_left_finger_joint': 0,      # left finger
        'gripper_right_finger_joint': 0,     # right finger
        'head_1_joint': 0,                   # head left-right, along axis-z
        'head_2_joint': 0                    # head up-down, along axis-y
    }
                    
    self.MAX_GRIP_FORCE = 10.0  # Maximum allowed gripping force
    
    # Initialize devices
    self.init_devices()
```

- **Arm Joint Positions**: Defines a dictionary (`robot_init_joints`) specifying the initial positions for various joints of the robot's arm and other movable parts. This setup is crucial for ensuring the manipulator starts from a known, safe position.
  
- **Maximum Gripper Force**: Sets `MAX_GRIP_FORCE` to `10.0`, preventing the gripper from exerting excessive force that could damage objects or the robot itself.
  
- **Device Initialization**: Calls the `init_devices()` method to initialize and configure all necessary devices, including motors and sensors.

### Display and Mapping Structures Initialization

```python
    # Initialize mapping structures
    self.map = np.zeros((250, 300))
    self.trajectory_map = np.zeros((250, 300))
    self.kernel = np.ones((20, 20))
    self.angles = np.linspace(2.094395, -2.094395, 667)
```

- **Occupancy Map (`self.map`)**: Initializes a 2D NumPy array of zeros with dimensions `250x300` pixels, representing the robot's environment. Each cell will be updated based on LIDAR data to indicate the presence or absence of obstacles.
  
- **Trajectory Map (`self.trajectory_map`)**: Another 2D NumPy array of zeros with the same dimensions, used to track and visualize the robot's movement path.
  
- **Convolution Kernel (`self.kernel`)**: Creates a `20x20` matrix of ones, used later for convolution operations to process the occupancy map, enhancing obstacle detection accuracy.
  
- **LIDAR Angles (`self.angles`)**: Generates an array of `667` angles ranging from approximately +120 degrees (`2.094395` radians) to -120 degrees (`-2.094395` radians). These angles correspond to the directional measurements taken by the LIDAR sensor, mapping each range reading to a specific angle relative to the robot's heading.

### Waypoints and Angle Array Initialization

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

- **Waypoints (`self.WP`)**: A predefined list of `(x, y)` tuples representing the sequence of waypoints the robot should navigate through. These waypoints define the trajectory, ensuring the robot follows a specific path within its environment.
  
- **Waypoint Index (`self.index`)**: Tracks the current target waypoint in the list. Initialized to `0`, pointing to the first waypoint.
  
- **Robot Stop Flag (`self.robot_stop`)**: A boolean flag indicating whether the robot should halt its movement. Initially set to `False`.

---

## Motor Control Methods

Effective motor control is essential for precise robot movements. The `TiagoController` class provides methods to initialize motors and set their velocities based on control algorithms.

### Initializing Motors

```python
def init_devices(self):
    """
    Initializes all necessary devices for the robot, including motors for
    the arm and wheels, sensors like GPS, compass, LIDAR, and display.
    Sets up encoders and enables force feedback for gripper joints.
    """
    # Initialize arm motors
    self.motor_handles = {}
    for joint_name in self.robot_init_joints.keys():
        self.motor_handles[joint_name] = self.robot.getDevice(joint_name)
        
    # Initialize mobility devices
    self.leftMotor = self.robot.getDevice('wheel_left_joint')
    self.rightMotor = self.robot.getDevice('wheel_right_joint')
    self.leftMotor.setPosition(float('inf'))
    self.rightMotor.setPosition(float('inf'))
    
    # Initialize sensors
    self.gps = self.robot.getDevice('gps')
    self.compass = self.robot.getDevice('compass')
    self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
    self.display = self.robot.getDevice('display')
    
    # Enable sensors
    self.gps.enable(self.timestep)
    self.compass.enable(self.timestep)
    self.lidar.enable(self.timestep)
    self.lidar.enablePointCloud()
           
    # Define marker
    self.marker = self.robot.getFromDef("marker").getField("translation")
    
    # Set up display color
    self.display.setColor(0xFF0000)

    # Initialize encoders
    self.encoders = {}  # Reset encoders dictionary
    for joint_name in self.robot_init_joints.keys():
        try:
            if 'gripper' in joint_name:
                # Special case for gripper sensors
                sensor_name = joint_name.replace('gripper_', 'gripper_').replace('finger_joint', 'sensor_finger_joint')
            else:
                sensor_name = joint_name + '_sensor'
            
            encoder = self.robot.getDevice(sensor_name)
            if encoder is not None:
                encoder.enable(self.timestep)
                self.encoders[joint_name] = encoder
            else:
                print(f"Warning: Encoder '{sensor_name}' not found")
        except Exception as e:
            print(f"Error initializing {joint_name}: {e}")

    # Enable force feedback for gripper joints
    self.motor_handles['gripper_left_finger_joint'].enableForceFeedback(self.timestep)
    self.motor_handles['gripper_right_finger_joint'].enableForceFeedback(self.timestep)
```

- **Arm Motors Initialization**:
    - **`self.motor_handles`**: A dictionary that stores handles to each joint's motor device. Iterates through the `robot_init_joints` dictionary to retrieve and store each joint's motor.
  
- **Mobility Devices Initialization**:
    - **Left and Right Motors**: Retrieves the motor devices named `'wheel_left_joint'` and `'wheel_right_joint'`.
    - **Infinite Positioning**: Sets both motors' positions to infinity (`float('inf')`), enabling continuous rotation based on velocity commands rather than fixed positions.
  
- **Sensor Initialization**:
    - **GPS, Compass, LIDAR, Display**: Retrieves the respective sensor and display devices from the robot's hardware.
    - **Enabling Sensors**: Activates the GPS, compass, and LIDAR sensors with the defined time step. Also enables point cloud data generation for the LIDAR.
  
- **Marker Initialization**:
    - **Marker Object**: Accesses the `'marker'` object in the simulation environment, manipulating its `'translation'` field to indicate the current waypoint.
  
- **Display Configuration**:
    - **Color Setting**: Sets the display color to red (`0xFF0000`) for visualizing occupancy and trajectory data.
  
- **Encoders Initialization**:
    - **Encoders Dictionary**: Initializes an empty dictionary (`self.encoders`) to store handles to encoder devices.
    - **Encoder Retrieval**: Iterates through each joint in `robot_init_joints`, attempting to retrieve and enable corresponding encoder devices. Handles special cases for gripper joints by adjusting sensor names.
    - **Error Handling**: Prints warnings if encoders are not found and errors if initialization fails.
  
- **Force Feedback for Grippers**:
    - **Enabling Force Feedback**: Activates force feedback on the gripper joints (`'gripper_left_finger_joint'` and `'gripper_right_finger_joint'`), allowing the robot to sense and respond to forces exerted by the gripper.

### Setting Motor Velocities

```python
def set_motor_velocity(self, left_velocity, right_velocity):
    """
    Set the velocities for both wheel motors.
    
    Args:
        left_velocity (float): Desired velocity for left wheel in rad/s
        right_velocity (float): Desired velocity for right wheel in rad/s
        
    Note:
        Velocities are automatically clamped to MAX_SPEED in calculate_motor_speeds
    """
    self.left_motor.setVelocity(left_velocity)
    self.right_motor.setVelocity(right_velocity)
```

- **Purpose**: Provides a method to set the velocities of both wheel motors simultaneously, facilitating synchronized movement.
  
- **Operation**:
    - **Motor Velocity Control**: Applies the specified `left_velocity` and `right_velocity` to the left and right motors, respectively, dictating the robot's movement direction and speed.
  
- **Note**: The method assumes that the velocities passed to it are already within the permissible range, as clamping is handled in the `calculate_motor_speeds` method to prevent exceeding `MAX_SPEED`.

---

## Sensor Reading Methods

Accurate sensor data is crucial for effective navigation and manipulation. The `TiagoController` class initializes and retrieves data from various sensors to maintain situational awareness and control precision.

### Initializing Sensors

Already covered in the `init_devices()` method, where GPS, compass, and LIDAR sensors are initialized and enabled.

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

- **Purpose**: Retrieves the robot's current position and orientation, providing essential data for navigation and control algorithms.
  
- **Operation**:
    - **Position Retrieval**: Uses the GPS sensor to obtain the X and Y coordinates of the robot in the world frame.
    - **Orientation Calculation**: Utilizes the compass sensor to determine the robot's heading angle (`theta`). The `np.arctan2` function calculates the angle between the positive X-axis and the vector defined by the compass readings, normalizing it within `[-π, π]` radians.
  
- **Return**: Provides a tuple `(x, y, theta)` representing the robot's current position and orientation.

---

## Odometry and Positioning

Odometry involves estimating the robot's position and orientation based on sensor data, particularly wheel encoders and inertial measurements. In this script, odometry calculations are integrated with sensor readings to maintain an accurate estimate of the robot's pose.

### World to Map Coordinate Transformation

```python
def world2map(self, xw, yw):
    """
    Converts world coordinates (x, y) to map grid coordinates (px, py).

    Args:
        xw (float): X-coordinate in the world frame.
        yw (float): Y-coordinate in the world frame.

    Returns:
        list: [px, py] coordinates on the map grid, clamped within map boundaries.
    """
    px = int(52 * xw + 124.8)
    py = int(-52 * yw + 93.834)
    px = max(min(px, 249), 0)
    py = max(min(py, 299), 0)
    return [px, py]
```

- **Purpose**: Transforms the robot's real-world coordinates into corresponding grid positions on the occupancy map.
  
- **Operation**:
    - **Scaling and Translation**:
        - **X-axis (`px`)**: Scales the world X-coordinate by `52` and translates it by `124.8` to fit the map's pixel dimensions.
        - **Y-axis (`py`)**: Scales the world Y-coordinate by `-52` (inverting the axis) and translates it by `93.834` to align with the map's pixel dimensions.
    - **Clamping**: Ensures that the resulting map coordinates do not exceed the boundaries of the occupancy grid (`250x300` pixels), preventing indexing errors.
    
- **Return**: Provides a list `[px, py]` representing the corresponding grid coordinates on the occupancy map.

**Note**: The scaling factors and translations (`52`, `124.8`, `-52`, `93.834`) are calibrated based on the simulation environment's scale and origin, ensuring accurate mapping from world coordinates to grid positions.

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
    - **Desired Heading Angle (`absolute_error`)**:
        - Uses `np.arctan2` to calculate the angle between the positive X-axis and the line connecting the robot's current position to the target waypoint.
    - **Heading Error (`alpha`)**:
        - Computes the difference between the desired heading angle (`absolute_error`) and the robot's current orientation (`theta`).
        - Normalizes `alpha` to ensure it remains within the range `[-π, π]` radians for consistency in control algorithms.
  
- **Return**: Provides a tuple `(rho, alpha)` representing the distance and heading errors, respectively.

### Calculating Motor Speeds

```python
def calculate_motor_speeds(self, alpha, rho):
    """
    Calculates the left and right motor speeds based on angular and distance errors.

    Args:
        alpha (float): Angle error in radians.
        rho (float): Distance error in meters.

    Returns:
        tuple: (left_speed, right_speed) in radians/second, clamped to MAX_SPEED.
    """
    p1, p2 = 2.8, 2.6
    if abs(alpha) > np.pi/4:
        leftSpeed = -alpha * p1 / 2 + rho * p2 / 8
        rightSpeed = alpha * p1 / 2 + rho * p2 / 8
    else:
        leftSpeed = -alpha * p1 + rho * p2
        rightSpeed = alpha * p1 + rho * p2
        
    # Clamp speeds to maximum allowed
    leftSpeed = max(min(leftSpeed, self.MAX_SPEED), -self.MAX_SPEED)
    rightSpeed = max(min(rightSpeed, self.MAX_SPEED), -self.MAX_SPEED)
    return leftSpeed, rightSpeed
```

- **Purpose**: Determines the appropriate velocities for the left and right motors to correct the robot's heading and reduce its distance to the target waypoint.
  
- **Operation**:
    - **Proportional Control Factors**:
        - **`p1`**: Proportional gain for heading correction.
        - **`p2`**: Proportional gain for distance correction.
    - **Significant Turn Handling**:
        - If the absolute heading error (`abs(alpha)`) exceeds π/4 radians (~45 degrees), indicating a significant turn is required, the motor speeds are adjusted with reduced influence from distance correction to enhance turning accuracy.
        - **Motor Speed Calculations**:
            - **For Significant Turns**:
                - **Left Motor**: `-alpha * p1 / 2 + rho * p2 / 8`
                - **Right Motor**: `alpha * p1 / 2 + rho * p2 / 8`
            - **For Regular Movement**:
                - **Left Motor**: `-alpha * p1 + rho * p2`
                - **Right Motor**: `alpha * p1 + rho * p2`
    - **Clamping**:
        - Ensures that neither motor speed exceeds `MAX_SPEED` in either direction, preventing mechanical strain and ensuring safe operation.
  
- **Return**: Provides a tuple `(leftSpeed, rightSpeed)` representing the calculated velocities for the left and right motors, respectively.

**Functionality**: This method implements a simple proportional controller that adjusts motor speeds based on real-time sensor data to steer the robot towards its target waypoint efficiently.

---

## Waypoint Management

Managing waypoints is crucial for guiding the robot along the desired trajectory. The `TiagoController` class provides methods to update the current waypoint and handle the robot's stop condition upon reaching the final waypoint.

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
        - If the current waypoint is the last in the list (`self.index >= len(self.WP) - 1`), it prints a message indicating the final waypoint has been reached, stops the robot, and processes the occupancy map by calling `show_convolved_map()`.
    - **Advancing to Next Waypoint**:
        - Increments the waypoint index (`self.index += 1`) to target the next waypoint.
        - Prints the updated waypoint index and its coordinates for monitoring.
    - **Boundary Check**:
        - If the waypoint index exceeds the list length (`self.index >= len(self.WP)`), it confirms waypoint completion, stops the robot, and processes the occupancy map.
  
- **Functionality**: This method ensures that the robot progresses through the predefined waypoints sequentially and halts upon completing the final waypoint, triggering necessary post-navigation procedures.

### Stopping the Robot

```python
def stop_robot(self):
    """
    Stops the robot by setting both motor velocities to zero.
    """
    self.leftMotor.setVelocity(0)
    self.rightMotor.setVelocity(0)
```

- **Purpose**: Halts the robot's movement by setting both wheel motor velocities to zero.
  
- **Operation**:
    - **Motor Velocity Control**: Applies a velocity of `0` to both the left and right motors, effectively stopping the robot.
  
- **Functionality**: Ensures that the robot ceases all movement when it has completed its trajectory or upon encountering specific conditions requiring a stop.

---

## Occupancy Mapping and Visualization

Occupancy mapping involves creating a grid-based representation of the environment, indicating the presence or absence of obstacles. The `TiagoController` class integrates LIDAR data to update the occupancy map and provides visualization capabilities for both the map and the robot's trajectory.

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
    X_r = np.array([
        (valid_ranges * np.cos(valid_angles)),
        (valid_ranges * np.sin(valid_angles)),
        np.ones(len(valid_angles))
    ])

    # Transformation matrix from robot frame to world frame
    w_T_r = np.array([
        [np.cos(theta), -np.sin(theta), xw],
        [np.sin(theta), np.cos(theta), yw],
        [0, 0, 1]
    ])
    D = w_T_r @ X_r

    # Update the occupancy map based on transformed points
    for i in range(D.shape[1]):
        px, py = self.world2map(D[0, i], D[1, i])
        self.map[px, py] = min(self.map[px, py] + 0.01, 1.0)
        color_byte = int(self.map[px, py] * 255)
        color = (color_byte << 16) | (color_byte << 8) | color_byte
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
        - Transforms world coordinates to map grid coordinates using `world2map()`.
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
    px, py = self.world2map(xw, yw)
    self.trajectory_map[px, py] = 1.0  # Mark the trajectory point
    self.display.setColor(0xFF0000)    # Set color to red for the trajectory
    for i in range(250):
        for j in range(300):
            if self.trajectory_map[i, j] > 0:
                self.display.drawPixel(i, j)
```

- **Purpose**: Visualizes the robot's path by marking its current position on the trajectory map and displaying it on the simulation's display device.
  
- **Operation**:
    - **Display Check**: Ensures that the display device is initialized before attempting to draw.
    - **Coordinate Transformation**: Converts the robot's current world coordinates `(xw, yw)` to map grid coordinates `(px, py)` using `world2map()`.
    - **Trajectory Marking**: Updates the trajectory map by setting the corresponding grid cell to `1.0`, indicating the robot has traversed that point.
    - **Visualization**:
        - Sets the drawing color to red (`0xFF0000`) to differentiate the trajectory from the occupancy map.
        - Iterates through the entire trajectory map, drawing red pixels at positions where the trajectory has been marked.
  
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

## Manipulator Control

The manipulator (arm and gripper) is integral to the robot's ability to interact with objects. The `TiagoController` class provides methods to control the arm's joints and manage object manipulation sequences like grabbing and dropping objects.

### Grabbing an Object

```python
def grab_object(self, action_id):
    """
    Executes a sequence of actions to grab an object using the robot's arm and gripper.

    Args:
        action_id (int): Identifier for the grab action sequence.

    Raises:
        RuntimeError: If any step in the grab sequence fails.

    Returns:
        bool: True if the object is successfully grabbed, otherwise raises an exception.
    """
    try:
        # Move arm to horizontal position
        if not self.control_arm(f"grab-{action_id}a", {
            'arm_2_joint': 0,
            'arm_4_joint': 0,
            'arm_5_joint': 1.571,
        }):
            raise RuntimeError("Failed to position arm horizontally")
            
        self.wait_for_milliseconds(300)
        
        # Close gripper to grasp the object
        if not self.control_arm(f"grab-{action_id}b", {
            'gripper_left_finger_joint': 0.04,
            'gripper_right_finger_joint': 0.04,
        }):
            raise RuntimeError("Failed to close gripper")
            
        self.wait_for_milliseconds(300)
        
        # Lift the arm with the object
        if not self.control_arm(f"grab-{action_id}c", {
            'arm_2_joint': 1.000,
            'arm_4_joint': -0.262,
            'arm_5_joint': 1.571,
        }):
            raise RuntimeError("Failed to lift arm")
            
        self.wait_for_milliseconds(300)
        return True
        
    except Exception as e:
        raise RuntimeError(f"Grab object sequence {action_id} failed: {str(e)}")
```

- **Purpose**: Automates the sequence of movements required to grab an object using the robot's arm and gripper.
  
- **Operation**:
    - **Sequence Steps**:
        1. **Positioning the Arm Horizontally**:
            - Calls `control_arm` with specific joint positions to move the arm into a horizontal position.
            - If unsuccessful, raises a `RuntimeError`.
            - Waits for `300` milliseconds to ensure the arm has time to reach the desired position.
        2. **Closing the Gripper**:
            - Calls `control_arm` to close the gripper joints, attempting to grasp the object.
            - If unsuccessful, raises a `RuntimeError`.
            - Waits for `300` milliseconds to allow the gripper to secure the object.
        3. **Lifting the Arm**:
            - Calls `control_arm` to lift the arm, carrying the object.
            - If unsuccessful, raises a `RuntimeError`.
            - Waits for `300` milliseconds to ensure the arm has lifted the object.
    - **Error Handling**: Catches any exceptions during the grab sequence, stopping the robot and providing detailed error messages.
  
- **Return**: Returns `True` if the object is successfully grabbed; otherwise, raises an exception detailing the failure.

### Dropping an Object

```python
def drop_object(self, action_id):
    """
    Executes a sequence of actions to drop an object using the robot's arm and gripper.

    Args:
        action_id (int): Identifier for the drop action sequence.

    Raises:
        RuntimeError: If any step in the drop sequence fails.

    Returns:
        bool: True if the object is successfully dropped, otherwise raises an exception.
    """
    try:
        # Lower torso and position arm for dropping
        if not self.control_arm(f"drop-{action_id}d", {
            'torso_lift_joint': 0.10,
            'arm_2_joint': 0,
            'arm_6_joint': -0.524,
        }):
            raise RuntimeError("Failed to position arm for drop")
            
        self.wait_for_milliseconds(300)
        
        # Open gripper to release the object
        if not self.control_arm(f"drop-{action_id}b", {
            'gripper_left_finger_joint': 0.07,
            'gripper_right_finger_joint': 0.07,
        }):
            raise RuntimeError("Failed to open gripper")
            
        self.wait_for_milliseconds(300)
        
        # Return arm and torso to upright position
        if not self.control_arm(f"drop-{action_id}c", {
            'torso_lift_joint': 0.28,
            'arm_2_joint': 1.000,
            'arm_6_joint': 0,
        }):
            raise RuntimeError("Failed to return to upright position")
            
        self.wait_for_milliseconds(300)
        return True
        
    except Exception as e:
        raise RuntimeError(f"Drop object sequence {action_id} failed: {str(e)}")
```

- **Purpose**: Automates the sequence of movements required to drop an object using the robot's arm and gripper.
  
- **Operation**:
    - **Sequence Steps**:
        1. **Lowering the Torso and Positioning the Arm for Drop**:
            - Calls `control_arm` with specific joint positions to lower the torso and position the arm appropriately for dropping the object.
            - If unsuccessful, raises a `RuntimeError`.
            - Waits for `300` milliseconds to ensure the arm has reached the desired position.
        2. **Opening the Gripper**:
            - Calls `control_arm` to open the gripper joints, releasing the object.
            - If unsuccessful, raises a `RuntimeError`.
            - Waits for `300` milliseconds to allow the gripper to release the object.
        3. **Returning Arm and Torso to Upright Position**:
            - Calls `control_arm` to return the arm and torso to their initial upright positions.
            - If unsuccessful, raises a `RuntimeError`.
            - Waits for `300` milliseconds to ensure the robot has returned to the upright position.
    - **Error Handling**: Catches any exceptions during the drop sequence, stopping the robot and providing detailed error messages.
  
- **Return**: Returns `True` if the object is successfully dropped; otherwise, raises an exception detailing the failure.

---

## Utility Methods

Utility methods provide essential support functions such as pausing execution and logging debug messages, enhancing the controller's robustness and debuggability.

### Waiting for Milliseconds

```python
def wait_for_milliseconds(self, milliseconds):
    """
    Pauses the controller for a specified duration in milliseconds.

    Args:
        milliseconds (int): Duration to wait in milliseconds.
    """
    loop = int(milliseconds / self.timestep)
    for _ in range(loop):
        self.robot.step(self.timestep)
```

- **Purpose**: Introduces a controlled pause in the controller's execution, allowing the robot's actuators to reach target positions before proceeding to the next action.
  
- **Operation**:
    - **Loop Calculation**: Determines the number of simulation steps required to achieve the desired pause duration based on the simulation's time step.
    - **Execution**: Iteratively calls `self.robot.step(self.timestep)` to advance the simulation, effectively creating a pause.
  
- **Usage**: Utilized within action sequences like grabbing and dropping objects to ensure sufficient time for each step to complete.

### Printing Debug Messages

```python
def print_debug(self, message):
    """
    Prints a debug message with a timestamp.

    Args:
        message (str): The debug message to print.
    """
    # Get the current date and time formatted to milliseconds
    formatted_time = datetime.now().strftime('%Y%m%d %H%M%S.%f')[:-3]
    print(f"{formatted_time}\t{message}")
```

- **Purpose**: Provides a standardized way to log debug messages with precise timestamps, aiding in monitoring and troubleshooting the controller's operations.
  
- **Operation**:
    - **Timestamp Generation**: Retrieves the current date and time, formatting it to include milliseconds for high-resolution logging.
    - **Message Formatting**: Concatenates the timestamp with the provided debug message, separated by a tab for readability.
    - **Output**: Prints the formatted message to the console.
  
- **Usage**: Employed throughout the controller to log significant events, status updates, and error messages.

---

## Main Control Loop

The `run` method encapsulates the robot's primary operational logic, executing continuously to process sensor data, calculate control commands, update waypoints, manage occupancy mapping, and handle object manipulation sequences.

```python
def run(self):
    """
    Executes the main control loop for the robot, performing initial setup
    and executing multiple rounds of object grabbing and dropping sequences.

    Raises:
        RuntimeError: If any step in the mission fails.

    Returns:
        bool: True if the mission completes successfully, otherwise False.
    """
    try:
        # -----------------------------------------------------------------------
        # Initial Round Setup
        # -----------------------------------------------------------------------
        if not self.control_arm('arm-initial', {
            'torso_lift_joint': 0.28,
            'arm_1_joint': 1.571,
            'arm_2_joint': 1.000,
            'arm_4_joint': -0.262,
            'arm_5_joint': 1.571,
            'gripper_left_finger_joint': 0.07,
            'gripper_right_finger_joint': 0.07,
        }):
            raise RuntimeError("Failed to set initial arm position")

        if not self.control_wheel('navigation-initial', [(0.960, 0.500)]):
            raise RuntimeError("Failed initial navigation")

        # make sure the objects from the kitchen are not in the same position
        finetune_waypoints = {
            1: [
                ( 0.000, -0.700),
                ( 0.960,  0.300)
            ],
            2: [
                ( 0.000, -0.900),
                ( 0.960,  0.100),
            ],
            3: [
                ( 0.000, -1.100),
                (-1.727,  0.968),
            ],
        }
        
        # -----------------------------------------------------------------------
        # Execute Rounds 1-3: Grab, Navigate, Drop, and Return
        # -----------------------------------------------------------------------
        for round_num in range(1, 4):
            # Grab object
            if not self.grab_object(round_num):
                raise RuntimeError(f"Failed to grab object in round {round_num}")
            
            # Navigate to drop position
            if not self.control_wheel(f'navigation-{round_num}a', [
                (1.025, -0.326),
                finetune_waypoints[round_num][0]
            ]):
                raise RuntimeError(f"Failed navigation {round_num}a")
            
            # Drop object
            if not self.drop_object(round_num):
                raise RuntimeError(f"Failed to drop object in round {round_num}")
            
            # Return path
            if not self.control_wheel(f'navigation-{round_num}b', [
                ( 0.550, -2.500),  # right
                ( 0.550, -3.100),  # right-bottom
                (-0.682, -3.500),  # bottom
                (-1.550, -2.800),  # left-bottom
                (-1.550, -1.500),  # left
                (-1.550, -0.000),  # left-top
                finetune_waypoints[round_num][1]
            ]):
                raise RuntimeError(f"Failed navigation {round_num}b")

        self.print_debug("Mission completed successfully")
        return True

    except Exception as e:
        self.print_debug(f"Mission failed: {str(e)}")
        # Emergency stop in case of failure
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
        return False
```

### Step-by-Step Breakdown

1. **Initial Round Setup**:
    - **Arm Positioning**:
        - Calls `control_arm` with the `'arm-initial'` action ID and a set of joint positions to set the arm to its initial state.
        - If unsuccessful, raises a `RuntimeError` indicating failure to set the initial arm position.
    - **Initial Navigation**:
        - Calls `control_wheel` with the `'navigation-initial'` action ID and a list containing the first waypoint `(0.960, 0.500)`.
        - If unsuccessful, raises a `RuntimeError` indicating failure in initial navigation.
  
2. **Fine-Tune Waypoints**:
    - **`finetune_waypoints`**: Defines additional waypoints for rounds 1 to 3 to ensure objects from the kitchen are not in the same position. These waypoints provide more precise navigation paths during the mission.

3. **Executing Rounds 1-3**:
    - **Loop**: Iterates through `round_num` values from `1` to `3`, executing a sequence of actions in each round.
    - **Actions per Round**:
        1. **Grabbing an Object**:
            - Calls `grab_object` with the current `round_num`.
            - If unsuccessful, raises a `RuntimeError` indicating failure to grab the object in the current round.
        2. **Navigating to Drop Position**:
            - Calls `control_wheel` with a specific action ID (`'navigation-{round_num}a'`) and a list of waypoints to navigate towards the drop position.
            - If unsuccessful, raises a `RuntimeError` indicating failure in navigation.
        3. **Dropping the Object**:
            - Calls `drop_object` with the current `round_num`.
            - If unsuccessful, raises a `RuntimeError` indicating failure to drop the object in the current round.
        4. **Returning Along the Path**:
            - Calls `control_wheel` with a specific action ID (`'navigation-{round_num}b'`) and a list of waypoints to navigate back along the return path.
            - If unsuccessful, raises a `RuntimeError` indicating failure in return navigation.
  
4. **Mission Completion**:
    - After successfully completing all rounds, logs a debug message indicating the mission's successful completion and returns `True`.
  
5. **Error Handling**:
    - Catches any exceptions during the mission execution, logs a debug message with the error, stops the robot by setting motor velocities to zero, and returns `False` to indicate mission failure.

**Functionality**: This main control loop ensures that the robot performs its mission in a structured manner—initializing its arm and navigation, executing multiple rounds of object manipulation and navigation, and handling any errors that may arise during the mission.

---

## Execution Entry Point

```python
if __name__ == "__main__":
    """
    Entry point of the script. Creates an instance of TiagoController and
    starts the control loop.
    """
    # Create and run the controller
    controller = TiagoController()
    controller.run()
```

- **Purpose**: Ensures that the `TiagoController` class is instantiated and the main control loop is initiated when the script is executed directly.
  
- **Operation**:
    - **Instantiation**: Creates an instance of the `TiagoController` class, triggering the initialization process outlined in the `__init__` method.
    - **Run Method**: Calls the `run` method to start the main control loop, enabling the robot to begin its autonomous mission.
  
- **Functionality**: This block allows the script to be executed as a standalone program, initiating all necessary components for autonomous navigation and manipulation.

---

## Conclusion

The `tiago_control.py` script presents a sophisticated framework for autonomous robot navigation and manipulation, integrating sensor data processing, motor control algorithms, occupancy mapping, and visual feedback to enable the TIAGo robot to perform complex missions with precision and reliability. Here's a summary of its key components and functionalities:

- **Sensor Integration**:
    - **GPS and Compass**: Provide accurate localization and orientation data, essential for determining the robot's current pose.
    - **LIDAR**: Offers environmental perception, allowing the robot to detect obstacles and update the occupancy map dynamically.
  
- **Control Algorithms**:
    - **Proportional Control**: Adjusts motor speeds based on heading and distance errors to steer the robot towards waypoints effectively.
    - **Waypoint Management**: Ensures sequential navigation through predefined waypoints, handling final waypoint completion gracefully.
  
- **Occupancy Mapping**:
    - **Grid-Based Representation**: Maintains an occupancy grid map that reflects the robot's surroundings, crucial for obstacle avoidance and spatial awareness.
    - **Convolution and Thresholding**: Enhances map accuracy by processing occupancy data to identify significant obstacles.
  
- **Manipulator Control**:
    - **Arm and Gripper Control**: Automates sequences to grab and drop objects, enabling the robot to interact with its environment.
  
- **Visualization**:
    - **Display Device**: Provides real-time visualization of the occupancy map and robot trajectory, aiding in monitoring and debugging.
    - **Matplotlib Integration**: Offers enhanced graphical representations of the convolved occupancy map for post-processing analysis.
  
- **Utility Functions**:
    - **Pausing and Logging**: Implements controlled pauses and timestamped debug messages to ensure synchronized operations and facilitate troubleshooting.
  
- **Error Handling**:
    - **Robust Exception Management**: Catches and handles exceptions gracefully, ensuring the robot can halt safely and provide informative error messages in case of failures.

### Potential Enhancements

1. **Advanced Control Strategies**:
    - **PID Controllers**: Implementing Proportional-Integral-Derivative (PID) controllers can provide more refined control over motor speeds, reducing overshooting and oscillations.
    - **Dynamic Gain Adjustment**: Adjusting control gains (`KP`, `KA`) based on real-time conditions can enhance adaptability.
  
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

By meticulously integrating various sensor inputs, control algorithms, and mapping techniques, the `tiago_control.py` script serves as an excellent foundation for building more sophisticated autonomous robotic systems. Understanding its components and logic not only aids in leveraging its capabilities but also inspires further innovations in robotic navigation, manipulation, and environmental interaction. Whether you're a robotics enthusiast, educator, or developer, dissecting and enhancing this script can provide valuable insights into the complexities and rewards of autonomous robot control.

