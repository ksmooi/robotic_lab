# Mastering Autonomous Robotics with Webots

Autonomous robotics is a field that seamlessly blends hardware, software, and intelligent algorithms to enable robots to perform complex tasks without human intervention. In this article, we delve into a Webots project comprising four pivotal Python scripts—`mapping.py`, `navigation.py`, `planning.py`, and `behavior_trees.py`. These scripts collectively empower a TIAGo robot (a versatile, humanoid robot) to navigate through an environment, map its surroundings, plan optimal paths, and execute tasks with precision. 

This comprehensive guide dissects each script, elucidating their roles, functionalities, and interconnections within the broader robotic system. Whether you're a seasoned robotics enthusiast or a newcomer eager to understand autonomous navigation, this article offers valuable insights into building intelligent robotic behaviors using Webots and Python.

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Script Breakdown](#script-breakdown)
    - [1. `mapping.py`](#1-mappingpy)
        - [World to Map Coordinate Transformation](#world-to-map-coordinate-transformation)
        - [Map to World Coordinate Transformation](#map-to-world-coordinate-transformation)
        - [Mapping Class](#mapping-class)
    - [2. `navigation.py`](#2-navigationpy)
        - [Navigation Class](#navigation-class)
    - [3. `planning.py`](#3-planningpy)
        - [Planning Class](#planning-class)
    - [4. `behavior_trees.py`](#4-behaviortreespy)
        - [Blackboard Class](#blackboard-class)
        - [DoesMapExist Class](#doesmapexist-class)
        - [Behavior Tree Creation](#behavior-tree-creation)
        - [Main Function](#main-function)
4. [Interconnection of Scripts](#interconnection-of-scripts)
5. [Execution Flow](#execution-flow)
6. [Potential Enhancements](#potential-enhancements)
7. [Conclusion](#conclusion)

---

## Overview

In the realm of autonomous robotics, the ability to perceive the environment, plan optimal paths, and execute tasks without human intervention is paramount. The provided scripts work in tandem to achieve these objectives:

- **`mapping.py`**: Handles environmental perception by creating an occupancy grid map using LIDAR data.
- **`navigation.py`**: Manages the robot's movement through predefined waypoints using sensor feedback.
- **`planning.py`**: Implements the A* algorithm to plan optimal paths from the robot's current position to a goal.
- **`behavior_trees.py`**: Orchestrates the behaviors using behavior trees, integrating mapping, navigation, and planning into a cohesive control system.

By leveraging libraries like `py_trees` for behavior trees, `numpy` and `scipy` for numerical computations, and `matplotlib` for visualization, these scripts form a robust foundation for autonomous navigation and task execution.

---

## Prerequisites

Before diving into the scripts, ensure you have the following set up:

1. **Webots**: A powerful open-source robot simulation environment. [Download Webots](https://cyberbotics.com/#download)
2. **Python**: Version 3.6 or higher.
3. **Required Python Libraries**:
    - `numpy`
    - `scipy`
    - `py_trees`
    - `matplotlib`

   You can install these using `pip`:

   ```bash
   pip install numpy scipy py_trees matplotlib
   ```

4. **TIAGo Robot Model**: Ensure that the TIAGo robot model is available within your Webots simulation environment.

---

## Script Breakdown

Let's dissect each script, understanding their components and functionalities.

### 1. `mapping.py`

This script is responsible for environmental perception, converting sensor data into a usable occupancy grid map. It defines functions for coordinate transformations and a `Mapping` class that integrates with behavior trees to handle mapping behaviors.

#### World to Map Coordinate Transformation

```python
def world2map(xw, yw):
    """Convert world coordinates to map pixel coordinates.
    
    Args:
        xw (float): X coordinate in world frame
        yw (float): Y coordinate in world frame
        
    Returns:
        list: [px, py] Pixel coordinates in map frame (0-199, 0-299)
    """
    # Improved boundary handling using np.clip
    px = np.clip(int((xw + 2.25) * 40), 0, 199)
    py = np.clip(int((yw - 2) * (-50)), 0, 299)
    return [px, py]
```

- **Purpose**: Transforms real-world coordinates `(xw, yw)` into corresponding pixel positions `(px, py)` on the occupancy grid map.
- **Mechanism**:
    - **Scaling and Translation**: Adjusts the world coordinates based on the map's scale and origin.
    - **Clipping**: Ensures that the resulting pixel coordinates stay within the map boundaries to prevent indexing errors.

#### Map to World Coordinate Transformation

```python
def map2world(px, py):
    """Convert map pixel coordinates to world coordinates.
    
    Args:
        px (int): X pixel coordinate in map frame
        py (int): Y pixel coordinate in map frame
        
    Returns:
        list: [xw, yw] Coordinates in world frame
    """
    xw = px/40 - 2.25
    yw = py/(-50) + 2
    return [xw, yw]
```

- **Purpose**: Converts map pixel positions back to world coordinates, facilitating tasks like path planning where waypoints are in world coordinates.
- **Mechanism**:
    - **Inverse Scaling and Translation**: Reverses the operations performed in `world2map` to retrieve original coordinates.

#### Mapping Class

```python
class Mapping(py_trees.behaviour.Behaviour):
    """Behavior tree node that handles robot mapping functionality.
    
    This class creates and updates an occupancy grid map using LIDAR readings.
    It converts LIDAR readings to world coordinates and maintains both a regular
    occupancy grid and a configuration space representation accounting for robot size.
    
    Args:
        name (str): Name of the behavior node
        blackboard (Blackboard): Shared blackboard for data access
    
    Attributes:
        map (ndarray): Occupancy grid map of size 200x300
        MAP_SIZE (tuple): Size of the map in pixels (200, 300)
        ROBOT_RADIUS (int): Robot radius in pixels for configuration space
        LIDAR_TRIM (int): Number of LIDAR readings to trim from each end
        OCCUPANCY_INCREMENT (float): Increment value for occupied cells
        MAX_OCCUPANCY (float): Maximum occupancy probability value
    """
    def __init__(self, name, blackboard):
        ...
```

- **Inheritance**: Extends `py_trees.behaviour.Behaviour`, integrating with behavior trees for modular and hierarchical behavior management.
- **Attributes**:
    - **`map`**: A 200x300 occupancy grid initialized to zeros, representing free space.
    - **`ROBOT_RADIUS`**: Defines the robot's size in pixels for configuration space mapping, crucial for path planning to account for the robot's physical dimensions.
    - **`LIDAR_TRIM`**: Number of LIDAR readings to exclude from each end, removing irrelevant or noisy data.
    - **`OCCUPANCY_INCREMENT`**: Increment value to update occupancy probabilities based on sensor readings.
    - **`MAX_OCCUPANCY`**: Caps occupancy probabilities to prevent overestimation.

#### Setup Method

```python
def setup(self):
    """Initialize robot sensors and devices.
    
    Sets up GPS, compass, LIDAR, and display devices with appropriate timesteps
    and enables required functionalities.
    """
    self.timestep = int(self.robot.getBasicTimeStep())
    self.gps = self.robot.getDevice('gps')
    self.gps.enable(self.timestep)
    self.compass = self.robot.getDevice('compass')
    self.compass.enable(self.timestep)
    self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
    self.lidar.enable(self.timestep)
    self.lidar.enablePointCloud()
    self.display = self.robot.getDevice('display')
```

- **Purpose**: Initializes and enables essential sensors and display devices.
- **Mechanism**:
    - **GPS and Compass**: Provide localization and orientation data.
    - **LIDAR**: Facilitates environmental perception.
    - **Display**: Used for visualizing the occupancy grid and robot trajectory.

#### Initialise Method

```python
def initialise(self):
    """Initialize the occupancy grid map and LIDAR parameters.
    
    Creates an empty occupancy grid and precomputes LIDAR angles for efficiency.
    The angles are stored after trimming specified number of readings from ends.
    """
    self.map = np.zeros(self.MAP_SIZE)
    # Precompute LIDAR angles once
    total_readings = 667
    angle_range = 4.19
    self.angles = np.linspace(angle_range/2, -angle_range/2, total_readings)
    self.angles = self.angles[self.LIDAR_TRIM:-self.LIDAR_TRIM]
```

- **Purpose**: Prepares the occupancy grid and calculates the relevant LIDAR angles, excluding trimmed readings to focus on meaningful data.
- **Mechanism**:
    - **Map Initialization**: Resets the occupancy grid to zero.
    - **Angle Calculation**: Generates a linearly spaced array of angles corresponding to LIDAR measurements, trimming the first and last `LIDAR_TRIM` readings to eliminate edge noise.

#### Update Method

```python
def update(self):
    """Update the occupancy grid map based on current LIDAR readings.
    
    Gets current robot pose from GPS and compass, transforms LIDAR readings
    to world coordinates, and updates the occupancy grid probabilities.
    Also visualizes the current map state on the robot's display.
    
    Returns:
        Status.RUNNING: Continuous mapping behavior
    """
    self.hasrun = True
    xw = self.gps.getValues()[0]
    yw = self.gps.getValues()[1]
    theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])

    px, py = world2map(xw, yw)
    self.display.setColor(0xFF0000)
    self.display.drawPixel(px, py)

    W_T_R = np.array([[np.cos(theta), -np.sin(theta), xw],
                      [np.sin(theta), np.cos(theta), yw],
                      [0, 0, 1]])

    # Read and preprocess range images
    ranges = np.array(self.lidar.getRangeImage())
    ranges = ranges[self.LIDAR_TRIM:-self.LIDAR_TRIM]
    ranges[ranges == np.inf] = 100
    
    # Vectorized point cloud transformation
    X_i = np.vstack([
        ranges * np.cos(self.angles),
        ranges * np.sin(self.angles),
        np.ones_like(ranges)
    ])
    world_points = W_T_R @ X_i

    # Update occupancy grid
    for point in world_points.T:
        px, py = world2map(point[0], point[1])
        self.map[px, py] = np.minimum(
            self.map[px, py] + self.OCCUPANCY_INCREMENT,
            self.MAX_OCCUPANCY
        )
        # Convert occupancy to grayscale color
        v = int(self.map[px, py] * 255)
        color = v * (256**2 + 256 + 1)
        self.display.setColor(int(color))
        self.display.drawPixel(px, py)

    return py_trees.common.Status.RUNNING
```

- **Purpose**: Continuously updates the occupancy grid based on real-time LIDAR data, reflecting changes in the environment.
- **Mechanism**:
    - **Pose Retrieval**: Obtains the robot's current position `(xw, yw)` and orientation `theta` using GPS and compass.
    - **Marker Visualization**: Marks the robot's current position on the display in red.
    - **Coordinate Transformation**:
        - **World to Robot Frame**: Uses the transformation matrix `W_T_R` to convert LIDAR points from the robot's frame to the world frame.
    - **Occupancy Grid Update**:
        - **Probability Increment**: Increases the occupancy probability for cells corresponding to detected obstacles.
        - **Visualization**: Updates the display with grayscale pixels representing occupancy levels.

#### Terminate Method

```python
def terminate(self, new_status):
    """Clean up and save the final map when behavior terminates.
    
    Args:
        new_status: New status the behavior is transitioning to
        
    Creates and saves the configuration space representation of the map
    accounting for robot size using convolution.
    """
    if self.hasrun:
        # Calculate configuration space using robot radius
        kernel = np.ones((2*self.ROBOT_RADIUS, 2*self.ROBOT_RADIUS))
        cspace = signal.convolve2d(self.map, kernel, mode='same')
        np.save('cspace', cspace)
        
        # Visualize the map
        # plt.figure(0)
        # plt.imshow(cspace)
        # plt.show()
```

- **Purpose**: Upon termination of the mapping behavior, it processes the occupancy grid to create a configuration space (C-space) that accounts for the robot's size, facilitating collision-free path planning.
- **Mechanism**:
    - **Convolution**: Applies a convolution with a kernel matching the robot's size to expand obstacle representations, ensuring planned paths maintain a safe distance from obstacles.
    - **Saving C-space**: Saves the processed C-space map to a file (`cspace.npy`) for use in path planning.
    - **Visualization**: (Commented out) Potentially visualizes the C-space using `matplotlib` for debugging and analysis.

### 2. `navigation.py`

This script manages the robot's movement through predefined waypoints using sensor feedback. It employs a proportional control strategy to adjust wheel velocities based on distance and heading errors.

#### Navigation Class

```python
class Navigation(py_trees.behaviour.Behaviour):
    """A behavior tree node that handles robot navigation through waypoints.
    
    This class implements waypoint-following behavior using GPS and compass sensors
    for position and heading feedback. It uses a simple proportional control law
    for both angular and linear velocity control.
    
    Attributes:
        MAX_VELOCITY (float): Maximum allowed motor velocity (6.28 rad/s)
        WAYPOINT_THRESHOLD (float): Distance threshold to consider waypoint reached (0.4m)
        ANGULAR_GAIN (float): Proportional gain for angular control (4.0)
        LINEAR_GAIN (float): Proportional gain for linear control (2.0)
    """
    # Add constants at class level
    MAX_VELOCITY = 6.28  # Maximum motor velocity
    WAYPOINT_THRESHOLD = 0.4  # Distance threshold to consider waypoint reached
    
    # Control gains
    ANGULAR_GAIN = 4  # P1 gain for angular control
    LINEAR_GAIN = 2   # P2 gain for linear control

    def __init__(self, name, blackboard):
        """Initialize the navigation behavior.
        
        Args:
            name (str): Name of the behavior node
            blackboard (Blackboard): Behavior tree blackboard for sharing data
        """
        super(Navigation, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard

    def setup(self):
        """Set up robot sensors and actuators.
        
        Initializes GPS, compass, wheel motors, and visualization marker.
        """
        self.timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)

        self.leftMotor = self.robot.getDevice('wheel_left_joint')
        self.rightMotor = self.robot.getDevice('wheel_right_joint')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))

        self.marker = self.robot.getFromDef("marker").getField("translation")

    def initialise(self):
        """Initialize the behavior state when it becomes active.
        
        Stops motors and resets waypoint tracking index.
        """
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
        self.index = 0
        self.WP = self.blackboard.read('waypoints')

    def _calculate_navigation_params(self, current_pos, target_pos, heading):
        """Calculate navigation parameters for waypoint following.
        
        Args:
            current_pos (tuple): Current robot position (x, y)
            target_pos (tuple): Target waypoint position (x, y)
            heading (float): Current robot heading in radians
            
        Returns:
            tuple: (rho, alpha) where:
                  rho: Distance to target
                  alpha: Angle to target relative to current heading
        """
        x, y = current_pos
        target_x, target_y = target_pos
        
        rho = np.sqrt((x - target_x)**2 + (y - target_y)**2)
        alpha = np.arctan2(target_y - y, target_x - x) - heading
        
        # Normalize alpha to [-pi, pi]
        if alpha > np.pi:
            alpha -= 2 * np.pi
        elif alpha < -np.pi:
            alpha += 2 * np.pi
            
        return rho, alpha

    def _calculate_motor_velocities(self, rho, alpha):
        """Calculate wheel motor velocities based on control law.
        
        Args:
            rho (float): Distance to target
            alpha (float): Angle to target
            
        Returns:
            tuple: (vL, vR) Left and right wheel velocities
        """
        # Base velocities from control law
        vL = -self.ANGULAR_GAIN * alpha + self.LINEAR_GAIN * rho
        vR = self.ANGULAR_GAIN * alpha + self.LINEAR_GAIN * rho
        
        # Clamp velocities
        vL = np.clip(vL, -self.MAX_VELOCITY, self.MAX_VELOCITY)
        vR = np.clip(vR, -self.MAX_VELOCITY, self.MAX_VELOCITY)
        
        return vL, vR

    def update(self):
        """Execute one iteration of the navigation behavior.
        
        Updates robot position, calculates control inputs, and checks for
        waypoint completion. Returns SUCCESS when all waypoints are reached,
        otherwise returns RUNNING.
        
        Returns:
            Status: Behavior status (SUCCESS or RUNNING)
        """
        # Get current position and heading
        current_pos = self.gps.getValues()[:2]  # Only take x, y
        heading = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
        
        # Update marker visualization
        self.marker.setSFVec3f([*self.WP[self.index], 0])
        
        # Calculate navigation parameters
        rho, alpha = self._calculate_navigation_params(current_pos, self.WP[self.index], heading)
        
        # Calculate and set motor velocities
        vL, vR = self._calculate_motor_velocities(rho, alpha)
        self.leftMotor.setVelocity(vL)
        self.rightMotor.setVelocity(vR)
        
        # Check if waypoint reached
        if rho < self.WAYPOINT_THRESHOLD:
            print(f"Reached waypoint: index={self.index}, position={self.WP[self.index]}")
            self.index += 1
            
            if self.index == len(self.WP):
                self.feedback_message = "Last waypoint reached"
                return py_trees.common.Status.SUCCESS
                
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Cleanup when behavior transitions to a non-running state
        
        Args:
            new_status: The new status the behavior is transitioning to
        """        
        # Stop the robot's motors
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
        
        # Reset navigation state if needed
        if new_status == py_trees.common.Status.SUCCESS:
            # Navigation completed successfully
            self.index = 0  # Reset waypoint index
            self.feedback_message = "Navigation terminated successfully"
        
        elif new_status == py_trees.common.Status.FAILURE:
            # Navigation failed
            self.feedback_message = "Navigation terminated with failure"
        
        # Remove marker visualization if it exists
        if hasattr(self, 'marker'):
            try:
                self.marker.setSFVec3f([0, 0, -10])  # Move marker out of view
            except:
                pass
```

- **Inheritance**: Extends `py_trees.behaviour.Behaviour`, integrating navigation functionality into behavior trees.
- **Attributes**:
    - **`MAX_VELOCITY`**: Caps the wheel velocities to prevent mechanical strain.
    - **`WAYPOINT_THRESHOLD`**: Defines how close the robot needs to be to a waypoint to consider it reached.
    - **`ANGULAR_GAIN` & `LINEAR_GAIN`**: Control gains for adjusting wheel velocities based on heading and distance errors.
- **Key Methods**:
    - **`setup()`**: Initializes sensors and actuators (GPS, compass, wheel motors, marker).
    - **`initialise()`**: Resets motor velocities and loads the list of waypoints from the blackboard.
    - **`_calculate_navigation_params()`**: Computes distance and heading errors relative to the target waypoint.
    - **`_calculate_motor_velocities()`**: Applies a proportional control law to determine wheel velocities based on computed errors.
    - **`update()`**: Executes navigation logic—calculates control parameters, sets motor velocities, checks for waypoint completion, and updates the marker.
    - **`terminate()`**: Ensures motors are stopped and cleans up visualization elements upon behavior termination.

- **Functionality**: Implements a straightforward proportional controller that guides the robot through a series of waypoints, adjusting its movement based on real-time sensor feedback to ensure accurate navigation.

### 3. `planning.py`

This script is dedicated to path planning using the A* algorithm. It generates an optimal path from the robot's current position to a specified goal while avoiding obstacles detected in the occupancy grid.

#### Planning Class

```python
class Planning(py_trees.behaviour.Behaviour):
    """A behavior tree node for path planning using A* algorithm.
    
    This class implements path planning functionality that takes the robot's current position
    and a goal position to generate an optimal path while avoiding obstacles using the A* algorithm.
    
    Attributes:
        blackboard: Shared data storage for behavior tree nodes
        robot: Reference to the robot controller
        goal: Target destination coordinates
        cspace: Configuration space map representing obstacles
    """
    
    def __init__(self, name, blackboard, goal):
        """Initialize the Planning behavior.
        
        Args:
            name: Name of the behavior node
            blackboard: Shared data storage for behavior tree nodes
            goal: Target destination coordinates (x, y)
        """
        super(Planning, self).__init__(name)
        self.blackboard = blackboard
        self.robot = blackboard.read('robot')
        self.goal = goal
        self.cspace = None  # Initialize cspace as class attribute
        
    def setup(self):
        """Set up the behavior by enabling required sensors."""
        self.timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
    def initialise(self):
        """Load the configuration space map from file.
        
        Returns:
            Status.SUCCESS if map loaded successfully, Status.FAILURE otherwise
        """
        # Load the configuration space map
        try:
            self.cspace = np.load('cspace.npy')
            return py_trees.common.Status.SUCCESS
        except (IOError, ValueError) as e:
            self.feedback_message = f"Map file error: {str(e)}"
            return py_trees.common.Status.FAILURE
            
    def heuristic(self, a, b):
        """Calculate Manhattan distance heuristic between two points.
        
        Args:
            a: Start point coordinates (x, y)
            b: End point coordinates (x, y)
            
        Returns:
            Manhattan distance between points a and b
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
    def is_valid_position(self, pos):
        """Check if a position is valid and traversable in the configuration space.
        
        Args:
            pos: Position coordinates (x, y) to check
            
        Returns:
            bool: True if position is valid and traversable, False otherwise
        """
        return (0 <= pos[0] < self.cspace.shape[0] and 
                0 <= pos[1] < self.cspace.shape[1] and
                self.cspace[pos] < 0.3)
        
    def get_neighbors(self, pos):
        """Get valid neighboring positions using 8-connectivity.
        
        Args:
            pos: Current position coordinates (x, y)
            
        Returns:
            list: List of valid neighboring positions
        """
        directions = [(0,1), (1,0), (0,-1), (-1,0), 
                     (1,1), (1,-1), (-1,1), (-1,-1)]  # 8-connectivity
        return [(pos[0] + dx, pos[1] + dy) for dx, dy in directions 
                if self.is_valid_position((pos[0] + dx, pos[1] + dy))]
        
    def get_movement_cost(self, current, next_pos):
        """Calculate movement cost between adjacent positions.
        
        Args:
            current: Current position coordinates (x, y)
            next_pos: Next position coordinates (x, y)
            
        Returns:
            float: 1.414 for diagonal movement, 1 for cardinal movement
        """
        return 1.414 if abs(next_pos[0]-current[0]) + abs(next_pos[1]-current[1]) == 2 else 1
        
    def plan_path(self, start, goal):
        """Plan a path using A* algorithm from start to goal position.
        
        Args:
            start: Start position in world coordinates (x, y)
            goal: Goal position in world coordinates (x, y)
            
        Returns:
            list: List of waypoints in world coordinates, or None if no path found
        """
        if self.cspace is None:
            return None
            
        start_map = tuple(world2map(start[0], start[1]))
        goal_map = tuple(world2map(goal[0], goal[1]))
        
        # Check if start or goal is in obstacle
        if not (self.is_valid_position(start_map) and self.is_valid_position(goal_map)):
            return None
            
        frontier = []
        heappush(frontier, (0, start_map))
        came_from = {start_map: None}
        cost_so_far = {start_map: 0}
        
        while frontier:
            current = heappop(frontier)[1]
            
            if current == goal_map:
                break
                
            for next_pos in self.get_neighbors(current):
                # Diagonal movement costs sqrt(2), otherwise 1
                new_cost = cost_so_far[current] + self.get_movement_cost(current, next_pos)
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal_map, next_pos)
                    heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current
        
        # Reconstruct path
        if goal_map not in came_from:
            return None
            
        path = []
        current = goal_map
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        
        # Convert path back to world coordinates
        world_path = []
        for p in path:
            wx, wy = map2world(p[0], p[1])
            world_path.append((wx, wy))
            
        return world_path
        
    def update(self):
        """Execute the planning behavior.
        
        Gets current robot position, plans path to goal, and writes waypoints to blackboard.
        
        Returns:
            Status.SUCCESS if path found and written to blackboard, Status.FAILURE otherwise
        """
        # Get current robot position
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        start = (xw, yw)
        
        # Plan path
        path = self.plan_path(start, self.goal)
        
        if path is None:
            self.feedback_message = "No path found"
            return py_trees.common.Status.FAILURE
            
        # Write path to blackboard for navigation
        self.blackboard.write('waypoints', np.array(path))
        self.feedback_message = "Path planned successfully"
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        """Cleanup when behavior finishes.
        
        Args:
            new_status: New status after termination
        """
        pass
```

- **Inheritance**: Extends `py_trees.behaviour.Behaviour`, integrating path planning into behavior trees.
- **Attributes**:
    - **`goal`**: The target coordinates `(x, y)` the robot aims to reach.
    - **`cspace`**: Configuration space map loaded from `cspace.npy`, representing obstacles.
- **Key Methods**:
    - **`setup()`**: Initializes GPS sensor for position retrieval.
    - **`initialise()`**: Loads the configuration space map (`cspace.npy`). Returns `SUCCESS` if successful, `FAILURE` otherwise.
    - **`heuristic()`**: Implements the Manhattan distance heuristic for the A* algorithm.
    - **`is_valid_position()`**: Checks if a given map position is within bounds and free of obstacles.
    - **`get_neighbors()`**: Retrieves valid neighboring positions based on 8-connectivity (including diagonals).
    - **`get_movement_cost()`**: Assigns movement costs—`1.414` for diagonal moves and `1` for cardinal moves.
    - **`plan_path()`**: Executes the A* algorithm to find an optimal path from the start to the goal position in world coordinates.
    - **`update()`**: Initiates path planning and writes the resulting waypoints to the blackboard for the navigation behavior.
    - **`terminate()`**: Placeholder for any cleanup operations upon behavior termination.

- **Functionality**: Facilitates intelligent path planning by generating a collision-free trajectory from the robot's current location to a specified goal, leveraging the precomputed configuration space map.

### 4. `behavior_trees.py`

This script orchestrates the behaviors defined in the previous scripts using behavior trees. It sets up a shared blackboard for inter-behavior communication and constructs a behavior tree that sequences mapping, navigation, and planning tasks.

#### Blackboard Class

```python
class Blackboard:
    """A shared memory space for storing and accessing data between behaviors.
    
    The Blackboard class provides a simple key-value store that allows different
    nodes in the behavior tree to share information.
    """
    
    def __init__(self):
        self.data: dict = {}

    def write(self, key: str, value: any) -> None:
        """Write a value to the blackboard.
        
        Args:
            key (str): The identifier for storing the value
            value (any): The value to store
        """
        self.data[key] = value

    def read(self, key: str) -> any:
        """Read a value from the blackboard.
        
        Args:
            key (str): The identifier of the value to retrieve
            
        Returns:
            any: The value associated with the key, or None if not found
        """
        return self.data.get(key)
```

- **Purpose**: Facilitates data sharing between different behavior nodes within the behavior tree.
- **Mechanism**:
    - **`write()`**: Stores a value with a specified key.
    - **`read()`**: Retrieves a value based on its key.

#### DoesMapExist Class

```python
class DoesMapExist(py_trees.behaviour.Behaviour):
    """A behavior that checks if a map file exists.
    
    This behavior checks for the existence of 'cspace.npy' file and returns
    SUCCESS if found, FAILURE otherwise.
    """
    
    def update(self) -> py_trees.common.Status:
        """Execute the map existence check.
        
        Returns:
            Status: SUCCESS if map exists, FAILURE otherwise
        """
        if exists('cspace.npy'):
            print("Map already exists")
            return py_trees.common.Status.SUCCESS
        print("Map does not exist")
        return py_trees.common.Status.FAILURE
```

- **Purpose**: Checks whether the configuration space map (`cspace.npy`) has already been generated, determining if mapping needs to be performed.
- **Mechanism**:
    - **File Existence Check**: Utilizes the `exists` function to verify the presence of `cspace.npy`.
    - **Status Return**: Returns `SUCCESS` if the map exists, signaling that mapping can be skipped; otherwise, returns `FAILURE` to trigger mapping behavior.

#### Behavior Tree Creation

```python
def create_behavior_tree(blackboard: Blackboard) -> py_trees.trees.BehaviourTree:
    """Create and configure the behavior tree for robot navigation.
    
    Args:
        blackboard (Blackboard): The shared memory space for the behaviors
        
    Returns:
        BehaviourTree: The configured behavior tree ready for execution
    """
    tree = Sequence("Main", memory=True, children=[
        Selector("Does map exist?", children=[
            DoesMapExist("Test for map"),
            Parallel("Mapping", 
                    policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
                    children=[
                        Mapping("map the environment", blackboard),
                        Navigation("move around the table", blackboard)
                    ])
        ], memory=True),
        Planning("compute path to lower left corner", blackboard, (-1.46, -3.12)),
        Navigation("move to lower left corner", blackboard),
        Planning("compute path to sink", blackboard, (0.88, 0.09)),
        Navigation("move to sink", blackboard)
    ])
    tree.setup_with_descendants()
    return tree
```

- **Purpose**: Constructs the behavior tree, defining the sequence and hierarchy of behaviors the robot will execute.
- **Components**:
    - **Sequence**: Ensures that behaviors are executed in a specific order.
    - **Selector**: Chooses between checking for map existence and performing mapping.
    - **Parallel**: Executes mapping and navigation simultaneously, proceeding when one of them succeeds.
    - **Planning and Navigation**: Alternates between path planning and movement to reach designated goals.

- **Behavior Flow**:
    1. **Map Existence Check**:
        - **DoesMapExist**: Checks if `cspace.npy` exists.
        - **Mapping & Navigation**: If the map doesn't exist, initiates mapping and an initial navigation to avoid immediate collision.
    2. **Path Planning and Navigation**:
        - **Planning**: Computes a path to the lower left corner.
        - **Navigation**: Moves the robot along the planned path.
        - **Planning**: Computes a path to the sink.
        - **Navigation**: Moves the robot to the sink.

#### Main Function

```python
def main():
    """Main entry point for the behavior tree-based navigation system.
    
    Initializes the robot, creates a blackboard with necessary data,
    constructs the behavior tree, and runs the main control loop.
    """
    # Initialize robot and blackboard
    robot, timestep = initialize_robot()
    blackboard = Blackboard()
    blackboard.write('robot', robot)
    blackboard.write('waypoints', np.concatenate((WAYPOINTS, np.flip(WAYPOINTS, 0)), axis=0))

    # Create and run behavior tree
    tree = create_behavior_tree(blackboard)
    while robot.step(timestep) != -1:
        tree.tick_once()
```

- **Purpose**: Serves as the entry point of the script, setting up the robot, blackboard, and behavior tree, and initiating the main control loop.
- **Mechanism**:
    - **Robot Initialization**: Calls `initialize_robot()` to set up the robot supervisor and retrieve the simulation timestep.
    - **Blackboard Setup**: Creates a `Blackboard` instance and writes the robot reference and a list of waypoints into it.
    - **Behavior Tree Creation**: Constructs the behavior tree using `create_behavior_tree()`.
    - **Control Loop**: Continuously steps the simulation and ticks the behavior tree to execute behaviors.

#### Helper Function: Initialize Robot

```python
def initialize_robot() -> tuple[Supervisor, int]:
    """Initialize the robot and get the simulation timestep.
    
    Returns:
        tuple[Supervisor, int]: A tuple containing:
            - The initialized robot supervisor
            - The simulation timestep value
    """
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    return robot, timestep
```

- **Purpose**: Simplifies robot initialization by encapsulating the creation of the `Supervisor` instance and retrieval of the simulation timestep.

### Summary of Scripts

- **`mapping.py`**: Focuses on environmental mapping using LIDAR data, converting sensor readings into an occupancy grid and generating a configuration space map for collision-free navigation.
- **`navigation.py`**: Manages the robot's movement through waypoints using sensor feedback, employing a proportional control strategy to adjust wheel velocities based on distance and heading errors.
- **`planning.py`**: Implements the A* path planning algorithm to generate optimal paths from the robot's current position to specified goals, utilizing the configuration space map to avoid obstacles.
- **`behavior_trees.py`**: Integrates the mapping, navigation, and planning behaviors into a structured behavior tree, managing their execution sequence and interdependencies using a shared blackboard for data exchange.

---

## Interconnection of Scripts

The scripts are designed to work cohesively within a behavior tree framework provided by the `py_trees` library. Here's how they interact:

1. **Initialization**:
    - The `behavior_trees.py` script initializes the robot and blackboard, writing essential data like waypoints.
2. **Behavior Tree Construction**:
    - Constructs a behavior tree that sequences and selects between mapping, navigation, and planning behaviors.
3. **Mapping**:
    - If the configuration space map (`cspace.npy`) doesn't exist, the `Mapping` behavior is executed to generate it using LIDAR data.
4. **Navigation & Planning**:
    - Once mapping is complete or if the map already exists, the `Planning` behavior computes an optimal path to the specified goal.
    - The resulting waypoints are fed into the `Navigation` behavior, which guides the robot along the planned path.
5. **Feedback Loop**:
    - The behaviors continuously update based on sensor inputs, allowing the robot to adapt to dynamic environments.

---

## Execution Flow

1. **Startup**:
    - The robot initializes, setting up sensors and motors.
    - The behavior tree starts execution, checking if the environment map exists.
2. **Mapping Phase**:
    - If no map exists, the robot performs mapping, scanning the environment with LIDAR to build the occupancy grid.
    - Once mapping is complete, the configuration space map is saved for path planning.
3. **Path Planning Phase**:
    - The `Planning` behavior computes an optimal path from the robot's current position to the designated goal using the A* algorithm.
    - The planned waypoints are stored on the blackboard for the navigation behavior to consume.
4. **Navigation Phase**:
    - The `Navigation` behavior reads the waypoints and adjusts the robot's movement accordingly, guiding it along the planned path.
    - Sensor feedback ensures that the robot stays on course, making real-time adjustments as needed.
5. **Loop**:
    - The behavior tree continuously cycles through mapping, planning, and navigation, allowing the robot to handle dynamic changes in the environment.

---

## Potential Enhancements

While the current setup provides a solid foundation for autonomous navigation and mapping, several enhancements can further elevate the system's capabilities:

1. **Dynamic Replanning**:
    - Implement mechanisms to detect dynamic obstacles and trigger replanning on-the-fly, ensuring adaptability in changing environments.
2. **Advanced Control Strategies**:
    - Incorporate PID controllers for more refined velocity adjustments, reducing overshooting and oscillations.
3. **Sensor Fusion**:
    - Integrate additional sensors like IMUs or wheel encoders to enhance pose estimation and navigation accuracy.
4. **3D Mapping and Path Planning**:
    - Extend the occupancy grid to three dimensions, enabling the robot to navigate in more complex environments with varying elevations.
5. **Interactive Visualization**:
    - Develop real-time visualization tools to monitor the robot's mapping, planning, and navigation processes, aiding in debugging and performance assessment.
6. **Energy Efficiency**:
    - Optimize motor usage based on movement requirements to enhance the robot's energy efficiency and operational lifespan.

---

## Conclusion

The integration of `mapping.py`, `navigation.py`, `planning.py`, and `behavior_trees.py` within a Webots simulation environment showcases a comprehensive approach to autonomous robotics. By leveraging behavior trees, the system modularizes complex tasks into manageable behaviors, facilitating scalable and maintainable robot control architectures.

- **Environmental Mapping**: Provides the robot with a spatial understanding of its surroundings, essential for safe navigation.
- **Path Planning**: Utilizes intelligent algorithms like A* to compute optimal routes, ensuring efficiency and obstacle avoidance.
- **Navigation Control**: Employs proportional control strategies to guide the robot accurately along planned paths.
- **Behavior Tree Orchestration**: Seamlessly integrates various behaviors, managing their execution and interdependencies effectively.

As autonomous robotics continues to evolve, such structured and modular approaches become increasingly vital, enabling robots to perform complex tasks with reliability and intelligence. This project serves as an exemplary model for aspiring roboticists aiming to build sophisticated autonomous systems using simulation environments like Webots and powerful Python libraries.

