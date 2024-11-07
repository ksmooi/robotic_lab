from controller import Robot, Supervisor
import numpy as np
from scipy import signal
from matplotlib import pyplot as plt

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

    def __init__(self):
        """
        Initializes the TrajectoryGenerator by setting up the robot, sensors, actuators,
        mapping structures, and predefined waypoints.
        """
        # Initialize Robot and Supervisor
        self.robot = Supervisor()

        # Time step
        self.timestep = int(self.robot.getBasicTimeStep())

        # Initialize sensors and actuators
        self._init_motors()
        self._init_sensors()
        self._init_display()
        self._init_lidar()

        # Initialize mapping structures
        self.map = np.zeros((self.MAP_WIDTH, self.MAP_HEIGHT))
        self.trajectory_map = np.zeros((self.MAP_WIDTH, self.MAP_HEIGHT))
        self.kernel = np.ones((20, 20))

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

        # Angles for Lidar readings
        self.angles = np.linspace(2.094395, -2.094395, 667)  # From ~120 degrees to -120 degrees

    def _init_motors(self):
        """
        Initializes the left and right wheel motors, setting their positions to infinity
        to allow velocity control.
        """
        self.left_motor = self.robot.getDevice('wheel_left_joint')
        self.right_motor = self.robot.getDevice('wheel_right_joint')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

    def _init_sensors(self):
        """
        Initializes the GPS and Compass sensors, enabling them with the defined timestep.
        """
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)

    def _init_display(self):
        """
        Initializes the display device and the marker used for visualizing waypoints.
        It sets the initial color to white, clears the display, and then sets it back to red
        for drawing the trajectory and map points.
        """
        self.display = self.robot.getDevice('display')
        
        # Initialize display to white and clear any previous drawings
        # self.display.setColor(0xFFFFFF)  # Set initial color to white
        # self.display.fillRectangle(0, 0, self.MAP_WIDTH, self.MAP_HEIGHT)  # Clear display
        # self.display.setColor(0xFF0000)  # Set back to red for drawing

        # Retrieve the 'marker' object from the simulation environment
        self.marker = self.robot.getFromDef("marker").getField("translation")

    def _init_lidar(self):
        """
        Initializes the Lidar sensor, enabling it and its point cloud data with the defined timestep.
        """
        self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

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

    def stop_robot(self):
        """
        Stops the robot by setting both motor velocities to zero.
        """
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

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

if __name__ == "__main__":
    """
    Entry point for the script. Instantiates the TrajectoryGenerator and starts the control loop.
    """
    traj_gen = TrajectoryGenerator()
    traj_gen.run()
