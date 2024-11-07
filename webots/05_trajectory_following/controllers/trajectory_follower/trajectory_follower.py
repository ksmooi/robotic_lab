import math
from controller import Robot, Supervisor, Motor, DistanceSensor
import matplotlib.pyplot as plt
import numpy as np

class TrajectoryFollower:
    """
    A controller class for a differential drive robot that follows a predefined trajectory.
    
    This class implements a waypoint-following behavior using GPS and compass sensors
    for localization. It uses a simple proportional controller to adjust wheel speeds
    based on distance and heading errors to the current target waypoint.
    
    Attributes:
        MAX_SPEED (float): Maximum wheel speed in radians/second
        WHEEL_RADIUS (float): Radius of the wheels in meters
        WHEEL_DISTANCE (float): Distance between wheels in meters
    """
    
    # Constants
    MAX_SPEED = 6.28              # Maximum speed in radians/second
    WHEEL_RADIUS = 0.0201         # meters
    WHEEL_DISTANCE = 0.052        # meters
    VISUALIZE_OPTION = 0          # 0 = No visualization, 1 = Robot frame, 2 = World frame, 3 = Transform and visualize

    def __init__(self):
        """
        Initialize the robot controller with necessary sensors and actuators.
        
        Sets up the robot supervisor, motors, GPS, compass, and initializes the waypoint
        trajectory that the robot will follow.
        """
        # Initialize the Robot instance and timestep
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.time_step_seconds = self.timestep / 1000.0  # Convert to seconds

        # Initialize sensors and motors
        self.left_motor, self.right_motor = self.initialize_motors()

        # Add display initialization (add on children node of the robot)
        self.display = self.robot.getDevice('display')
        
        # Accessing the GPS (add on children node of the robot)
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
        # Accessing the compass (add on children node of the robot)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)

        # Accessing the marker (ping pong ball)
        self.marker = self.robot.getFromDef("marker").getField("translation")
        
        self.WP = [(0, 0.68), (0.44, 0.68), (0.66, 0.51), (0.35, 0.24), 
                   (0.63, 0), (0.63, -0.17), (0, -0.17), (0, 0.68)]
        self.index = 0

        # Start moving forward slightly to avoid immediate start line detection
        self.set_motor_velocity(self.MAX_SPEED, self.MAX_SPEED)
        self.robot.step(1000)  # Move for 1 second

    def initialize_motors(self):
        """
        Initialize and configure the robot's wheel motors.
        
        Sets up both left and right wheel motors to run in velocity control mode
        by setting their positions to infinity.
        
        Returns:
            tuple: (left_motor, right_motor) Motor device objects for both wheels
        """
        left_motor = self.robot.getDevice('left wheel motor')
        right_motor = self.robot.getDevice('right wheel motor')
        
        # Set the motors to run indefinitely
        left_motor.setPosition(float('inf'))
        right_motor.setPosition(float('inf'))
        
        return left_motor, right_motor

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

    def calculate_wheel_speeds(self, rho, alpha):
        """
        Calculate desired wheel speeds based on distance and heading errors.
        
        Implements a proportional controller where:
        - Angular correction is proportional to heading error (alpha)
        - Forward speed is proportional to distance error (rho)
        
        Args:
            rho (float): Distance to target waypoint in meters
            alpha (float): Heading error in radians
            
        Returns:
            tuple: (left_speed, right_speed) Desired wheel velocities in rad/s,
                  clamped to MAX_SPEED
        """
        p1 = 6  # Angular gain
        p2 = self.MAX_SPEED * 2  # Linear gain

        left_speed = -alpha * p1 + rho * p2
        right_speed = alpha * p1 + rho * p2

        # Clamp speeds to maximum
        left_speed = min(left_speed, self.MAX_SPEED)
        right_speed = min(right_speed, self.MAX_SPEED)
        
        return left_speed, right_speed

    def run(self):
        """
        Main control loop for the trajectory following behavior.
        
        Continuously:
        1. Gets current robot pose
        2. Updates marker position to show current target
        3. Calculates control errors
        4. Updates wheel speeds
        5. Checks if current waypoint is reached and updates target
        
        The loop continues until the simulation is stopped.
        """
        while self.robot.step(self.timestep) != -1:
            # Get current robot pose
            x, y, theta = self.get_robot_pose()
            
            # Update marker position
            self.marker.setSFVec3f([*self.WP[self.index], 0])
            
            # Calculate control errors
            rho, alpha = self.calculate_control_errors(x, y, theta)
            
            # Calculate and set motor speeds
            left_speed, right_speed = self.calculate_wheel_speeds(rho, alpha)
            self.set_motor_velocity(left_speed, right_speed)
            
            # Check if waypoint reached and update index
            if rho < 0.15:
                self.index = 0 if self.index >= len(self.WP) - 1 else self.index + 1
                # print(f"Stepped to index: {self.index}")

if __name__ == "__main__":
    follower = TrajectoryFollower()
    follower.run()
