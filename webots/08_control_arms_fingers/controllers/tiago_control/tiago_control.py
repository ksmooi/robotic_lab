from controller import Supervisor
import numpy as np
from scipy import signal
from datetime import datetime
from matplotlib import pyplot as plt

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
        map (np.ndarray): Occupancy map of the environment.
        trajectory_map (np.ndarray): Map of the robot's trajectory.
        kernel (np.ndarray): Kernel used for map processing.
        angles (np.ndarray): Array of angles corresponding to LIDAR measurements.
    """

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
        
        # Mapping variables
        self.map = np.zeros((250, 300))
        self.trajectory_map = np.zeros((250, 300))
        self.kernel = np.ones((20, 20))
        self.angles = np.linspace(2.094395, -2.094395, 667)

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

    def control_wheel(self, action=None, waypoints=None):
        """
        Controls the robot's wheel movement to navigate through specified waypoints.

        Args:
            action (str, optional): Action name for logging purposes.
            waypoints (list of tuples, optional): List of (x, y) coordinates to navigate through.

        Raises:
            ValueError: If no waypoints are provided.
            RuntimeError: If movement fails during execution.

        Returns:
            bool: True if navigation is successful, otherwise raises an exception.
        """
        if not waypoints:
            raise ValueError("No waypoints provided")
        
        try:
            index = 0
            while self.robot.step(self.timestep) != -1:
                # Get current position and orientation
                gps_values = self.gps.getValues()
                compass_values = self.compass.getValues()
                xw, yw = gps_values[0], gps_values[1]
                theta = np.arctan2(compass_values[0], compass_values[1])
                
                # Update marker position
                self.marker.setSFVec3f([*waypoints[index], 0])
                
                # Calculate errors
                rho = np.sqrt((xw - waypoints[index][0])**2 + 
                             (yw - waypoints[index][1])**2)
                alpha = np.arctan2(waypoints[index][1] - yw, 
                                 waypoints[index][0] - xw) - theta
                
                # Normalize alpha to be within [-pi, pi]
                if alpha > np.pi: alpha -= 2 * np.pi
                elif alpha < -np.pi: alpha += 2 * np.pi
                
                # Calculate and set motor speeds
                left_speed, right_speed = self.calculate_motor_speeds(alpha, rho)
                
                # Check if waypoint is reached
                if rho < 0.5:
                    self.print_debug(f"  > Reached waypoint '{index}', position: {waypoints[index]}")
                    index += 1
                    if index == len(waypoints):
                        self.leftMotor.setVelocity(0)
                        self.rightMotor.setVelocity(0)
                        self.print_debug(f"Wheel action '{action}' completed")
                        return True
                
                # Set motor velocities
                self.leftMotor.setVelocity(left_speed)
                self.rightMotor.setVelocity(right_speed)
                
                # Update mapping with current position
                self.update_mapping(xw, yw, theta)
                
                # Update trajectory map and display
                px, py = self.world2map(xw, yw)
                self.trajectory_map[px, py] = 1.0
                self.display.setColor(0xFF0000)
                for i in range(250):
                    for j in range(300):
                        if self.trajectory_map[i, j] > 0:
                            self.display.drawPixel(i, j)
        
        except Exception as e:
            # Stop the motors in case of any exception
            self.leftMotor.setVelocity(0)
            self.rightMotor.setVelocity(0)
            raise RuntimeError(f"Wheel control failed during '{action}': {str(e)}")

    def control_arm(self, action=None, positions=None, tolerance=0.005):
        """
        Controls the robot's arm by setting specified joint positions and waits
        until the arm reaches the desired positions within a given tolerance.

        Args:
            action (str, optional): Action name for logging purposes.
            positions (dict, optional): Dictionary of joint names and target positions.
            tolerance (float, optional): Position error tolerance in radians.

        Raises:
            ValueError: If provided positions are invalid.
            RuntimeError: If movement fails or times out.

        Returns:
            bool: True if the arm reaches the target positions, otherwise raises an exception.
        """
        if not positions:
            positions = self.robot_init_joints
        
        try:
            # Validate and set target positions for each joint
            for joint_name, target_position in positions.items():
                if joint_name not in self.motor_handles:
                    raise ValueError(f"Invalid joint name: {joint_name}")
                self.motor_handles[joint_name].setPosition(target_position)

            # Wait for the arm to reach the target positions
            timeout_steps = 1000  # Prevent infinite loops
            steps = 0
            
            while self.robot.step(self.timestep) != -1:
                steps += 1
                if steps > timeout_steps:
                    raise RuntimeError("Movement timeout exceeded")
                
                all_reached = True
                for joint_name, target_position in positions.items():
                    if joint_name in self.encoders:
                        current_pos = self.encoders[joint_name].getValue()
                        if abs(current_pos - target_position) > tolerance:
                            all_reached = False
                            break
                
                if all_reached:
                    self.print_debug(f"Arm action '{action}' completed")
                    return True
                
        except Exception as e:
            raise RuntimeError(f"Arm control failed during '{action}': {str(e)}")

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

    def update_mapping(self, xw, yw, theta):
        """
        Updates the occupancy map based on LIDAR data and the robot's current position.

        Args:
            xw (float): X-coordinate in the world frame.
            yw (float): Y-coordinate in the world frame.
            theta (float): Orientation angle of the robot in radians.
        """
        ranges = np.array(self.lidar.getRangeImage())
        ranges[ranges == np.inf] = 100  # Replace infinite ranges with a large number
        
        # Select a subset of LIDAR ranges and corresponding angles
        valid_ranges = ranges[80:-80]
        valid_angles = self.angles[80:-80]
        
        # Convert polar coordinates to Cartesian coordinates in the robot frame
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

    def wait_for_milliseconds(self, milliseconds):
        """
        Pauses the controller for a specified duration in milliseconds.

        Args:
            milliseconds (int): Duration to wait in milliseconds.
        """
        loop = int(milliseconds / self.timestep)
        for _ in range(loop):
            self.robot.step(self.timestep)

    def print_debug(self, message):
        """
        Prints a debug message with a timestamp.

        Args:
            message (str): The debug message to print.
        """
        # Get the current date and time formatted to milliseconds
        formatted_time = datetime.now().strftime('%Y%m%d %H%M%S.%f')[:-3]
        print(f"{formatted_time}\t{message}")

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


if __name__ == "__main__":
    """
    Entry point of the script. Creates an instance of TiagoController and
    starts the control loop.
    """
    # Create and run the controller
    controller = TiagoController()
    controller.run()
