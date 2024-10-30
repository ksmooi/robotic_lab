import math
from controller import Robot, Motor, DistanceSensor

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

        # Initialize sensors and motors
        self.ground_sensors = self.initialize_ground_sensors(3)
        self.left_motor, self.right_motor = self.initialize_motors()

        # Initialize odometry variables
        self.total_distance = 0.0
        self.total_rotation = 0.0  # in radians

        # Initialize position variables
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0  # Will remain 0 for 2D movement

        # Start moving forward slightly to avoid immediate start line detection
        self.set_motor_velocity(self.MAX_SPEED, self.MAX_SPEED)
        self.robot.step(1000)  # Move for 1 second

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

    def set_motor_velocity(self, left_velocity, right_velocity):
        """
        Sets the velocities of the left and right motors.

        Args:
            left_velocity (float): Velocity for the left motor.
            right_velocity (float): Velocity for the right motor.
        """
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

    def read_ground_sensors(self):
        """
        Reads the current values from all ground sensors.

        Returns:
            list: A list of sensor values.
        """
        return [sensor.getValue() for sensor in self.ground_sensors]

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

if __name__ == "__main__":
    odometry_counter = OdometryCounter()
    odometry_counter.run()
