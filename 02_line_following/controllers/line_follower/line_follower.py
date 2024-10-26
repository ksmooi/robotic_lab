from controller import Robot, Motor, DistanceSensor
import math

class LineFollower:
    """
    LineFollower is an object-oriented representation of a line-following robot controller.
    
    This class initializes the robot's sensors and motors, implements the line-following logic,
    and calculates odometry information such as total distance traveled and total rotation.
    """
    
    def __init__(self):
        """
        Initializes the LineFollower instance by setting up the robot, sensors, motors,
        and relevant constants.
        """
        # Create the Robot instance
        self.robot = Robot()
        
        # Get the time step of the current world
        self.timestep = int(self.robot.getBasicTimeStep())
        self.time_step_seconds = self.timestep / 1000.0  # Convert to seconds
        
        # Constants
        self.MAX_SPEED = 6.28  # Maximum speed in radians/second
        self.WHEEL_RADIUS = 0.0201  # meters
        self.WHEEL_DISTANCE = 0.052  # meters
        
        # Initialize ground sensors
        self.ground_sensors = self._initialize_ground_sensors()
        
        # Initialize motors
        self.left_motor, self.right_motor = self._initialize_motors()
        
        # Initialize odometry variables
        self.total_distance = 0.0
        self.total_rotation = 0.0  # in radians
        
        # Move robot slightly forward to not detect the start line immediately
        self._move_forward_initial()
    
    def _initialize_ground_sensors(self):
        """
        Initializes the ground sensors used for line detection.
        
        Returns:
            list: A list of initialized DistanceSensor devices.
        """
        sensors = []
        for i in range(3):
            sensor = self.robot.getDevice(f'gs{i}')
            sensor.enable(self.timestep)
            sensors.append(sensor)
        return sensors
    
    def _initialize_motors(self):
        """
        Initializes the left and right wheel motors.
        
        Returns:
            tuple: A tuple containing the left and right Motor devices.
        """
        left_motor = self.robot.getDevice('left wheel motor')
        right_motor = self.robot.getDevice('right wheel motor')
        left_motor.setPosition(float('inf'))
        right_motor.setPosition(float('inf'))
        return left_motor, right_motor
    
    def _move_forward_initial(self):
        """
        Moves the robot forward for a short duration to avoid immediate detection of the start line.
        """
        self.left_motor.setVelocity(self.MAX_SPEED)
        self.right_motor.setVelocity(self.MAX_SPEED)
        self.robot.step(1000)  # Move for 1 second
    
    def run(self):
        """
        Runs the main control loop for the line-following robot.
        Continuously reads sensor values, determines motor actions based on sensor input,
        updates odometry, and prints the robot's current state.
        """
        while self.robot.step(self.timestep) != -1:
            # Read ground sensors
            sensor_values = [sensor.getValue() for sensor in self.ground_sensors]
            
            # Line following logic
            left_sensor, center_sensor, right_sensor = sensor_values
            if left_sensor > 500 and center_sensor < 350 and right_sensor > 500:
                # On line, drive straight
                left_speed = self.MAX_SPEED
                right_speed = self.MAX_SPEED
            elif right_sensor < 550:
                # Line is on the right, turn right
                left_speed = 0.25 * self.MAX_SPEED
                right_speed = -0.1 * self.MAX_SPEED
            elif left_sensor < 550:
                # Line is on the left, turn left
                left_speed = -0.1 * self.MAX_SPEED
                right_speed = 0.25 * self.MAX_SPEED
            else:
                # Default case, search by turning
                left_speed = 0.15 * self.MAX_SPEED
                right_speed = -0.15 * self.MAX_SPEED
            
            # Set motor velocities
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
            
            # Calculate displacement for this timestep
            delta_x = (self.WHEEL_RADIUS * (left_speed + right_speed) / 2.0) * self.time_step_seconds
            
            # Calculate rotation for this timestep (positive is counterclockwise)
            delta_omega = (self.WHEEL_RADIUS * (right_speed - left_speed) / self.WHEEL_DISTANCE) * self.time_step_seconds
            
            # Update total distance and rotation
            self.total_distance += abs(delta_x)  # Use absolute value for total distance
            self.total_rotation += delta_omega
            
            # Print odometry information (convert rotation to degrees for readability)
            rotation_degrees = (self.total_rotation / math.pi) * 180.0
            print(f"Distance traveled: {self.total_distance:.3f} meters")
            print(f"Current rotation: {rotation_degrees:.1f} degrees")

if __name__ == "__main__":
    line_follower = LineFollower()
    line_follower.run()

