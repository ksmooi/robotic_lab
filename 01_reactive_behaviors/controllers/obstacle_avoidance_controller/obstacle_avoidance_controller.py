from controller import Robot, Motor, DistanceSensor
import math

class ObstacleAvoidanceController:
    """
    A controller for robot obstacle avoidance using distance sensors and motor control.
    The robot navigates through different states to detect and avoid obstacles.
    """

    # Constants
    DISTANCE_THRESHOLD = 0.05  # 0.05 meters threshold
    MAX_SPEED = 5.0            # Maximum speed for the wheels
    TURN_SPEED = 2.0           # Speed for turning
    TURN_180_DURATION = 4000   # milliseconds for 180-degree turn

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

    def get_left_distance(self):
        """
        Retrieve the distance from the left sensor (ps5).

        Returns:
            float: The left distance value.
        """
        return self.distance_sensors[5].getValue()

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

if __name__ == "__main__":
    controller = ObstacleAvoidanceController()
    controller.run()
