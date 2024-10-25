from controller import Robot, Motor, DistanceSensor
import math

# Constants
DISTANCE_THRESHOLD = 0.05  # 0.05 meters threshold
MAX_SPEED = 5.0  # Maximum speed for the wheels
TURN_SPEED = 2.0  # Speed for turning

# Initialize the robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize motors
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

# Initialize distance sensors
ds = []
for i in range(8):
    ds.append(robot.getDevice('ps' + str(i)))
    ds[-1].enable(timestep)

# Robot states
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

# Initialize state
current_state = STATE_FORWARD_TO_A
turn_start_time = 0
turn_180_duration = 4000  # milliseconds for 180-degree turn
previous_state = None

def get_front_distance():
    # Get values from front sensors (ps0 and ps7)
    left_front = ds[0].getValue()
    right_front = ds[7].getValue()
    print(f"Raw front sensor values - Left: {left_front}, Right: {right_front}")
    return max(left_front, right_front)

def get_left_distance():
    # Left sensor (ps5)
    return ds[5].getValue()

def print_debug_info(front_distance, left_distance, raw_front_value, raw_left_value):
    global previous_state
    if current_state != previous_state:
        print(f"\n--- State Change: {state_names[current_state]} ---")
        previous_state = current_state
    
    print(f"State: {state_names[current_state]}")
    print(f"Raw Front Value: {raw_front_value}")
    print(f"Raw Left Value: {raw_left_value}")
    print(f"Front Distance: {front_distance:.3f}m")
    print(f"Left Distance: {left_distance:.3f}m")
    
    if current_state == STATE_TURN_180:
        elapsed_time = robot.getTime() * 1000 - turn_start_time
        turn_percentage = (elapsed_time / turn_180_duration) * 100
        print(f"Turn Progress: {turn_percentage:.1f}%")
    print("-------------------")

# Main control loop
while robot.step(timestep) != -1:
    # Read raw sensor values
    raw_front_value = get_front_distance()
    raw_left_value = get_left_distance()
    front_distance = raw_front_value / 100.0
    left_distance = raw_left_value / 100.0
    
    # Print debug information
    print_debug_info(front_distance, left_distance, raw_front_value, raw_left_value)
    
    if current_state == STATE_FORWARD_TO_A:
        if raw_front_value > 80:  # ~0.05m threshold
            print("*** DETECTED BOX A - STARTING 180 TURN ***")
            current_state = STATE_TURN_180
            turn_start_time = robot.getTime() * 1000
            motor_left.setVelocity(0)
            motor_right.setVelocity(0)
        else:
            motor_left.setVelocity(MAX_SPEED)
            motor_right.setVelocity(MAX_SPEED)
    
    elif current_state == STATE_TURN_180:
        elapsed_time = robot.getTime() * 1000 - turn_start_time
        
        if elapsed_time >= turn_180_duration:
            print("*** COMPLETED 180 TURN - MOVING TO BOX B ***")
            current_state = STATE_FORWARD_TO_B
            motor_left.setVelocity(0)
            motor_right.setVelocity(0)
        else:
            turn_multiplier = 3.0
            motor_left.setVelocity(-TURN_SPEED * turn_multiplier)
            motor_right.setVelocity(TURN_SPEED * turn_multiplier)
    
    elif current_state == STATE_FORWARD_TO_B:
        if raw_front_value > 80:  # ~0.05m threshold
            print("*** DETECTED BOX B - STARTING CLOCKWISE ROTATION ***")
            current_state = STATE_ROTATE_CLOCKWISE
            motor_left.setVelocity(0)
            motor_right.setVelocity(0)
        else:
            motor_left.setVelocity(MAX_SPEED)
            motor_right.setVelocity(MAX_SPEED)
    
    elif current_state == STATE_ROTATE_CLOCKWISE:
        if raw_left_value > 80:  # ~0.05m threshold
            print("*** LEFT SENSOR DETECTED WALL - STARTING WALL FOLLOWING ***")
            current_state = STATE_WALL_FOLLOWING
            motor_left.setVelocity(MAX_SPEED)
            motor_right.setVelocity(MAX_SPEED)
        else:
            # Rotate clockwise until left sensor detects the wall
            motor_left.setVelocity(TURN_SPEED)
            motor_right.setVelocity(-TURN_SPEED)
    
    elif current_state == STATE_WALL_FOLLOWING:
        if raw_left_value > 80:  # ~0.05m threshold
            # Continue moving forward while wall is detected on left
            motor_left.setVelocity(MAX_SPEED)
            motor_right.setVelocity(MAX_SPEED)
        else:
            # Stop if wall is lost
            print("*** WALL LOST - STOPPING ***")
            motor_left.setVelocity(0)
            motor_right.setVelocity(0)