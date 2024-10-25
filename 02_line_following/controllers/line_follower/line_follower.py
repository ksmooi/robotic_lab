
from controller import Robot, Motor, DistanceSensor
import math

# Create the Robot instance
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Constants
MAX_SPEED = 6.28  # Maximum speed in radians/second
WHEEL_RADIUS = 0.0201  # meters
WHEEL_DISTANCE = 0.052  # meters
TIME_STEP_SECONDS = timestep / 1000.0  # Convert to seconds

# Initialize ground sensors
gs = []
for i in range(3):
    gs.append(robot.getDevice('gs' + str(i)))
    gs[-1].enable(timestep)

# Initialize motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Initialize odometry variables
total_distance = 0.0
total_rotation = 0.0  # in radians

# Move robot slightly forward to not detect start line immediately
leftMotor.setVelocity(MAX_SPEED)
rightMotor.setVelocity(MAX_SPEED)
robot.step(1000)  # Move for 1 second

while robot.step(timestep) != -1:
    # Read ground sensors
    g = []
    for sensor in gs:
        g.append(sensor.getValue())
    
    # Line following logic
    if g[0] > 500 and g[1] < 350 and g[2] > 500:  # On line, drive straight
        phildot = MAX_SPEED
        phirdot = MAX_SPEED
    elif g[2] < 550:  # Line is on the right, turn right
        phildot = 0.25 * MAX_SPEED
        phirdot = -0.1 * MAX_SPEED
    elif g[0] < 550:  # Line is on the left, turn left
        phildot = -0.1 * MAX_SPEED
        phirdot = 0.25 * MAX_SPEED
    else:  # Default case, search by turning
        phildot = 0.15 * MAX_SPEED
        phirdot = -0.15 * MAX_SPEED

    # Set motor velocities
    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)
    
    # Calculate displacement for this timestep
    delta_x = (WHEEL_RADIUS * (phildot + phirdot) / 2.0) * TIME_STEP_SECONDS
    
    # Calculate rotation for this timestep (positive is counterclockwise)
    delta_omega = (WHEEL_RADIUS * (phirdot - phildot) / WHEEL_DISTANCE) * TIME_STEP_SECONDS
    
    # Update total distance and rotation
    total_distance += abs(delta_x)  # Use absolute value for total distance
    total_rotation += delta_omega
    
    # Print odometry information (convert rotation to degrees for readability)
    rotation_degrees = (total_rotation / math.pi) * 180.0
    print(f"Distance traveled: {total_distance:.3f} meters")
    print(f"Current rotation: {rotation_degrees:.1f} degrees")