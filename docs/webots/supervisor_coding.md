# Supervisor Programming in Webots: An In-Depth Guide

**Webots User Guide R2023b**

Webots is a sophisticated open-source robotics simulator that enables developers to design, simulate, and test robotic systems in a virtual environment. Beyond standard controller programming, Webots offers Supervisor Programming, which provides advanced capabilities to manage and manipulate the simulation environment dynamically. This guide delves into Supervisor Programming, elucidating its functionalities, applications, and practical examples. While the examples provided are in C, the concepts are universally applicable to other supported languages such as C++, Java, Python, and MATLAB.

## Table of Contents

1. [Introduction](#introduction)
2. [Tracking the Position of Robots](#tracking-the-position-of-robots)
3. [Setting the Position of Robots](#setting-the-position-of-robots)

---

## Introduction

In Webots, Supervisor Programming extends the capabilities of standard robot controllers by granting access to the `wb_supervisor_*` functions in addition to the regular `wb_robot_*` functions. This dual functionality is enabled by setting the **supervisor** field of a `Robot` node to `TRUE`. A supervisor-enabled `Robot` node retains all functionalities of a regular robot node while providing enhanced control over the simulation process and the Scene Tree.

### Key Features of Supervisor Programming

- **Simulation Control**: Supervisors can start, restart, or terminate simulations programmatically.
- **Scene Tree Manipulation**: Modify the Scene Tree by adding, removing, or altering nodes and their properties.
- **Data Management**: Record and analyze simulation data such as robot trajectories and environmental changes.
- **Advanced Operations**: Take screenshots or videos, adjust lighting conditions, and manipulate object attributes within the simulation.

**Important Consideration**: Supervisor functionalities are akin to human intervention within the simulation environment and are not typically available on real robots. This distinction underscores the specialized nature of the `wb_supervisor_*` functions compared to the standard `wb_robot_*` functions.

---

## Tracking the Position of Robots

Tracking the position of robots is a fundamental task in robotics simulations, essential for tasks such as navigation, path planning, and performance analysis. While robots can independently determine their positions using onboard sensors like GPS, Supervisor Programming offers a centralized and efficient method to monitor multiple robots simultaneously.

### Example: Tracking a Single Robot's Position

The following example demonstrates how to use the Supervisor API to track and print the position of a single robot named `MY_ROBOT`. This approach can be easily scaled to monitor multiple robots by extending the logic accordingly.

```python
from controller import Supervisor
import sys

TIME_STEP = 32  # Time step in milliseconds

# Initialize the Supervisor
supervisor = Supervisor()

# Retrieve the node reference for the robot with DEF name "MY_ROBOT"
robot_node = supervisor.getFromDef("MY_ROBOT")
if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)

# Access the 'translation' field of the robot node to get its position
trans_field = robot_node.getField("translation")

# Main loop to track the robot's position
while supervisor.step(TIME_STEP) != -1:
    # Retrieve the current position as a list [x, y, z]
    values = trans_field.getSFVec3f()
    print(f"MY_ROBOT is at position: {values[0]} {values[1]} {values[2]}")
```

### Explanation

1. **Supervisor Initialization**: Instantiate the Supervisor object to gain access to Supervisor-specific functions.
2. **Node Retrieval**: Use `getFromDef("MY_ROBOT")` to obtain a reference to the robot node defined with the DEF name `MY_ROBOT` in the world file (`.wbt` or `.proto`).
3. **Field Access**: Access the `translation` field of the robot node, which contains its position in the global coordinate system.
4. **Position Tracking Loop**: Continuously call `supervisor.step(TIME_STEP)` to advance the simulation and retrieve the latest position values, printing them to the console.

**Best Practices**:
- **Initial Setup Outside the Loop**: Obtain node and field references once before entering the main loop to enhance performance and avoid redundant operations.
- **DEF Name Usage**: Ensure that the DEF name used in `getFromDef` matches exactly with the one defined in the robot's description.

---

## Setting the Position of Robots

Supervisor Programming not only allows tracking but also empowers developers to manipulate the simulation environment by setting or resetting the positions of robots. This capability is invaluable for tasks such as resetting experiments, optimizing robot behaviors, and conducting systematic evaluations.

### Example: Optimizing Robot Locomotion by Setting Positions

The following example illustrates how to use the Supervisor API to optimize a robot's locomotion by adjusting two parameters (`a` and `b`) and measuring the distance traveled. After each evaluation, the robot is reset to its initial position to ensure consistent starting conditions for subsequent trials.

```python
from math import sqrt
from controller import Supervisor
import sys

TIME_STEP = 32  # Time step in milliseconds

# Initialize the Supervisor
supervisor = Supervisor()

# Retrieve the node reference for the robot with DEF name "MY_ROBOT"
robot_node = supervisor.getFromDef("MY_ROBOT")
if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)

# Access the 'translation' field of the robot node
trans_field = robot_node.getField("translation")

# Define the initial position to reset the robot after each evaluation
INITIAL_POSITION = [0, 0.5, 0]

# Iterate over parameter 'a' from 0 to 24
for a in range(0, 25):
    # Iterate over parameter 'b' from 0 to 32
    for b in range(0, 33):
        # Record the start time of the evaluation
        start_time = supervisor.getTime()
        
        # Evaluate robot behavior for 60 seconds of simulation time
        while supervisor.getTime() - start_time < 60:
            # Placeholder for robot control logic based on parameters 'a' and 'b'
            # Example: set motor positions or velocities
            # set_robot_control(a, b)
            
            # Advance the simulation by one time step
            if supervisor.step(TIME_STEP) == -1:
                sys.exit(0)
        
        # Retrieve the final position of the robot
        values = trans_field.getSFVec3f()
        distance_traveled = sqrt(values[0]**2 + values[2]**2)
        print(f"a={a}, b={b} -> distance={distance_traveled}")
        
        # Reset the robot's position to the initial coordinates
        trans_field.setSFVec3f(INITIAL_POSITION)
        
        # Reset the physics to clear any residual forces or motions
        robot_node.resetPhysics()
```

### Explanation

1. **Supervisor Initialization**: Instantiate the Supervisor object to access Supervisor-specific functions.
2. **Node Retrieval and Field Access**: Obtain references to the robot node and its `translation` field.
3. **Parameter Iteration**: Use nested loops to iterate over the two parameters `a` and `b`, representing the search space for optimization.
4. **Robot Evaluation**:
    - **Control Logic Placeholder**: Insert robot control logic that adjusts behaviors based on the current values of `a` and `b`. This could involve setting motor positions, velocities, or other actuator parameters.
    - **Simulation Advancement**: Continuously call `supervisor.step(TIME_STEP)` to advance the simulation, ensuring that motor commands are processed at each time step.
5. **Distance Measurement**: After 60 seconds of simulation time, calculate the distance traveled using the robot's final position.
6. **Robot Reset**:
    - **Position Reset**: Use `trans_field.setSFVec3f(INITIAL_POSITION)` to move the robot back to its starting position.
    - **Physics Reset**: Call `robot_node.resetPhysics()` to clear any ongoing physics simulations, ensuring a clean state for the next evaluation.

**Best Practices**:
- **Efficient Use of Supervisor Functions**: Minimize the use of Supervisor functions within loops to enhance performance.
- **Consistent Resetting**: Always reset both the position and physics to maintain consistency across evaluations.
- **Termination Handling**: Gracefully handle simulation termination by checking the return value of `supervisor.step(TIME_STEP)`.

---

# Conclusion

Supervisor Programming in Webots provides a robust framework for advanced simulation control and environment manipulation. By leveraging the `wb_supervisor_*` functions, developers can execute tasks that extend beyond the capabilities of standard robot controllers, such as centralized tracking, environment adjustments, and systematic evaluations. This guide has covered the foundational aspects of Supervisor Programming, including tracking and setting robot positions, illustrated through practical examples. As Webots continues to evolve, Supervisor Programming remains an essential tool for developers seeking comprehensive control over their robotic simulations.

For further information and detailed API references, consult the [Webots User Guide](https://cyberbotics.com/doc/reference/supervisor) and explore language-specific documentation to harness the full potential of Supervisor Programming in your projects.

