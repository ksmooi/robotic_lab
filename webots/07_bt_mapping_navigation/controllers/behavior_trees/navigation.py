import py_trees
import numpy as np

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
