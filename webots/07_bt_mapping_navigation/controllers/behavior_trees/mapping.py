import py_trees
import numpy as np
from scipy import signal
from matplotlib import pyplot as plt

def world2map(xw, yw):
    """Convert world coordinates to map pixel coordinates.
    
    Args:
        xw (float): X coordinate in world frame
        yw (float): Y coordinate in world frame
        
    Returns:
        list: [px, py] Pixel coordinates in map frame (0-199, 0-299)
    """
    # Improved boundary handling using np.clip
    px = np.clip(int((xw + 2.25) * 40), 0, 199)
    py = np.clip(int((yw - 2) * (-50)), 0, 299)
    return [px, py]

def map2world(px, py):
    """Convert map pixel coordinates to world coordinates.
    
    Args:
        px (int): X pixel coordinate in map frame
        py (int): Y pixel coordinate in map frame
        
    Returns:
        list: [xw, yw] Coordinates in world frame
    """
    xw = px/40 - 2.25
    yw = py/(-50) + 2
    return [xw, yw]

class Mapping(py_trees.behaviour.Behaviour):
    """Behavior tree node that handles robot mapping functionality.
    
    This class creates and updates an occupancy grid map using LIDAR readings.
    It converts LIDAR readings to world coordinates and maintains both a regular
    occupancy grid and a configuration space representation accounting for robot size.
    
    Args:
        name (str): Name of the behavior node
        blackboard (Blackboard): Shared blackboard for data access
    
    Attributes:
        map (ndarray): Occupancy grid map of size 200x300
        MAP_SIZE (tuple): Size of the map in pixels (200, 300)
        ROBOT_RADIUS (int): Robot radius in pixels for configuration space
        LIDAR_TRIM (int): Number of LIDAR readings to trim from each end
        OCCUPANCY_INCREMENT (float): Increment value for occupied cells
        MAX_OCCUPANCY (float): Maximum occupancy probability value
    """
    def __init__(self, name, blackboard):
        super(Mapping, self).__init__(name)
        self.blackboard = blackboard
        self.hasrun = False
        self.robot = blackboard.read('robot')
        # Add constants as class attributes
        self.MAP_SIZE = (200, 300)
        self.ROBOT_RADIUS = 13  # Half of 26x26 convolution kernel
        self.LIDAR_TRIM = 80    # Number of readings to trim from each end
        self.OCCUPANCY_INCREMENT = 0.01
        self.MAX_OCCUPANCY = 1.0

    def setup(self):
        """Initialize robot sensors and devices.
        
        Sets up GPS, compass, LIDAR, and display devices with appropriate timesteps
        and enables required functionalities.
        """
        self.timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        self.display = self.robot.getDevice('display')

    def initialise(self):
        """Initialize the occupancy grid map and LIDAR parameters.
        
        Creates an empty occupancy grid and precomputes LIDAR angles for efficiency.
        The angles are stored after trimming specified number of readings from ends.
        """
        self.map = np.zeros(self.MAP_SIZE)
        # Precompute LIDAR angles once
        total_readings = 667
        angle_range = 4.19
        self.angles = np.linspace(angle_range/2, -angle_range/2, total_readings)
        self.angles = self.angles[self.LIDAR_TRIM:-self.LIDAR_TRIM]

    def update(self):
        """Update the occupancy grid map based on current LIDAR readings.
        
        Gets current robot pose from GPS and compass, transforms LIDAR readings
        to world coordinates, and updates the occupancy grid probabilities.
        Also visualizes the current map state on the robot's display.
        
        Returns:
            Status.RUNNING: Continuous mapping behavior
        """
        self.hasrun = True
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])

        px, py = world2map(xw, yw)
        self.display.setColor(0xFF0000)
        self.display.drawPixel(px, py)

        W_T_R = np.array([[np.cos(theta), -np.sin(theta), xw],
                          [np.sin(theta), np.cos(theta), yw],
                          [0, 0, 1]])

        # Read and preprocess range images
        ranges = np.array(self.lidar.getRangeImage())
        ranges = ranges[self.LIDAR_TRIM:-self.LIDAR_TRIM]
        ranges[ranges == np.inf] = 100
        
        # Vectorized point cloud transformation
        X_i = np.vstack([
            ranges * np.cos(self.angles),
            ranges * np.sin(self.angles),
            np.ones_like(ranges)
        ])
        world_points = W_T_R @ X_i

        # Update occupancy grid
        for point in world_points.T:
            px, py = world2map(point[0], point[1])
            self.map[px, py] = np.minimum(
                self.map[px, py] + self.OCCUPANCY_INCREMENT,
                self.MAX_OCCUPANCY
            )
            # Convert occupancy to grayscale color
            v = int(self.map[px, py] * 255)
            color = v * (256**2 + 256 + 1)
            self.display.setColor(int(color))
            self.display.drawPixel(px, py)

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Clean up and save the final map when behavior terminates.
        
        Args:
            new_status: New status the behavior is transitioning to
            
        Creates and saves the configuration space representation of the map
        accounting for robot size using convolution.
        """
        if self.hasrun:
            # Calculate configuration space using robot radius
            kernel = np.ones((2*self.ROBOT_RADIUS, 2*self.ROBOT_RADIUS))
            cspace = signal.convolve2d(self.map, kernel, mode='same')
            np.save('cspace', cspace)
            
            # Visualize the map
            # plt.figure(0)
            # plt.imshow(cspace)
            # plt.show()
