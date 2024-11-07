from os.path import exists

import numpy as np
import py_trees
from py_trees.composites import Sequence, Parallel, Selector

from navigation import Navigation
from mapping import Mapping
from planning import Planning
from controller import Supervisor

# Constants
TIMESTEP = None  # Will be set during initialization
WAYPOINTS = [
    (0.614, -0.19), (0.77, -0.94), (0.37, -3.04), (-1.41, -3.39),
    (-1.40, -3.39), (-1.8, -1.46), (-1.44, 0.38), (0, 0)
]

class Blackboard:
    """A shared memory space for storing and accessing data between behaviors.
    
    The Blackboard class provides a simple key-value store that allows different
    nodes in the behavior tree to share information.
    """
    
    def __init__(self):
        self.data: dict = {}

    def write(self, key: str, value: any) -> None:
        """Write a value to the blackboard.
        
        Args:
            key (str): The identifier for storing the value
            value (any): The value to store
        """
        self.data[key] = value

    def read(self, key: str) -> any:
        """Read a value from the blackboard.
        
        Args:
            key (str): The identifier of the value to retrieve
            
        Returns:
            any: The value associated with the key, or None if not found
        """
        return self.data.get(key)

class DoesMapExist(py_trees.behaviour.Behaviour):
    """A behavior that checks if a map file exists.
    
    This behavior checks for the existence of 'cspace.npy' file and returns
    SUCCESS if found, FAILURE otherwise.
    """
    
    def update(self) -> py_trees.common.Status:
        """Execute the map existence check.
        
        Returns:
            Status: SUCCESS if map exists, FAILURE otherwise
        """
        if exists('cspace.npy'):
            print("Map already exists")
            return py_trees.common.Status.SUCCESS
        print("Map does not exist")
        return py_trees.common.Status.FAILURE

def initialize_robot() -> tuple[Supervisor, int]:
    """Initialize the robot and get the simulation timestep.
    
    Returns:
        tuple[Supervisor, int]: A tuple containing:
            - The initialized robot supervisor
            - The simulation timestep value
    """
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    return robot, timestep

def create_behavior_tree(blackboard: Blackboard) -> py_trees.trees.BehaviourTree:
    """Create and configure the behavior tree for robot navigation.
    
    Args:
        blackboard (Blackboard): The shared memory space for the behaviors
        
    Returns:
        BehaviourTree: The configured behavior tree ready for execution
    """
    tree = Sequence("Main", memory=True, children=[
        Selector("Does map exist?", children=[
            DoesMapExist("Test for map"),
            Parallel("Mapping", 
                    policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
                    children=[
                        Mapping("map the environment", blackboard),
                        Navigation("move around the table", blackboard)
                    ])
        ], memory=True),
        Planning("compute path to lower left corner", blackboard, (-1.46, -3.12)),
        Navigation("move to lower left corner", blackboard),
        Planning("compute path to sink", blackboard, (0.88, 0.09)),
        Navigation("move to sink", blackboard)
    ])
    tree.setup_with_descendants()
    return tree

def main():
    """Main entry point for the behavior tree-based navigation system.
    
    Initializes the robot, creates a blackboard with necessary data,
    constructs the behavior tree, and runs the main control loop.
    """
    # Initialize robot and blackboard
    robot, timestep = initialize_robot()
    blackboard = Blackboard()
    blackboard.write('robot', robot)
    blackboard.write('waypoints', np.concatenate((WAYPOINTS, np.flip(WAYPOINTS, 0)), axis=0))

    # Create and run behavior tree
    tree = create_behavior_tree(blackboard)
    while robot.step(timestep) != -1:
        tree.tick_once()

if __name__ == "__main__":
    main()
