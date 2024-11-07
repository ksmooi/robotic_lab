import py_trees
import numpy as np
from heapq import heappush, heappop
from mapping import world2map, map2world

class Planning(py_trees.behaviour.Behaviour):
    """A behavior tree node for path planning using A* algorithm.
    
    This class implements path planning functionality that takes the robot's current position
    and a goal position to generate an optimal path while avoiding obstacles using the A* algorithm.
    
    Attributes:
        blackboard: Shared data storage for behavior tree nodes
        robot: Reference to the robot controller
        goal: Target destination coordinates
        cspace: Configuration space map representing obstacles
    """
    
    def __init__(self, name, blackboard, goal):
        """Initialize the Planning behavior.
        
        Args:
            name: Name of the behavior node
            blackboard: Shared data storage for behavior tree nodes
            goal: Target destination coordinates (x, y)
        """
        super(Planning, self).__init__(name)
        self.blackboard = blackboard
        self.robot = blackboard.read('robot')
        self.goal = goal
        self.cspace = None  # Initialize cspace as class attribute
        
    def setup(self):
        """Set up the behavior by enabling required sensors."""
        self.timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
    def initialise(self):
        """Load the configuration space map from file.
        
        Returns:
            Status.SUCCESS if map loaded successfully, Status.FAILURE otherwise
        """
        # Load the configuration space map
        try:
            self.cspace = np.load('cspace.npy')
            return py_trees.common.Status.SUCCESS
        except (IOError, ValueError) as e:
            self.feedback_message = f"Map file error: {str(e)}"
            return py_trees.common.Status.FAILURE
            
    def heuristic(self, a, b):
        """Calculate Manhattan distance heuristic between two points.
        
        Args:
            a: Start point coordinates (x, y)
            b: End point coordinates (x, y)
            
        Returns:
            Manhattan distance between points a and b
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
    def is_valid_position(self, pos):
        """Check if a position is valid and traversable in the configuration space.
        
        Args:
            pos: Position coordinates (x, y) to check
            
        Returns:
            bool: True if position is valid and traversable, False otherwise
        """
        return (0 <= pos[0] < self.cspace.shape[0] and 
                0 <= pos[1] < self.cspace.shape[1] and
                self.cspace[pos] < 0.3)
        
    def get_neighbors(self, pos):
        """Get valid neighboring positions using 8-connectivity.
        
        Args:
            pos: Current position coordinates (x, y)
            
        Returns:
            list: List of valid neighboring positions
        """
        directions = [(0,1), (1,0), (0,-1), (-1,0), 
                     (1,1), (1,-1), (-1,1), (-1,-1)]  # 8-connectivity
        return [(pos[0] + dx, pos[1] + dy) for dx, dy in directions 
                if self.is_valid_position((pos[0] + dx, pos[1] + dy))]
        
    def get_movement_cost(self, current, next_pos):
        """Calculate movement cost between adjacent positions.
        
        Args:
            current: Current position coordinates (x, y)
            next_pos: Next position coordinates (x, y)
            
        Returns:
            float: 1.414 for diagonal movement, 1 for cardinal movement
        """
        return 1.414 if abs(next_pos[0]-current[0]) + abs(next_pos[1]-current[1]) == 2 else 1
        
    def plan_path(self, start, goal):
        """Plan a path using A* algorithm from start to goal position.
        
        Args:
            start: Start position in world coordinates (x, y)
            goal: Goal position in world coordinates (x, y)
            
        Returns:
            list: List of waypoints in world coordinates, or None if no path found
        """
        if self.cspace is None:
            return None
            
        start_map = tuple(world2map(start[0], start[1]))
        goal_map = tuple(world2map(goal[0], goal[1]))
        
        # Check if start or goal is in obstacle
        if not (self.is_valid_position(start_map) and self.is_valid_position(goal_map)):
            return None
            
        frontier = []
        heappush(frontier, (0, start_map))
        came_from = {start_map: None}
        cost_so_far = {start_map: 0}
        
        while frontier:
            current = heappop(frontier)[1]
            
            if current == goal_map:
                break
                
            for next_pos in self.get_neighbors(current):
                # Diagonal movement costs sqrt(2), otherwise 1
                new_cost = cost_so_far[current] + self.get_movement_cost(current, next_pos)
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal_map, next_pos)
                    heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current
        
        # Reconstruct path
        if goal_map not in came_from:
            return None
            
        path = []
        current = goal_map
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        
        # Convert path back to world coordinates
        world_path = []
        for p in path:
            wx, wy = map2world(p[0], p[1])
            world_path.append((wx, wy))
            
        return world_path
        
    def update(self):
        """Execute the planning behavior.
        
        Gets current robot position, plans path to goal, and writes waypoints to blackboard.
        
        Returns:
            Status.SUCCESS if path found and written to blackboard, Status.FAILURE otherwise
        """
        # Get current robot position
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        start = (xw, yw)
        
        # Plan path
        path = self.plan_path(start, self.goal)
        
        if path is None:
            self.feedback_message = "No path found"
            return py_trees.common.Status.FAILURE
            
        # Write path to blackboard for navigation
        self.blackboard.write('waypoints', np.array(path))
        self.feedback_message = "Path planned successfully"
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        """Cleanup when behavior finishes.
        
        Args:
            new_status: New status after termination
        """
        pass
