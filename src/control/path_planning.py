import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point

class PathPlanner:
    def __init__(self):
        self.path_pub = rospy.Publisher('/amr/path', Path, queue_size=10)
    
    def plan(self, start, goal, obstacles):
        # A* path planning algorithm
        def heuristic(a, b):
            return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
        
        def get_neighbors(current):
            neighbors = []
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < grid_size[0] and 0 <= neighbor[1] < grid_size[1] and grid[neighbor] == 0:
                    neighbors.append(neighbor)
            return neighbors
        
        grid_size = (100, 100)
        grid = np.zeros(grid_size, dtype=np.int8)
        
        for obs in obstacles:
            grid[obs[0], obs[1]] = 1
        
        open_list = [(start, 0, heuristic(start, goal))]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        while open_list:
            current = min(open_list, key=lambda x: x[2])[0]
            
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                self.publish_path(path)
                return path
            
            open_list = [x for x in open_list if x[0] != current]
            
            for neighbor in get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    open_list.append((neighbor, tentative_g_score, f_score[neighbor]))
        
        return []
    
    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'
        path_msg.poses = [PoseStamped(pose=Pose(position=Point(x, y, 0))) for x, y in path]
        self.path_pub.publish(path_msg)
