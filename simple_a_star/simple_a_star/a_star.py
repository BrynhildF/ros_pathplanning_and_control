import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import numpy as np
from scipy.interpolate import splprep, splev
import heapq

class AStar(Node):
    def __init__(self):
        super().__init__('a_star_node')
        self.subscription = self.create_subscription(Int32MultiArray, 'raw_map', self.map_callback, 10)
        self.path_pub = self.create_publisher(Float32MultiArray, 'a_path', 10)

    def get_manhattan_distance(self, p1, p2):
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    # 新增剪枝，去掉多余节点 
    def prune_path(self, grid, path):
        if len(path) <= 2: return path
        pruned = [path[0]]
        curr = 0
        while curr < len(path) - 1:
            for i in range(len(path) - 1, curr, -1):
                if self.is_clear(grid, path[curr], path[i]):
                    pruned.append(path[i])
                    curr = i
                    break
        return pruned

    def is_clear(self, grid, p1, p2):
        points = np.linspace(p1, p2, 20)
        for pt in points:
            r, c = int(round(pt[0])), int(round(pt[1]))
            if grid[r][c] == 1: return False
        return True

    def a_star_algorithm(self, grid, start, end):
        rows, cols = grid.shape
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        
        while open_list:
            current_f, current = heapq.heappop(open_list)
            if current == end:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                    if grid[neighbor[0]][neighbor[1]] == 1: continue
                    cost = 1.414 if abs(dx) + abs(dy) == 2 else 1.0
                    tentative_g = g_score[current] + cost
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f = tentative_g + self.get_manhattan_distance(neighbor, end)
                        heapq.heappush(open_list, (f, neighbor))
        return None

    def map_callback(self, msg):
        grid = np.array(msg.data).reshape((10, 10))
        path = self.a_star_algorithm(grid, (0, 0), (9, 9))
        
        if path:
            #pruned_path = self.prune_path(grid, path)
            pruned_path = path
            try:
                x = [p[1] for p in pruned_path]
                y = [p[0] for p in pruned_path]
                tck, u = splprep([x, y], s=0.5, k=min(5, len(pruned_path)-1))
                u_new = np.linspace(0, 1, 50)
                new_points = splev(u_new, tck)
                
                msg = Float32MultiArray()
                combined = []
                for xi, yi in zip(new_points[0], new_points[1]):
                    combined.extend([float(xi), float(yi)])
                msg.data = combined
                self.path_pub.publish(msg)
            except:
                msg = Float32MultiArray()
                combined = []
                for p in path:
                    combined.append(float(p[1]))
                    combined.append(float(p[0]))
                msg.data = combined
                self.path_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AStar()
    rclpy.spin(node)
    rclpy.shutdown()
