import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import numpy as np
import matplotlib.pyplot as plt
import heapq # 用于高效获取最小代价节点的优先队列

class AStarVisualizer(Node):
    def __init__(self):
        super().__init__('a_star_visualizer')
        self.subscription = self.create_subscription(Int32MultiArray, 'raw_map', self.map_callback, 10)
        
        plt.ion()
        self.fig, self.ax = plt.subplots()

    def get_manhattan_distance(self, p1, p2):
        # 启发式函数 h(n): 曼哈顿距离
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    def a_star_algorithm(self, grid, start, end):
        rows, cols = grid.shape
        # 优先队列存放: (f_score, (x, y))
        open_list = []
        heapq.heappush(open_list, (0, start))
        
        # 记录每个点是从哪个点过来的，用于回溯路径
        came_from = {}
        # 记录起点到当前点的实际代价 g_score
        g_score = {start: 0}
        
        while open_list:
            # 弹出当前 f_score 最小的点
            current_f, current = heapq.heappop(open_list)

            if current == end:
                # 找到终点，开始回溯路径
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1] # 返回翻转后的路径

            # 检查上下左右四个方向
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 检查边界和障碍物 (1为障碍物)
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                    if grid[neighbor[0]][neighbor[1]] == 1:
                        continue
                    
                    # A* 核心：计算新的 g_score
                    tentative_g_score = g_score[current] + 1
                    
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        # 发现更优路径
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score = tentative_g_score + self.get_manhattan_distance(neighbor, end)
                        heapq.heappush(open_list, (f_score, neighbor))
        
        return None # 没找到路径

    def map_callback(self, msg):
        # 1. 解析地图
        size = 10
        grid = np.array(msg.data).reshape((size, size))
        start = (0, 0)
        end = (size - 1, size - 1)

        self.get_logger().info('收到新地图，计算寻路中...')

        # 2. 运行 A* 算法
        path = self.a_star_algorithm(grid, start, end)

        # 3. 可视化
        self.ax.clear()
        # 画出地图 (0白1黑)
        self.ax.imshow(grid, cmap='Greys', origin='upper')
        
        if path:
            self.get_logger().info('找到路径！')
            # 提取路径的 x 和 y 坐标用于绘图
            path_x = [p[1] for p in path] # 注意：imshow 的列对应 x
            path_y = [p[0] for p in path] # 注意：imshow 的行对应 y
            self.ax.plot(path_x, path_y, color='red', linewidth=2, label='Path')
        else:
            self.get_logger().warn('没有找到可行路径')

        self.ax.plot(start[1], start[0], 'go', label='Start') # 起点绿色
        self.ax.plot(end[1], end[0], 'bo', label='End')     # 终点蓝色
        self.ax.legend()
        
        plt.draw()
        plt.pause(0.01)

def main():
    rclpy.init()
    rclpy.spin(AStarVisualizer())
    rclpy.shutdown()
