import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import numpy as np

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        self.publisher = self.create_publisher(Int32MultiArray, 'raw_map', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.size = 10 # 10x10 的小地图，适合新手调试

    def timer_callback(self):
        msg = Int32MultiArray()
        # 生成 10x10 随机地图：0是路，1是墙
        # 概率：80%是路，20%是墙
        grid = np.random.choice([0, 1], size=(self.size * self.size), p=[0.8, 0.2])
        grid[0] = 0      # 左上角起点必为路
        grid[-1] = 0     # 右下角终点必为路
        
        msg.data = grid.tolist()
        self.publisher.publish(msg)
        self.get_logger().info('已发布 10x10 原始随机地图')

def main():
    rclpy.init()
    rclpy.spin(MapPublisher())
    rclpy.shutdown()
