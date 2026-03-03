import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Pose2D
import matplotlib.pyplot as plt
import numpy as np

class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        self.create_subscription(Int32MultiArray, 'raw_map', self.map_cb, 10)
        self.create_subscription(Float32MultiArray, 'a_path', self.path_cb, 10)
        self.create_subscription(Pose2D, 'robot_pose', self.pose_cb, 10)
        self.grid, self.path, self.pose = None, None, None
        plt.ion()
        self.fig, self.ax = plt.subplots()

    def map_cb(self, msg): self.grid = np.array(msg.data).reshape((10, 10))
    def path_cb(self, msg): self.path = msg.data
    def pose_cb(self, msg): 
        self.pose = msg
        self.draw()

    def draw(self):
        if self.grid is None: return
        self.ax.clear()
        self.ax.imshow(self.grid, cmap='Greys', origin='lower') # origin设为lower匹配坐标系
        if self.path:
            self.ax.plot(self.path[0::2], self.path[1::2], 'r-', label='Smooth Path')
        if self.pose:
            self.ax.plot(self.pose.x, self.pose.y, 'go', markersize=8)
            self.ax.quiver(self.pose.x, self.pose.y, np.cos(self.pose.theta), np.sin(self.pose.theta), color='g')
        plt.draw()
        plt.pause(0.001)

def main():
    rclpy.init(); rclpy.spin(Visualizer()); rclpy.shutdown()