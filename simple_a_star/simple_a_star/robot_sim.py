import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
import math

class RobotSim(Node):
    def __init__(self):
        super().__init__('robot_sim')
        self.pose_pub = self.create_publisher(Pose2D, 'robot_pose', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.x, self.y, self.theta = 0.5, 0.5, 0.0 # 起点
        self.v, self.w = 0.0, 0.0
        self.create_timer(0.02, self.update)

    def cmd_cb(self, msg):
        self.v, self.w = msg.linear.x, msg.angular.z

    def update(self):
        dt = 0.02
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt
        p = Pose2D()
        p.x, p.y, p.theta = self.x, self.y, self.theta
        self.pose_pub.publish(p)

def main():
    rclpy.init(); rclpy.spin(RobotSim()); rclpy.shutdown()