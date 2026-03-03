import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Pose2D
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_sub = self.create_subscription(Float32MultiArray, 'a_path', self.path_callback, 10)
        self.pose_sub = self.create_subscription(Pose2D, 'robot_pose', self.pose_cb, 10)
        
        self.path = []
        self.target_idx = 0
        self.pose = None
        
        # 增量 PID 参数 (角速度)
        self.kp, self.ki, self.kd = 1.5, 0.05, 0.1
        self.e1, self.e2 = 0.0, 0.0
        self.w = 0.0

        self.create_timer(0.05, self.loop)

    def path_callback(self, msg):
        self.path = [(msg.data[i], msg.data[i+1]) for i in range(0, len(msg.data), 2)]
        self.target_idx = 0

    def pose_cb(self, msg): self.pose = msg

    def loop(self):
        if not self.path or not self.pose: return
        if self.target_idx >= len(self.path): 
            self.cmd_pub.publish(Twist()) # 停下
            return

        tx, ty = self.path[self.target_idx]
        error_dist = math.sqrt((tx-self.pose.x)**2 + (ty-self.pose.y)**2)
        
        if error_dist < 0.3: # 到达当前点，换下一个
            self.target_idx += 1
            return

        # 角度误差
        target_angle = math.atan2(ty - self.pose.y, tx - self.pose.x)
        e = target_angle - self.pose.theta
        e = math.atan2(math.sin(e), math.cos(e)) # 归一化

        # 增量 PID
        dw = self.kp*(e - self.e1) + self.ki*e + self.kd*(e - 2*self.e1 + self.e2)
        self.w += dw
        self.e2, self.e1 = self.e1, e

        msg = Twist()
        msg.linear.x = 0.7 # 固定线速度
        msg.angular.z = self.w
        self.cmd_pub.publish(msg)

def main():
    rclpy.init(); rclpy.spin(PIDController()); rclpy.shutdown()
