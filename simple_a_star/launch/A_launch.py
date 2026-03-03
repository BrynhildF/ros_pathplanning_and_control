from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 地图发布节点
        Node(
            package='simple_a_star',
            executable='map_pub',
            name='map_pub'
        ),
        # 2. A* 路径规划节点
        Node(
            package='simple_a_star',
            executable='a_star',
            name='a_star'
        ),
        # 3. PID 控制节点
        Node(
            package='simple_a_star',
            executable='pid_controller',
            name='pid_controller'
        ),
        # 4. 机器人仿真节点
        Node(
            package='simple_a_star',
            executable='robot_sim',
            name='robot_sim'
        ),
        # 5. 可视化节点
        Node(
            package='simple_a_star',
            executable='visualizer',
            name='visualizer'
        ),
    ])
