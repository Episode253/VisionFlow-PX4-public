from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripper',
            executable='gripper',
            name='gripper',
            output='screen',
            parameters=[{
                'target_ip': '192.168.50.199',
                'target_port': 6003,
            }],
        ),
    ])
