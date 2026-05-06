import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    tracker_node = Node(
        package='yolo_human_tracking',
        executable='yolo_human_tracking',
        name='yolo_human_tracking',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        tracker_node
    ])
