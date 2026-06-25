from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clik',
            executable='clik',
            name='clik',
            output='screen',
            parameters=[{
                'yaw_offset': 0.0,
                'x_offset': 0.0,
                'y_offset': 0.0,
                'z_offset': 0.0677,
                'x_min': -2.0,
                'x_max': 20.0,
                'y_min': -20.0,
                'y_max': 2.0,
                'z_min': 0.02,
                'z_max': 28.0,
                'vel_vertical_max': 0.2,
                'vel_horizion_max': 0.2,
                'acc_vertica_max': 2.0,
                'acc_horizion_max': 2.0,
                'delta_x_min': -0.08,
                'delta_x_max': 0.08,
                'delta_y_min': -0.08,
                'delta_y_max': 0.08,
                'delta_z_min': -0.300,
                'delta_z_max': -0.150,
                'delta_vel': 0.8,
                'delta_acc': 2.0,
            }],
        )
    ])
