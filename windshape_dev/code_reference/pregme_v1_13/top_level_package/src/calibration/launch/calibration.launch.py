import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 启动本包的节点
    calibration_node = Node(
        package='calibration',
        executable='calibration_node',
        name='calibration',
        output='screen'
    )

    # 2. 嵌套引入 off_mission 的 launch (假设它已被迁移为 .launch.py)
    off_mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('off_mission'), 'launch', 'off_mission.launch.py')
        )
    )

    # 3. 嵌套引入 navigator 的 launch (假设它已被迁移为 .launch.py)
    navigator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('navigator'), 'launch', 'navigator.launch.py')
        )
    )

    return LaunchDescription([
        calibration_node,
        off_mission_launch,
        navigator_launch
    ])