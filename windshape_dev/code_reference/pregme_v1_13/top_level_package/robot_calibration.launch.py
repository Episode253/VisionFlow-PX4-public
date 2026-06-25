import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# 导入通用的解析源
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ================= 1. 声明传入的参数 (相当于 ROS 1 的 <arg>) =================
    fcu_url_arg = DeclareLaunchArgument('fcu_url', default_value='udp://:14540@127.0.0.1:14557')
    gcs_url_arg = DeclareLaunchArgument('gcs_url', default_value='')
    tgt_system_arg = DeclareLaunchArgument('tgt_system', default_value='1')
    tgt_component_arg = DeclareLaunchArgument('tgt_component', default_value='1')
    log_output_arg = DeclareLaunchArgument('log_output', default_value='screen')
    fcu_protocol_arg = DeclareLaunchArgument('fcu_protocol', default_value='v2.0')
    respawn_mavros_arg = DeclareLaunchArgument('respawn_mavros', default_value='false')

    # 获取 mavros 包路径，用于加载 yaml 配置
    mavros_share_dir = get_package_share_directory('mavros')

    # ================= 2. 嵌套引入其他 Launch 文件 =================
    
    # 2.1 MAVROS Launch
    # 注意：ROS 2 官方的 mavros 默认 launch 文件名可能因安装版本而异（如 mavros.launch.py 或 node.launch.py）
    # 这里按照最标准的翻译，向子 launch 传递 LaunchConfiguration 变量
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(mavros_share_dir, 'launch', 'node.launch')
        ),
        launch_arguments={
            'pluginlists_yaml': os.path.join(mavros_share_dir, 'launch', 'px4_pluginlists.yaml'),
            'config_yaml': os.path.join(mavros_share_dir, 'launch', 'px4_config.yaml'),
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('tgt_system'),
            'tgt_component': LaunchConfiguration('tgt_component'),
            'log_output': LaunchConfiguration('log_output'),
            'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            'respawn_mavros': LaunchConfiguration('respawn_mavros'),
        }.items()
    )

    # 2.2 Off Mission Launch
    off_mission_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('off_mission'), 'launch', 'off_mission.launch.py')
        )
    )

    # 2.3 Navigator Launch
    navigator_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('navigator'), 'launch', 'navigator.launch.py')
        )
    )

    # ================= 3. 定义各个 Node =================

    kinematics_node = Node(
        package='kinematics_computing',
        executable='Joint_Talker',
        name='kinematics_talker',
        output='screen'
    )

    sim2real_node = Node(
        package='sim2real',
        executable='sim_to_real',
        name='sim2real'
    )

    # Joy 节点 (参数直接用 Python 字典写入)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[{
            'dev': '/dev/input/js0',
            'autorepeat_rate': 50.0
        }]
    )

    # Calibration 节点 
    # ⚠️ 关键点：我们在上一个步骤中为了解决 CMake 报错，将它改名为 calibration_node 了！
    calibration_node = Node(
        package='calibration',
        executable='calibration_node', 
        name='calibration',
        output='screen'
    )

    # ================= 4. 返回完整结构 =================
    return LaunchDescription([
        # 声明变量
        fcu_url_arg, gcs_url_arg, tgt_system_arg, tgt_component_arg,
        log_output_arg, fcu_protocol_arg, respawn_mavros_arg,
        
        # 启动 MAVROS
        mavros_launch,
        
        # 启动独立节点
        kinematics_node,
        sim2real_node,
        joy_node,
        calibration_node,
        
        # 启动其它 Launch
        off_mission_launch,
        navigator_launch
    ])