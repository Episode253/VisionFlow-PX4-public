import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# 导入通用的解析源
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ================= 1. 声明传入的参数 (Launch Arguments) =================
    # 对应原 XML 中的 <arg name="..." default="..."/>
    mav_name_arg = DeclareLaunchArgument('mav_name', default_value='iris')
    fcu_url_arg = DeclareLaunchArgument('fcu_url', default_value='udp://:14540@127.0.0.1:14557')
    gcs_url_arg = DeclareLaunchArgument('gcs_url', default_value='')
    tgt_system_arg = DeclareLaunchArgument('tgt_system', default_value='1')
    tgt_component_arg = DeclareLaunchArgument('tgt_component', default_value='1')
    command_input_arg = DeclareLaunchArgument('command_input', default_value='2')
    gazebo_simulation_arg = DeclareLaunchArgument('gazebo_simulation', default_value='false')
    visualization_arg = DeclareLaunchArgument('visualization', default_value='true')
    log_output_arg = DeclareLaunchArgument('log_output', default_value='screen')
    fcu_protocol_arg = DeclareLaunchArgument('fcu_protocol', default_value='v2.0')
    respawn_mavros_arg = DeclareLaunchArgument('respawn_mavros', default_value='false')

    # ================= 2. 嵌套引入其他的 Launch 文件 =================
    
    # 2.1 MAVROS Launch
    mavros_share_dir = get_package_share_directory('mavros')
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

    # 2.2 Clik Launch
    clik_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('clik'), 'launch', 'clik.launch.py')
        )
    )

    # 2.3 保留你原代码中注释掉的 Launch 备用
    off_mission_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('off_mission'), 'launch', 'off_mission.launch.py')
        )
    )
    navigator_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('navigator'), 'launch', 'navigator.launch.py')
        )
    )

    # 2.4 Refactored (重构后) 的 Launch 文件
    # off_mission_refactored_launch = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('off_mission_refactored'), 'launch', 'off_mission_refactored.launch.py')
    #     )
    # )
    
    # navigator_refactored_launch = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('navigator_refactored'), 'launch', 'navigator_refactored.launch.py')
    #     )
    # )

    # ================= 3. URDF 解析与 Robot State Publisher 节点 =================
    
    # 获取 sim2real 包的共享安装路径
    sim2real_share_dir = get_package_share_directory('sim2real')
    
    # 拼接 URDF 的完整路径 (因为你直接放在了包的根目录，所以直接连文件名)
    urdf_path = os.path.join(sim2real_share_dir, 'GAMMA.urdf')

    # 检查文件是否存在（防止没 colcon build 导致找不到文件）
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"找不到 URDF 文件，请检查是否执行了 colcon build 且配置了 CMakeLists: {urdf_path}")

    # 读取 URDF 文件内容转换为字符串
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 创建 RSP 节点
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'publish_frequency': 100.0, # TF 树发布频率，与 Gazebo 状态更新频率匹配
        }]
    )

    # ================= 4. 定义各个独立节点 (Nodes) =================

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

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[{
            'dev': '/dev/input/js0',
            'autorepeat_rate': 50.0
        }]
    )

    # ================= 5. 拼装并返回完整的 Launch 描述 =================
    return LaunchDescription([
        # 5.1 声明的所有参数变量
        mav_name_arg, fcu_url_arg, gcs_url_arg, tgt_system_arg, tgt_component_arg,
        command_input_arg, gazebo_simulation_arg, visualization_arg,
        log_output_arg, fcu_protocol_arg, respawn_mavros_arg,
        
        # 5.2 启动的外部 Launch
        mavros_launch,
        clik_launch,
        off_mission_launch,
        navigator_launch,

        # 5.3 启动的具体节点
        rsp_node,           # <--- 新增的 Robot State Publisher 节点
        kinematics_node,
        sim2real_node,
        joy_node
    ])