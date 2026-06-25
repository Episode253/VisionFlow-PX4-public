cmake_minimum_required(VERSION 3.8)
project(sim2real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# 生成 msg 和 srv
# 注意：DEPENDENCIES 里面必须写上内置类型 builtin_interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperCmd.msg"
  "msg/PositionPub.msg"
  "msg/VelocityPub.msg"
  "srv/CmdMode.srv"
  DEPENDENCIES builtin_interfaces
)

include_directories(include)

# 添加可执行文件
add_executable(sim_to_real src/sim_to_real.cpp)

# 链接普通 ROS 2 库
ament_target_dependencies(sim_to_real
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  mavros_msgs
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
)

# 【关键】链接自定义生成的 msg 和 srv 库
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(sim_to_real "${cpp_typesupport_target}")

# 安装规则
install(TARGETS sim_to_real
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  GAMMA.urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()