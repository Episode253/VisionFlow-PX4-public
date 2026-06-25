cmake_minimum_required(VERSION 3.8)
project(calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(Boost REQUIRED)

# ================= 1. 生成消息目标 (目标名即为 calibration) =================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Action.msg"
  "msg/PositionPub.msg"
  "srv/ManipulatorMode.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

# Eigen3 配置
find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

# ================= 2. 节点可执行文件 (改名为 calibration_node 避免重名) =================
add_executable(calibration_node
  src/main.cpp
  src/clik_main.cpp
)

# 链接普通依赖
ament_target_dependencies(calibration_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  gazebo_msgs
  Boost
)

# 链接 Eigen3
target_link_libraries(calibration_node Eigen3::Eigen)

# 链接自动生成的 msg/srv 库 (修复了被弃用的宏警告)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(calibration_node "${cpp_typesupport_target}")

# ================= 3. 安装配置 =================
install(TARGETS calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()