cmake_minimum_required(VERSION 3.8)
project(clik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(custom_mavlink REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Action.msg"
  "msg/PositionPub.msg"
  "srv/ManipulatorMode.srv"
  "srv/TrajOutMsg.srv"
  "srv/TrajSolverMsg.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

add_executable(clik_node
  src/main.cpp
  src/clik_main.cpp
  src/arm_uav_kinematics.cpp
  src/forward_kinematic.cpp
  src/Jacobian.cpp
  src/log4z.cpp
)

set_target_properties(clik_node PROPERTIES OUTPUT_NAME clik)

target_include_directories(clik_node PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(clik_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  builtin_interfaces
  mavros_msgs
  custom_mavlink
)

target_link_libraries(clik_node
  Eigen3::Eigen
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libosqp.so
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(clik_node "${cpp_typesupport_target}")

install(TARGETS clik_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch msg srv scripts lib
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
