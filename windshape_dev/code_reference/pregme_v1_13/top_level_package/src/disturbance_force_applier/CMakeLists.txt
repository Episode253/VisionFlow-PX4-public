cmake_minimum_required(VERSION 3.8)
project(disturbance_force_applier)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

add_executable(disturbance_force_applier src/disturbance_force_applier.cpp)
target_compile_features(disturbance_force_applier PUBLIC cxx_std_17)
ament_target_dependencies(disturbance_force_applier
  rclcpp
  geometry_msgs
  gazebo_msgs
)

install(TARGETS disturbance_force_applier
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
