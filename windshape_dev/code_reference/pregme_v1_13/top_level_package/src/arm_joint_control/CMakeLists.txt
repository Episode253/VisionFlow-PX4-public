cmake_minimum_required(VERSION 3.8)
project(arm_joint_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(arm_joint_control src/arm_joint_control.cpp)
target_compile_features(arm_joint_control PUBLIC cxx_std_17)
target_link_libraries(arm_joint_control Eigen3::Eigen)
ament_target_dependencies(arm_joint_control rclcpp sensor_msgs)

install(TARGETS arm_joint_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
