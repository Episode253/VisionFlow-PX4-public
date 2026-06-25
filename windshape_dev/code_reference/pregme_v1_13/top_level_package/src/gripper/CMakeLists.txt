cmake_minimum_required(VERSION 3.8)
project(gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(gripper src/gripper.cpp)
target_compile_features(gripper PUBLIC cxx_std_17)
ament_target_dependencies(gripper rclcpp std_msgs)

install(TARGETS gripper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
