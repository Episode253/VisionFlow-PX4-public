cmake_minimum_required(VERSION 3.8)
project(yolo_human_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(yolo_ros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(yolo_human_tracking src/yolo_human_tracking.cpp)
target_include_directories(yolo_human_tracking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(yolo_human_tracking PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(yolo_human_tracking
  rclcpp
  geometry_msgs
  nav_msgs
  mavros_msgs
  yolo_ros_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS yolo_human_tracking
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
