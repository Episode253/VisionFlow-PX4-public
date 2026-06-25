cmake_minimum_required(VERSION 3.8)
project(qt_build)

find_package(ament_cmake REQUIRED)

install(FILES
  qt-ros.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY cmake/
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package(CONFIG_EXTRAS qt-ros.cmake)
