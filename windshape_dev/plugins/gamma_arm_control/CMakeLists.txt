cmake_minimum_required(VERSION 3.16)
project(GammaArmControlPlugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(GZ_LINK_TARGETS "")

function(find_gz_target OUT_VAR BASE_NAME)
    set(_found_target "")
    foreach(_ver IN LISTS ARGN)
        set(_pkg "${BASE_NAME}${_ver}")
        find_package(${_pkg} QUIET)
        if(TARGET ${_pkg}::${_pkg})
            set(_found_target ${_pkg}::${_pkg})
            message(STATUS "Found ${_pkg}: target ${_found_target}")
            break()
        endif()
    endforeach()
    set(${OUT_VAR} "${_found_target}" PARENT_SCOPE)
endfunction()

# Common PX4 / Gazebo combinations:
#   Garden   -> gz-sim7
#   Harmonic -> gz-sim8
#   Ionic    -> gz-sim9
find_gz_target(GZ_SIM_TARGET       gz-sim       9 8 7)
find_gz_target(GZ_TRANSPORT_TARGET gz-transport 15 14 13 12 11)
find_gz_target(GZ_MSGS_TARGET      gz-msgs      11 10 9 8)
find_gz_target(GZ_COMMON_TARGET    gz-common    6 5 4)

# sdformat target name normally follows sdformat13::sdformat13, etc.
find_gz_target(SDFORMAT_TARGET     sdformat     15 14 13 12)

# gz-plugin has a special register target used by GZ_ADD_PLUGIN.
set(GZ_PLUGIN_REGISTER_TARGET "")
foreach(_ver 3 2)
    set(_pkg "gz-plugin${_ver}")
    find_package(${_pkg} QUIET COMPONENTS register)
    if(TARGET ${_pkg}::register)
        set(GZ_PLUGIN_REGISTER_TARGET ${_pkg}::register)
        message(STATUS "Found ${_pkg}: target ${GZ_PLUGIN_REGISTER_TARGET}")
        break()
    elseif(TARGET ${_pkg}::${_pkg})
        set(GZ_PLUGIN_REGISTER_TARGET ${_pkg}::${_pkg})
        message(STATUS "Found ${_pkg}: target ${GZ_PLUGIN_REGISTER_TARGET}")
        break()
    endif()
endforeach()

if(NOT GZ_SIM_TARGET)
    message(FATAL_ERROR
        "Cannot find gz-sim development package.\n"
        "Your code includes <gz/sim/System.hh>, so install the matching dev package first, for example:\n"
        "  sudo apt install libgz-sim7-dev      # Gazebo Garden / common PX4 Ubuntu 22.04 setup\n"
        "or:\n"
        "  sudo apt install libgz-sim8-dev      # Gazebo Harmonic\n"
        "Then delete build/ and run cmake again."
    )
endif()

list(APPEND GZ_LINK_TARGETS ${GZ_SIM_TARGET})
foreach(_target
    GZ_PLUGIN_REGISTER_TARGET
    GZ_TRANSPORT_TARGET
    GZ_MSGS_TARGET
    GZ_COMMON_TARGET
    SDFORMAT_TARGET
)
    if(${_target})
        list(APPEND GZ_LINK_TARGETS ${${_target}})
    endif()
endforeach()

find_package(orocos_kdl REQUIRED)

set(ROS_PREFIX "/opt/ros/$ENV{ROS_DISTRO}")
if(NOT EXISTS "${ROS_PREFIX}")
    set(ROS_PREFIX "/opt/ros/humble")
endif()

find_path(KDL_PARSER_INCLUDE_DIR
    NAMES kdl_parser/kdl_parser.hpp
    PATHS
        ${ROS_PREFIX}/include
        ${ROS_PREFIX}/include/kdl_parser
        /opt/ros/humble/include
        /opt/ros/humble/include/kdl_parser
)

find_library(KDL_PARSER_LIB
    NAMES kdl_parser
    PATHS
        ${ROS_PREFIX}/lib
        /opt/ros/humble/lib
)

if(NOT KDL_PARSER_INCLUDE_DIR OR NOT KDL_PARSER_LIB)
    message(FATAL_ERROR
        "Cannot find ROS2 kdl_parser. Please source ROS2 or install it:\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  sudo apt install ros-humble-kdl-parser"
    )
endif()

set(OROCOS_KDL_LINK "")
if(TARGET orocos-kdl)
    set(OROCOS_KDL_LINK orocos-kdl)
elseif(TARGET Orocos-KDL::orocos-kdl)
    set(OROCOS_KDL_LINK Orocos-KDL::orocos-kdl)
else()
    set(OROCOS_KDL_LINK ${orocos_kdl_LIBRARIES})
endif()

add_library(${PROJECT_NAME} SHARED
    GammaArmControl.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${orocos_kdl_INCLUDE_DIRS}
        ${KDL_PARSER_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${GZ_LINK_TARGETS}
        ${OROCOS_KDL_LINK}
        ${KDL_PARSER_LIB}
)

# We want the .so to be named libgamma_arm_control.so
set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_NAME gamma_arm_control
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
