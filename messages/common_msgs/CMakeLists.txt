cmake_minimum_required(VERSION 3.0.2)
project(common_msgs)

find_package(catkin REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
)

add_message_files(DIRECTORY msg
  FILES
    GpsInfo.msg
    ChassisState.msg
    ChassisControl.msg
    VirtualVehicleState.msg
    ControlState.msg
    Trigger.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
