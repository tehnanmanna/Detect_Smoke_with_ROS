cmake_minimum_required(VERSION 2.8.3)

project(gpu_laser_collision_plugin)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  roscpp
  roslib
  std_msgs
  std_srvs
  sensor_msgs
  message_generation
  simulation_msgs
)

find_package(gazebo REQUIRED)
find_package(yaml-cpp REQUIRED)




catkin_package(
  INCLUDE_DIRS
  LIBRARIES GpuLaserCollisionPlugin
  CATKIN_DEPENDS
    message_runtime
    roscpp
    simulation_msgs
  DEPENDS
    gazebo_msgs
    gazebo_ros
)


link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  ${YAML_CPP_LIBRARY_DIRS}
)

include_directories(
  include
  ${ADDITIONAL_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_library(GpuLaserCollisionPlugin src/GpuLaserCollisionPlugin.cpp src/GpuLaserCollisionPlugin.h)
target_link_libraries(GpuLaserCollisionPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
