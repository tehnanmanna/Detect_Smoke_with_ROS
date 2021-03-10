cmake_minimum_required(VERSION 2.8.3)
project(gpu_laser_sensor_particle_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_ros
  gazebo_dev
  simulation_msgs
  message_generation
  roscpp
  std_msgs
  std_srvs
  roslib
)

find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS
  LIBRARIES GpuLaserSensorParticlePlugin
  CATKIN_DEPENDS
    message_runtime
    roscpp
    gazebo_ros
    simulation_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
include_directories(
  include
  ${ADDITIONAL_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(GpuLaserSensorParticlePlugin src/GpuLaserSensorParticlePlugin.cpp src/GpuLaserSensorParticlePlugin.h)
target_link_libraries(GpuLaserSensorParticlePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} gazebo_ros_gpu_laser)
