cmake_minimum_required(VERSION 2.8.3)
project(model_particle_plugin)

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
  LIBRARIES ParticlePlugin
  CATKIN_DEPENDS
    message_runtime
    roscpp
    simulation_msgs
    gazebo_ros
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

include_directories(
  include
  ${ADDITIONAL_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_library(ModelParticlePlugin SHARED src/ModelParticlePlugin.cpp src/ModelParticlePlugin.h)
target_link_libraries(ModelParticlePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
