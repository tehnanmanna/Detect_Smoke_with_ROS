cmake_minimum_required(VERSION 2.8.3)
project(camera_effect_plugin)

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
  sensor_msgs
  std_srvs
  image_transport
  cv_bridge
  message_generation
)

find_package(gazebo REQUIRED)

set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/")
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

add_service_files(DIRECTORY srv FILES
  SetIntVal.srv
  SetDoubleVal.srv
  ColorSwap.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES CameraEffectPlugin
  DEPENDS
    gazebo_msgs
    gazebo_ros
)



#generate_messages(DEPENDENCIES
#  std_msgs
#  )

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
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


add_library(CameraEffectPlugin
  src/CameraEffectPlugin.cpp
  src/CameraEffectPlugin.h)
target_link_libraries(CameraEffectPlugin 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBS}
)
