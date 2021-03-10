cmake_minimum_required(VERSION 2.8.3)
project(mock_vision)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_msgs
  roscpp
  roslib
  geometry_msgs
  std_srvs
  tf
  message_generation
  std_msgs
  observer
#  imes_3d_vision
#  luh_youbot_gripper
#  moving_object_tracker
)

find_package(gazebo REQUIRED)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES gazebo_mock_vision_plugin
  DEPENDS
    roscpp
    gazebo_ros
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  include
  ${ADDITIONAL_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(gazebo_mock_vision_plugin src/mock_vision_camera_plugin.cpp src/mock_vision_camera_plugin.h)
target_link_libraries(gazebo_mock_vision_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Declare a C++ executable
#add_executable(mock_vision_node src/gazebo_mock_vision_old.cpp)
#add_dependencies(mock_vision_node gazebo_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(mock_vision_node ${catkin_LIBRARIES})
