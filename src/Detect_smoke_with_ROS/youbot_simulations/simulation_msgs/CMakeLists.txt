cmake_minimum_required(VERSION 2.8.3)
project(simulation_msgs)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
  geometry_msgs
)

add_service_files(DIRECTORY srv FILES
  SetIntVal.srv
  SetDoubleVal.srv
  ColorSwap.srv
  LaserParticle.srv
  Material.srv
  DelMaterial.srv
  Scanner.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

