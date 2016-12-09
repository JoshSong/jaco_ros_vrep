cmake_minimum_required(VERSION 2.8.3)
project(jaco_control)

# Set make arguments
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  control_msgs
  actionlib
  vrep_api
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    control_msgs
    actionlib
    vrep_api
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_executable(vrep_interface src/VrepInterface.cpp)
target_link_libraries(vrep_interface ${catkin_LIBRARIES})

#add_executable(jaco_human_playback src/JacoHumanPlayback.cpp)
#target_link_libraries(jaco_human_playback ${catkin_LIBRARIES} )

#add_executable(moveit_interface src/MoveitInterface.cpp)
#target_link_libraries(moveit_interface ${catkin_LIBRARIES} )
