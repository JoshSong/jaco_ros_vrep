cmake_minimum_required(VERSION 2.8.3)
project(jaco_control)

# Set make arguments
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  control_msgs
  visualization_msgs
  actionlib
  vrep_api
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  kinova_msgs
  kinova_driver
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    roslib
    sensor_msgs
    control_msgs
    visualization_msgs
    actionlib
    vrep_api
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    kinova_msgs
    kinova_driver
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_library(${PROJECT_NAME}
    src/VrepInterface.cpp
    src/JacoRos.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(JacoRos
    src/JacoRos.cpp
)
target_link_libraries(JacoRos ${catkin_LIBRARIES})

add_executable(vrep_interface src/VrepInterface.cpp)
target_link_libraries(vrep_interface ${catkin_LIBRARIES})

add_executable(vrep_ik_interface src/VrepIKInterface.cpp)
target_link_libraries(vrep_ik_interface ${catkin_LIBRARIES})

add_executable(force_control src/ForceControl.cpp)
target_link_libraries(force_control ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(jaco_human_playback src/JacoHumanPlayback.cpp)
#target_link_libraries(jaco_human_playback ${catkin_LIBRARIES} )

#add_executable(moveit_interface src/MoveitInterface.cpp)
#target_link_libraries(moveit_interface ${catkin_LIBRARIES} )

add_executable(enable_torque src/EnableTorque.cpp)
target_link_libraries(enable_torque ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(gravity_z_estimation src/GravityZEstimation.cpp)
target_link_libraries(gravity_z_estimation ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
