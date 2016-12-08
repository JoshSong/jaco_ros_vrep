cmake_minimum_required(VERSION 2.8.3)
project(vrep_api)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES VrepApi
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS} include include/vrep_api
)

add_library(VrepApi
    src/extApi.c
    src/extApiPlatform.c
)
target_compile_definitions(VrepApi PRIVATE NON_MATLAB_PARSING MAX_EXT_API_CONNECTIONS=255)