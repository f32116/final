cmake_minimum_required(VERSION 2.8.3)
project(gps_transformer)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  roslib
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3	 REQUIRED)
find_package(PCL REQUIRED COMPONENTS )
find_package(OpenCV REQUIRED)
find_package (GeographicLib REQUIRED)
add_definitions (${GeographicLib_DEFINITIONS})


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  src
  ${GeographicLib_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/gps_transformer.cpp
 )

 add_executable(gps_transformer_node src/gps_transformer.cpp)

 target_link_libraries(gps_transformer_node
   ${GeographicLib_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
 )

