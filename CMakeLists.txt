cmake_minimum_required(VERSION 3.0.2)
project(ase389)

## Compile as C++11, supported in ROS Melodic and newer
add_compile_options(-std=c++14)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
)

find_package(Eigen3 REQUIRED)
find_package(RBDL COMPONENTS URDFReader REQUIRED)

catkin_package()
  
## Your package locations should be listed before other locations
## Specify additional locations of header files
include_directories(
  include
  ${RBDL_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}/eigen3
)
  
add_executable(PupperController 
              src/PupperCplusplusController/PupperControl.cpp
              src/PupperCplusplusController/PupperWBC.cpp
)

target_link_libraries(PupperController 
    ${catkin_LIBRARIES} 
    ${RBDL_LIBRARY}
    ${RBDL_URDFReader_LIBRARY}
)
