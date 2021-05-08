cmake_minimum_required(VERSION 3.0.2)
project(ase389)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

# This line ensures we can find RBDL with the FindRBDL.cmake file in the source directory
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

# ROS dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

# Non-ROS dependencies
find_package(Eigen3 REQUIRED)
find_package(RBDL COMPONENTS URDFReader REQUIRED)
find_package(osqp REQUIRED)

# Set all our include paths to a single variable INCLUDES for ease of access
set(INCLUDES
  include 
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}/eigen3  
)

# Create a catkin package, allowing this project to be included in other projects
catkin_package(
  INCLUDE_DIRS ${INCLUDES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS RBDL Eigen3
)

# Create a library for our class to be accesible to other packages
add_library(ase389 
  src/PupperCplusplusController/PupperModel.cpp
  src/PupperCplusplusController/PupperWBC.cpp
)

# Link all necessary libraries to out library
target_link_libraries(${PROJECT_NAME}
  osqp::osqp 
  ${RBDL_LIBRARY} 
  ${RBDL_URDFReader_LIBRARY})
  
# Specify include directories
include_directories(
  ${INCLUDES}
)
  
# Create an executable file and link its dependencies
add_executable(PupperController 
  src/PupperCplusplusController/PupperControl.cpp
)
target_link_libraries(PupperController ${PROJECT_NAME})
