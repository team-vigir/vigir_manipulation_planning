cmake_minimum_required(VERSION 2.8.3)
project(vigir_moveit_utils)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  moveit_core
  moveit_msgs
  visualization_msgs
  vigir_teleop_planning_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
INCLUDE_DIRS include
#  LIBRARIES vigir_moveit_utils
CATKIN_DEPENDS moveit_core moveit_msgs visualization_msgs vigir_teleop_planning_msgs
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/  
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}  
  PATTERN "*.h"
)

