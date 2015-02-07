cmake_minimum_required(VERSION 2.8.3)
project(vigir_plan_execution)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_msgs
  moveit_ros_move_group
  roscpp
  vigir_planning_msgs
  vigir_moveit_utils
  eigen_conversions
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES vigir_plan_execution
   CATKIN_DEPENDS moveit_msgs moveit_ros_move_group roscpp vigir_planning_msgs vigir_moveit_utils eigen_conversions
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vigir_plan_execution
  src/continuous_plan_execution.cpp
  )

target_link_libraries(vigir_plan_execution ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(vigir_plan_execution ${catkin_EXPORTED_TARGETS})

install(TARGETS vigir_plan_execution
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION include)
