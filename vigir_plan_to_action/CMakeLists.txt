cmake_minimum_required(VERSION 2.8.3)
project(vigir_plan_to_action)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(Boost REQUIRED system filesystem date_time program_options signals thread)
find_package(catkin REQUIRED COMPONENTS cmake_modules vigir_utilities geometry_msgs flor_planning_msgs vigir_ocs_msgs roscpp actionlib vigir_planning_msgs vigir_moveit_utils eigen_conversions moveit_core moveit_ros_planning)

find_package(Eigen REQUIRED)

catkin_package(
   INCLUDE_DIRS
   LIBRARIES
   CATKIN_DEPENDS
     vigir_utilities
     geometry_msgs
     flor_planning_msgs
     roscpp
     actionlib
     vigir_planning_msgs
     vigir_moveit_utils
     eigen_conversions
     moveit_core
     moveit_ros_planning
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

SET(HEADERS
)

SET(SOURCE
src/plan_to_action_node.cpp
)


## Declare a cpp executable
add_executable(plan_to_action_node ${SOURCE} ${HEADERS} )
target_link_libraries(plan_to_action_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
add_dependencies(plan_to_action_node ${catkin_EXPORTED_TARGETS})


#install executable
install( TARGETS
    plan_to_action_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install the launch files.
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE  
)
