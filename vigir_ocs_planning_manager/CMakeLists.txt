cmake_minimum_required(VERSION 2.8.3)
project(vigir_ocs_planning_manager)

#find_package(Boost REQUIRED COMPONENTS system)

find_package(Boost REQUIRED system filesystem date_time program_options signals thread)
find_package(catkin REQUIRED COMPONENTS cmake_modules geometry_msgs vigir_teleop_planning_msgs roscpp)

find_package(Eigen REQUIRED)

catkin_package(
   INCLUDE_DIRS
     include
   LIBRARIES
   CATKIN_DEPENDS
     geometry_msgs
     vigir_teleop_planning_msgs
     roscpp
     vigir_ocs_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

#link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

SET(HEADERS
include/${PROJECT_NAME}/inv_reach_manager.h
)


SET(SOURCE
src/ocs_planning_manager_node.cpp
)


## Declare a cpp executable
add_executable(ocs_planning_manager_node ${SOURCE} ${HEADERS} )
target_link_libraries(ocs_planning_manager_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_dependencies(ocs_planning_manager_node ${catkin_EXPORTED_TARGETS})

#install executable
install( TARGETS
    ocs_planning_manager_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install the launch files.
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE  
)


