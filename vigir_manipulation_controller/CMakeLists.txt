cmake_minimum_required(VERSION 2.8.3)
project(vigir_manipulation_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  flor_control_msgs
  vigir_grasp_msgs
  vigir_ocs_msgs
  geometric_shapes
  moveit_core
  moveit_msgs
  roscpp
  urdf
  tf
  tf_conversions
  trajectory_msgs
  vigir_object_template_msgs
  vigir_planning_interface
  vigir_planning_msgs
  actionlib
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   control_msgs#   flor_control_msgs#   vigir_grasp_msgs#   vigir_ocs_msgs#   moveit_msgs#   trajectory_msgs#   vigir_object_template_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vigir_manipulation_controller
  CATKIN_DEPENDS actionlib flor_control_msgs vigir_grasp_msgs vigir_ocs_msgs geometric_shapes moveit_core moveit_msgs roscpp urdf tf tf_conversions trajectory_msgs vigir_object_template_msgs vigir_planning_interface vigir_planning_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

SET(HEADERS
include/${PROJECT_NAME}/vigir_manipulation_controller.h
)

SET(SOURCE
src/vigir_manipulation_controller.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCE} ${HEADERS} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} vigir_object_template_msgs_generate_messages_cpp)

## Declare a cpp executable
#add_executable(vigir_manipulation_controller_node ${SOURCE} ${HEADERS})
#add_dependencies(vigir_manipulation_controller_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(vigir_manipulation_controller_node  ${catkin_LIBRARIES})

#############
## Install ##
#############

install( TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### Mark executables and/or libraries for installation
#install(TARGETS
#   vigir_manipulation_controller_node
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install ( DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


