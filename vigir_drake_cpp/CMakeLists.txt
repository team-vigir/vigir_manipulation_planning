cmake_minimum_required(VERSION 2.8.3)
project(vigir_drake_cpp)

#set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp  
  vigir_planning_msgs
)

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
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs  # Or other packages containing msgs
#)

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
#  INCLUDE_DIRS include
#  LIBRARIES vigir_drake_cpp
CATKIN_DEPENDS 
  vigir_planning_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(POD_NAME drake_planner_cpp)

if(DEFINED ENV{VIGIR_ROOT_DIR})
  set(DRAKE_DIR $ENV{VIGIR_ROOT_DIR}/drake-distro/drake)  
  add_definitions(-DATLAS_PLANNER)
elseif(DEFINED ENV{THOR_ROOT_DIR})
  set(DRAKE_DIR $ENV{THOR_ROOT_DIR}/drake-distro/drake)  
  add_definitions(-DTHOR_PLANNER)
else()
  message(FATAL_ERROR "Found neither VIGIR_ROOT_DIR nor THOR_ROOT_DIR environment variable. Unable to continue.")
endif()

message("-- Found Drake at location ${DRAKE_DIR}")

include(${DRAKE_DIR}/cmake/pods.cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

pods_find_pkg_config(eigen3 3.2)

if (eigen3_FOUND)
   pods_use_pkg_config_includes( eigen3 )
else()
   # then use the traditional approach to finding Eigen3
   find_package( Eigen3 3.2 )
   if (Eigen3_FOUND)
      set(eigen3_FOUND 1)
      include_directories( EIGEN3_INCLUDE_DIR )
   endif()
endif()


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${DRAKE_DIR}/pod-build/include/drake
)

LINK_DIRECTORIES(${DRAKE_DIR}/pod-build/lib)

get_directory_property(output INCLUDE_DIRECTORIES)
message(STATUS ${output})

## Declare a cpp executable
set(SOURCE_FILES
    src/drake_planner.cpp
    src/drake_planner_node.cpp
    src/planner_modules/planner_module.cpp
    src/planner_modules/position_ik_planner_module.cpp
    src/planner_modules/trajectory_planner_module.cpp
    src/planner_modules/cartesian_trajectory_planner_module.cpp
)

add_executable(vigir_drake_cpp_node ${SOURCE_FILES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(vigir_drake_cpp_node vigir_planning_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(vigir_drake_cpp_node
   ${catkin_LIBRARIES}
   drakeSplineGeneration
   drakeRigidBodyConstraint
   drakeIK
   drakeRBM
   drakePolynomial
   drakePiecewisePolynomial
 )
