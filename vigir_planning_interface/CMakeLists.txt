cmake_minimum_required(VERSION 2.8.3)
project(vigir_planning_interface)

set(CMAKE_BUILD_TYPE Debug)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

find_package(Boost REQUIRED COMPONENTS
  date_time
  filesystem
  program_options
  python
  system
  thread
)
find_package(PythonLibs 2.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  moveit_ros_warehouse
  moveit_ros_manipulation
  moveit_ros_move_group
  eigen_conversions
  tf_conversions
  tf
  roscpp
  actionlib
  rospy
  rosconsole
  cmake_modules
  vigir_planning_msgs
)
find_package(Eigen REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
  py_bindings_tools/include
  common_planning_interface_objects/include
  planning_scene_interface/include
  vigir_move_group_interface/include
)

#catkin_python_setup()

catkin_package(
  LIBRARIES
    moveit_common_planning_interface_objects
    moveit_planning_scene_interface
    vigir_move_group_interface
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_ros_planning
    moveit_ros_warehouse 
    moveit_ros_manipulation
    moveit_ros_move_group
    vigir_planning_msgs
    )

include_directories(${THIS_PACKAGE_INCLUDE_DIRS} 
                    ${catkin_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
		    ${Boost_INCLUDE_DIRS}
                    ${PYTHON_INCLUDE_DIRS})
		  
link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#add_subdirectory(py_bindings_tools)
#add_subdirectory(common_planning_interface_objects)
#add_subdirectory(planning_scene_interface)
add_subdirectory(vigir_move_group_interface)
#add_subdirectory(robot_interface)

