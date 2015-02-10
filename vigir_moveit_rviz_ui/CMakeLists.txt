cmake_minimum_required(VERSION 2.8.3)
project(vigir_moveit_rviz_ui)

find_package(Boost REQUIRED thread date_time system filesystem)

# Ogre
pkg_check_modules(OGRE OGRE)
link_directories( ${OGRE_LIBRARY_DIRS} )

# Qt Stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS

find_package(Eigen REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_visualization
)

include_directories(rviz_plugin_render_tools/include
                    robot_state_rviz_plugin/include
                    planning_scene_rviz_plugin/include
                    motion_planning_rviz_plugin/include
		    ${catkin_INCLUDE_DIRS})
		    
include_directories(SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${QT_INCLUDE_DIR}
                    ${OGRE_INCLUDE_DIRS})		    


catkin_package(
  LIBRARIES
    moveit_ros_visualization
  INCLUDE_DIRS
    motion_planning_rviz_plugin/include
  CATKIN_DEPENDS
    moveit_ros_visualization
    )
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  motion_planning_rviz_plugin/include
)

include_directories(SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${QT_INCLUDE_DIR}
                    ${OGRE_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
           
add_subdirectory(motion_planning_rviz_plugin)

install(FILES
  motion_planning_rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})           
                    
