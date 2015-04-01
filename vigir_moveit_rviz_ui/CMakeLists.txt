cmake_minimum_required(VERSION 2.8.3)
project(vigir_moveit_rviz_ui)

find_package(Boost REQUIRED thread date_time system filesystem)


find_package(PkgConfig REQUIRED)

# Ogre
#pkg_check_modules(OGRE OGRE)
#link_directories( ${OGRE_LIBRARY_DIRS} )

# Qt Stuff
set(QT_USE_QTXML TRUE)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_visualization
  cmake_modules
  vigir_planning_interface
  rviz
)

find_package(Eigen REQUIRED)
		    
include_directories(SYSTEM
                    vigir_motion_planning_rviz_plugin/include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${QT_INCLUDE_DIR}
                    ${OGRE_INCLUDE_DIRS})		    


catkin_package(
  LIBRARIES
    moveit_ros_visualization
  INCLUDE_DIRS
    vigir_motion_planning_rviz_plugin/include
  CATKIN_DEPENDS
    moveit_ros_visualization vigir_planning_interface
    )
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
           
add_subdirectory(vigir_motion_planning_rviz_plugin)

install(FILES
  vigir_motion_planning_rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})           
                    
