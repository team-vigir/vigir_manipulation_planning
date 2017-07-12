cmake_minimum_required(VERSION 2.8.3)
project(vigir_lidar_octomap_updater)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_perception
  pluginlib
  roscpp
  laser_geometry
  laser_filters
  vigir_perception_msgs
  hector_nav_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/LidarOctomapUpdater.cfg)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vigir_lidar_octomap_updater
  CATKIN_DEPENDS moveit_ros_perception pluginlib roscpp vigir_perception_msgs hector_nav_msgs dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

set(MOVEIT_LIB_NAME vigir_lidar_octomap_updater)



add_library(${MOVEIT_LIB_NAME}_core src/lidar_octomap_updater.cpp)
target_link_libraries(${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(${MOVEIT_LIB_NAME}_core PROPERTIES LINK_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(${MOVEIT_LIB_NAME}_core ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
    lidar_octomap_updater_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
