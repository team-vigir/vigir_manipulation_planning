set(QT_USE_QTXML TRUE)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set( headers
  include/moveit/vigir_motion_planning_rviz_plugin/motion_planning_display.h
  include/moveit/vigir_motion_planning_rviz_plugin/motion_planning_frame.h
)

# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(MOC_SOURCES ${headers})

qt4_wrap_ui(UIC_FILES
  src/ui/motion_planning_rviz_plugin_frame.ui
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Plugin Source
set(SOURCE_FILES 
  src/motion_planning_frame.cpp
  src/motion_planning_frame_context.cpp
  src/motion_planning_frame_planning.cpp
  src/motion_planning_frame_objects.cpp
  src/motion_planning_frame_scenes.cpp
  src/motion_planning_frame_states.cpp
  src/motion_planning_frame_cartesian_trajectory.cpp
  src/motion_planning_display.cpp
  src/motion_planning_frame_manipulation.cpp
)

set(MOVEIT_LIB_NAME vigir_moveit_motion_planning_rviz_plugin)
add_library(${MOVEIT_LIB_NAME}_core ${SOURCE_FILES} ${MOC_SOURCES} ${UIC_FILES})
add_dependencies(${MOVEIT_LIB_NAME}_core vigir_planning_msgs_generate_messages_cpp)

target_link_libraries(${MOVEIT_LIB_NAME}_core
  #vigir_moveit_rviz_plugin_render_tools
  moveit_planning_scene_rviz_plugin_core
  ${catkin_LIBRARIES} ${OGRE_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES})

add_library(${MOVEIT_LIB_NAME} src/plugin_init.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
