cmake_minimum_required(VERSION 3.0.2)
project(uwb_localization_dwm)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  UWBrange.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(jackal_teleop_key src/telep_jackal_key.cpp)
target_link_libraries(jackal_teleop_key ${catkin_LIBRARIES})
add_dependencies(jackal_teleop_key ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  src/scripts/get_uwb_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)