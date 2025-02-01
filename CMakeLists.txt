cmake_minimum_required(VERSION 3.0.2)
project(robot_status_tui)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  topic_tools
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  actionlib_msgs
  geographic_msgs
  message_generation
)

add_message_files(
  FILES
  Status.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  actionlib_msgs
  geographic_msgs
)

catkin_package(CATKIN_DEPENDS geometry_msgs geographic_msgs nav_msgs roscpp rospy std_msgs actionlib_msgs sensor_msgs message_runtime message_generation)

# Manually search for ncurses library
find_library(NCURSES_LIBRARY
  NAMES ncurses
  PATHS /usr/lib /usr/local/lib /lib
)

# Manually search for the ncurses.h header
find_path(NCURSES_INCLUDE_DIR
  NAMES ncurses.h
  PATHS /usr/include /usr/local/include
)

if(NCURSES_LIBRARY AND NCURSES_INCLUDE_DIR)
  message(STATUS "Found ncurses library: ${NCURSES_LIBRARY}")
  message(STATUS "Found ncurses include dir: ${NCURSES_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "Could not find ncurses. Install with: sudo apt-get install libncurses5-dev libncursesw5-dev")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${NCURSES_INCLUDE_DIR}
)

add_executable(robot_status_node
  src/robot_status_node.cpp
  src/status_tui.cpp
)
target_link_libraries(robot_status_node
  ${catkin_LIBRARIES}
  ${NCURSES_LIBRARY}
)

add_executable(random_publisher scripts/random_publisher.cpp)
target_link_libraries(random_publisher ${catkin_LIBRARIES})

