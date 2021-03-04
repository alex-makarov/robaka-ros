cmake_minimum_required(VERSION 2.8.3)
project(robaka)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  tf2
  actionlib
  actionlib_msgs
  rosserial_arduino
)

#generate_messages (
#  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
#)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_listener src/cartographer_tf_to_odom.cpp)
target_link_libraries(odom_listener ${catkin_LIBRARIES})
install(TARGETS odom_listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS
  src/display.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
