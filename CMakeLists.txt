cmake_minimum_required(VERSION 3.0.2)
project(Attachment-Model)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Action.msg
  Emotion.msg
  RobotPub.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  rospy
  std_msgs
  message_runtime
)

## Mark executable scripts (Python etc.) for installation
catkin_install_python(PROGRAMS
  src/utils.py
  src/subscribers.py
  src/actuators.py
  src/perception.py
  src/behaviors.py
  src/child_node.py
  src/parent_node.py
  src/central_control.py
  src/plot_graph.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
