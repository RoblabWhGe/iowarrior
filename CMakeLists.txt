cmake_minimum_required(VERSION 2.8.3)
project(iowarrior)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp message_generation std_msgs
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  iowarrior_getStatus.srv
  iowarrior_setOutput.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(iowarrior_node src/iowarrior_node.cpp)
add_dependencies(iowarrior_node iowarrior_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(iowarrior_node ${catkin_LIBRARIES})
