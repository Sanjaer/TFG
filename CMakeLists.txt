cmake_minimum_required(VERSION 2.8.3)
project(tfg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable

add_executable(mano src/mano.cpp)

## Add cmake target dependencies of the executable
## same as for the library above

add_dependencies(mano mano_generate_messages_cpp)

# Specify libraries to link a library or executable target against

target_link_libraries(mano ${catkin_LIBRARIES})



