cmake_minimum_required(VERSION 3.0.2)
project(my_pckg)

# Set the CMake module path to include the GeographicLib find module
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
find_package(gazebo REQUIRED)

find_package(GeographicLib REQUIRED)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  PoseSimple.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pckg
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
${GeographicLib_INCLUDE_DIRS}
)

add_executable(state_status_node src/state_status.cpp)
target_link_libraries(state_status_node ${catkin_LIBRARIES})

add_executable(state_control_node src/state_control.cpp)
target_link_libraries(state_control_node ${catkin_LIBRARIES})

add_executable(gps_handler_node src/gps_handler.cpp)
target_link_libraries(gps_handler_node ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})

add_executable(user_interface_node src/user_interface.cpp)
target_link_libraries(user_interface_node ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})

add_executable(location_scaler_node src/location_scaler.cpp)
target_link_libraries(location_scaler_node ${catkin_LIBRARIES})

add_executable(obstacle_avoidance_node src/obstacle_avoidance.cpp)
target_link_libraries(obstacle_avoidance_node ${catkin_LIBRARIES})