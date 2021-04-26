cmake_minimum_required(VERSION 3.0.2)
project(assignment1)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  assignment1_setup
  gazebo_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  kalman_filter.srv
  pid_algorithm.srv
  sonar_wrapper.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   gazebo_msgs
   geometry_msgs
   std_msgs
   assignment1_setup
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES assignment1
#  CATKIN_DEPENDS gazebo_msgs geometry_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
    include  ${catkin_INCLUDE_DIRS}
)

add_executable(assignment1_controller src/controller.cpp)
target_link_libraries(assignment1_controller ${catkin_LIBRARIES})
add_dependencies(assignment1_controller assignment1_generate_messages_cpp)

add_executable(assignment1_kalmanFilter src/kalmanFilter.cpp)
target_link_libraries(assignment1_kalmanFilter ${catkin_LIBRARIES})
add_dependencies(assignment1_kalmanFilter assignment1_generate_messages_cpp)

add_executable(assignment1_pidAlgorithm.cpp src/pidAlgorithm.cpp)
target_link_libraries(assignment1_pidAlgorithm.cpp ${catkin_LIBRARIES})
add_dependencies(assignment1_pidAlgorithm.cpp assignment1_generate_messages_cpp)

add_executable(assignment1_sonarWrapper src/sonarWrapper.cpp)
target_link_libraries(assignment1_sonarWrapper ${catkin_LIBRARIES})
add_dependencies(assignment1_sonarWrapper assignment1_generate_messages_cpp)

add_executable(assignment1_kalman_controller src/controller.cpp)
target_compile_definitions(assignment1_kalman_controller PUBLIC -DNOISY_SONAR)
target_link_libraries(assignment1_kalman_controller ${catkin_LIBRARIES})
add_dependencies(assignment1_kalman_controller assignment1_generate_messages_cpp)
