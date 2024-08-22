cmake_minimum_required(VERSION 3.0.2)
project(kalman_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  turtlesim
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kalman_filter
  CATKIN_DEPENDS roscpp geometry_msgs turtlesim std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(kalman_filter src/kalman_filter.cpp)
target_link_libraries(kalman_filter ${catkin_LIBRARIES})

add_executable(noise_generater src/noise_generater.cpp)
target_link_libraries(noise_generater ${catkin_LIBRARIES})

add_executable(turtle_filter src/turtle_filter.cpp)
target_link_libraries(turtle_filter kalman_filter ${catkin_LIBRARIES})
