cmake_minimum_required(VERSION 3.0.2)
project(me5413_world)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  rviz
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  gazebo_ros
  jsk_rviz_plugins
  jackal_gazebo
  jackal_navigation
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/path_publisher.cfg
  cfg/path_tracker.cfg
  cfg/lqr_tracker.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES me5413_world
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs dynamic_reconfigure
  DEPENDS system_lib EIGEN3 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

# Add Nodes
add_executable(path_publisher_node src/path_publisher_node.cpp)
target_link_libraries(path_publisher_node ${catkin_LIBRARIES})
add_dependencies(path_publisher_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(path_tracker_node src/path_tracker_node.cpp)
target_link_libraries(path_tracker_node ${catkin_LIBRARIES})
add_dependencies(path_tracker_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(lqr_tracker_node src/lqr_tracker_node.cpp)
target_link_libraries(lqr_tracker_node ${catkin_LIBRARIES})
add_dependencies(lqr_tracker_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

catkin_install_python(PROGRAMS
  scripts/lqr_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)