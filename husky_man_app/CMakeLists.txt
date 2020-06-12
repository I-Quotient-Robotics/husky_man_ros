cmake_minimum_required(VERSION 3.0.2)
project(husky_man_app)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  roscpp
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  actionlib
  actionlib_msgs
  aruco_msgs
)

catkin_python_setup()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  brain_control_msg.msg
)

add_service_files(
  DIRECTORY
  srv
  FILES
  vision_grasp.srv
  PickGift.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)



catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES xfm_driver
  CATKIN_DEPENDS message_runtime rospy std_msgs
  #DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(marker_detect_node src/marker_detect_node.cpp)

add_dependencies(marker_detect_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(marker_detect_node
  ${catkin_LIBRARIES}
)


add_executable(box_detect_node src/box_detect_node.cpp)

add_dependencies(box_detect_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(box_detect_node
  ${catkin_LIBRARIES}
)
