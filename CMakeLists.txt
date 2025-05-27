cmake_minimum_required(VERSION 3.0.2)
project(yoseen_thermal)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /home/ros/YoseenSDK/SDK
)

link_directories(
  /usr/lib # SDK .so 实际路径
)

catkin_package()

add_executable(yoseen_ros_node src/yoseen_ros_node.cpp)

target_link_libraries(yoseen_ros_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  YoseenSDK
  YoseenFfmpeg
)
