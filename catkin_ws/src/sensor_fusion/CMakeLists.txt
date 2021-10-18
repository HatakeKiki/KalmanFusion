cmake_minimum_required(VERSION 3.0.2)
project(sensor_fusion)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  pcl_ros
  darknet_ros_msgs
)
# find_package(OpenCV REQUIRED)
# find_package(PCL 1.7 REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
# link_directories(${OpenCV_INCLUDE_DIRS})

add_executable(sensor_fusion src/sensor_fusion.cpp src/GroundRemove.cpp src/detection_fusion.cpp src/Tracking.cpp)
target_link_libraries(sensor_fusion
  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#  ${OpenCV_LIBS}
)

