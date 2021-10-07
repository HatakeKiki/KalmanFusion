cmake_minimum_required(VERSION 3.0.2)
project(kitti_cpp)

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
)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs visualization_msgs geometry_msgs pcl_ros
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_INCLUDE_DIRS})

add_executable(kitti_node src/kitti_node.cpp src/data_utils.cpp src/GroundRemove.cpp src/Detection.cpp src/Tracking.cpp)
target_link_libraries(kitti_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

