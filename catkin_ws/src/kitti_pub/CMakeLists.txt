cmake_minimum_required(VERSION 3.18.0)

project(kitti_pub)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED
  COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  sensor_msgs
  message_generation
  geometry_msgs
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
  pcl_ros
  sensor_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# link_directories(${PCL_LIBRARY_DIRS})
# link_directories(${OpenCV_INCLUDE_DIRS})

add_executable(kitti_pub src/kitti_pub.cpp src/data_utils.cpp)

target_link_libraries(kitti_pub
  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#  ${OpenCV_LIBS}
)


