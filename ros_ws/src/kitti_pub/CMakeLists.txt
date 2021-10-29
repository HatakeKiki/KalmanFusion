cmake_minimum_required(VERSION 3.18)
project(kitti_pub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 3.4 REQUIRED)

###########
## Build ##
###########
include_directories(
  include
  ${RCLCPP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(${PROJECT_NAME} src/kitti_pub.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
  geometry_msgs
  darknet_ros_msgs
  pcl_conversion
)
target_link_libraries(
  ${PROJECT_NAME} 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
