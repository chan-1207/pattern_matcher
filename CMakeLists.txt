cmake_minimum_required(VERSION 3.8)
project(pattern_matcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  /opt/ros/humble/include
  /opt/ros/humble/include/pcl_conversions
)

add_definitions(${PCL_DEFINITIONS})

# Add the pattern_recorder executable
add_executable(pattern_recorder src/recorder.cpp)
ament_target_dependencies(pattern_recorder
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  laser_geometry
  ament_index_cpp
)
target_link_libraries(pattern_recorder ${PCL_LIBRARIES})

# Add the pattern_matcher executable
add_executable(pattern_matcher src/matcher.cpp)
ament_target_dependencies(pattern_matcher
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  laser_geometry
  ament_index_cpp
  visualization_msgs
)
target_link_libraries(pattern_matcher ${PCL_LIBRARIES})

# Add the navigator executable
add_executable(navigator src/navigator.cpp)
ament_target_dependencies(navigator
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav2_msgs
  rclcpp_action
)

# Install executables
install(TARGETS
  pattern_recorder
  pattern_matcher
  navigator
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install mesh files
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install PCD files
install(DIRECTORY
  pcd
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
