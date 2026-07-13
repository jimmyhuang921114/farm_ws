cmake_minimum_required(VERSION 3.8)
project(rog_map_vlp16_bringup)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rog_map REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(rog_map_vlp16_node src/rog_map_vlp16_node.cpp)
target_compile_definitions(
  rog_map_vlp16_node PRIVATE
  USE_ROS2
  ORIGIN_AT_CORNER
  ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../SUPER/rog_map/"
)
target_include_directories(rog_map_vlp16_node PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(
  rog_map_vlp16_node
  rclcpp
  rog_map
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  pcl_conversions
)
target_link_libraries(rog_map_vlp16_node ${PCL_LIBRARIES} yaml-cpp)

install(TARGETS rog_map_vlp16_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
