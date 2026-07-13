cmake_minimum_required(VERSION 3.5)
project(fdilink_ahrs)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# interfaces
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# thirdpart
find_package(serial REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ahrs_driver_node
  src/ahrs_driver.cpp
  src/crc_table.cpp
)
ament_target_dependencies(ahrs_driver_node
  serial
  rclcpp
  tf2_ros
  tf2
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  ament_index_cpp
  tf2_geometry_msgs
)

add_executable(imu_tf_node
  src/imu_tf.cpp)
ament_target_dependencies(imu_tf_node
  serial
  rclcpp
  tf2_ros
  tf2
  std_msgs
  sensor_msgs
  tf2_geometry_msgs
  ament_index_cpp
)

install(
  TARGETS
    ahrs_driver_node
    imu_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
