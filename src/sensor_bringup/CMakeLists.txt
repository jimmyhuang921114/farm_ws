cmake_minimum_required(VERSION 3.8)
project(sensor_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    sensor_bringup/open_camera.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME open_camera
)

install(
  PROGRAMS
    sensor_bringup/imu_transform.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_transform
)

install(
  PROGRAMS
    sensor_bringup/livox_cloud_monitor.py
    sensor_bringup/livox_time_sync_monitor.py
    sensor_bringup/glim_session_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(livox_cloud_guard src/livox_cloud_guard.cpp)
ament_target_dependencies(
  livox_cloud_guard rclcpp sensor_msgs diagnostic_msgs std_msgs std_srvs)
install(TARGETS livox_cloud_guard DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # This checkout may contain preserved nested build/install/log artifacts
  # from an earlier package-local colcon invocation.  Give each linter an
  # explicit source list so generated files are never treated as source.
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(python_sources
    sensor_bringup/__init__.py
    sensor_bringup/imu_transform.py
    sensor_bringup/imu_visualizer.py
    sensor_bringup/glim_session_supervisor.py
    sensor_bringup/livox_cloud_monitor.py
    sensor_bringup/livox_time_sync_monitor.py
    sensor_bringup/open_camera.py
    sensor_bringup/pointcloud2_utils.py
    launch/sensors.launch.py
  )

  ament_flake8(${python_sources})
  ament_pep257(${python_sources})
  ament_lint_cmake(CMakeLists.txt)
  ament_xmllint(package.xml)
endif()


install(
  PROGRAMS
    sensor_bringup/imu_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_visualizer
)

ament_package()
