cmake_minimum_required(VERSION 3.10.2)
project(handeye_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(moveit_calibration_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(yaml-cpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add executable
add_executable(handeye_calibrator src/handeye_calibrator.cpp)
ament_target_dependencies(handeye_calibrator
  rclcpp
  rclcpp_action
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  cv_bridge
  image_transport
  moveit_calibration_plugins
  pluginlib
  OpenCV
  Eigen3
)
target_link_libraries(handeye_calibrator yaml-cpp)

# Install targets
install(TARGETS
  handeye_calibrator
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_action
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  cv_bridge
  image_transport
  moveit_calibration_plugins
  pluginlib
  OpenCV
  Eigen3
  yaml-cpp
)

ament_package()
