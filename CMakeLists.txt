cmake_minimum_required(VERSION 3.8)
project(rl_race)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)


install(DIRECTORY
    launch urdf worlds scripts
    DESTINATION share/${PROJECT_NAME}/
)

# add_executable(map_builder src/map_builder.cpp)
# ament_target_dependencies(map_builder rclcpp sensor_msgs nav_msgs std_msgs)

# add_executable(data_publisher src/data_publisher.cpp)
# ament_target_dependencies(data_publisher rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs)

install(TARGETS
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/visualize_odometry.py
  scripts/teleop_twist.py
  scripts/maping.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
