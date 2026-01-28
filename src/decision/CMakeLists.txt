cmake_minimum_required(VERSION 3.8)
project(decision)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# C++ executables
add_executable(decision_node src/decision_node.cpp)
ament_target_dependencies(decision_node
  rclcpp
  std_msgs
  sensor_msgs
  ackermann_msgs
)
target_include_directories(decision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(lidar_obstacle_node src/lidar_obstacle_node.cpp)
ament_target_dependencies(lidar_obstacle_node
  rclcpp
  std_msgs
  sensor_msgs
)
target_include_directories(lidar_obstacle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install C++ executables
install(TARGETS decision_node lidar_obstacle_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/decision_node.py
  scripts/decision_node_2026.py
  scripts/decision_node_ai.py
  scripts/lidar_obstacle_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
