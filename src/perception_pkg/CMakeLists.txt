cmake_minimum_required(VERSION 3.8)
project(perception_pkg)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# OpenCV 4.x
find_package(OpenCV 4 REQUIRED)

# Eigen3 for polynomial fitting
find_package(Eigen3 REQUIRED)

# Shared library for common utilities
add_library(perception_common SHARED
  src/common/image_utils.cpp
  src/common/lane_geometry.cpp
)
ament_target_dependencies(perception_common
  rclcpp
  cv_bridge
  sensor_msgs
)
target_link_libraries(perception_common
  ${OpenCV_LIBRARIES}
)
target_include_directories(perception_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# C++ executables
add_executable(lane_marking_node src/lane_marking_node.cpp)
target_link_libraries(lane_marking_node perception_common)
ament_target_dependencies(lane_marking_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
target_include_directories(lane_marking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(lane_tracking_node src/lane_tracking_node.cpp)
target_link_libraries(lane_tracking_node perception_common)
ament_target_dependencies(lane_tracking_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
target_include_directories(lane_tracking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(obstacle_detection_node src/obstacle_detection_node.cpp)
target_link_libraries(obstacle_detection_node perception_common)
ament_target_dependencies(obstacle_detection_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
target_include_directories(obstacle_detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install C++ library and executables
install(TARGETS perception_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS lane_marking_node lane_tracking_node obstacle_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/lane_tracking_node.py
  scripts/lane_marking_node.py
  scripts/speed_sign_node.py
  scripts/traffic_light_node.py
  scripts/traffic_light_color_node.py
  scripts/obstacle_detection_node.py
  perception_pkg/parking_line_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install config files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install models
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/models)
  install(DIRECTORY models
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(perception_common)
ament_export_dependencies(rclcpp cv_bridge sensor_msgs OpenCV)

ament_package()
