cmake_minimum_required(VERSION 3.8)
project(arduino_driver)

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
find_package(ackermann_msgs REQUIRED)

# Boost for serial communication
find_package(Boost REQUIRED COMPONENTS system thread)

# C++ executable
add_executable(arduino_bridge_node
  src/arduino_bridge_node.cpp
  src/serial_protocol.cpp
)
ament_target_dependencies(arduino_bridge_node
  rclcpp
  std_msgs
  ackermann_msgs
)
target_link_libraries(arduino_bridge_node
  ${Boost_LIBRARIES}
)
target_include_directories(arduino_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
)

# Install C++ executable
install(TARGETS arduino_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executable
install(PROGRAMS
  scripts/arduino_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Arduino sketches (ino files)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/ino)
  install(DIRECTORY ino
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
