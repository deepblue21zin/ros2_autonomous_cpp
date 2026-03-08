cmake_minimum_required(VERSION 3.8)
project(arduino_driver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Install Python executable
install(PROGRAMS
  scripts/arduino_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
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
