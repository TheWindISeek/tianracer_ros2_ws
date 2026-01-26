cmake_minimum_required(VERSION 3.8)
project(scan_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scan_filter/dynamic_obstacle_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
