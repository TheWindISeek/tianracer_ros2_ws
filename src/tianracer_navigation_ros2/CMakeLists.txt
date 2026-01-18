cmake_minimum_required(VERSION 3.8)
project(tianracer_navigation_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install param files (Nav2 configuration)
install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install docs
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

ament_package()

