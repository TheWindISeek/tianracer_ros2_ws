cmake_minimum_required(VERSION 3.8)
project(tianracer_slam_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install param files (Cartographer configuration)
install(DIRECTORY param/
  DESTINATION share/${PROJECT_NAME}/param
)

# Install maps directory
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

ament_package()
