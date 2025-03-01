cmake_minimum_required(VERSION 3.10.2)
project(odometry_conversion)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wfatal-errors)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Include directories
include_directories(
  include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

# Create library
add_library(${PROJECT_NAME}
  src/OdometryConversion.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  Eigen3
)

# Create executable
add_executable(${PROJECT_NAME}_node
  src/odometry_conversion_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  Eigen3
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

# Export package
ament_package()
