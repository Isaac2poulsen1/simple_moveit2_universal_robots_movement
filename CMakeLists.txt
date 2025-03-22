cmake_minimum_required(VERSION 3.8)
project(basic_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)  # Added for transformations
find_package(tf2_geometry_msgs REQUIRED)  # Added for geometry conversions
find_package(graph_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)


# Add Executable (Corrected)
add_executable(basic_move 
  src/basic_move.cpp 
  src/collision_objects.cpp
  src/static_pos.cpp
)


# Include Directories 
target_include_directories(basic_move PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Set C++ Standard
target_compile_features(basic_move PUBLIC cxx_std_17)  # Fixed

# Link Dependencies
ament_target_dependencies(basic_move
  moveit_ros_planning_interface
  rclcpp
  tf2
  tf2_geometry_msgs
  graph_msgs
  moveit_visual_tools
  rviz_visual_tools
)

# Install Executable
install(TARGETS basic_move
  DESTINATION lib/${PROJECT_NAME})

# Install Launch Files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
