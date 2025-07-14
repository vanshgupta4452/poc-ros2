cmake_minimum_required(VERSION 3.10)
project(kdl_ik_solver)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(interactive_markers REQUIRED)


# Find and link NLOPT manually
find_library(NLOPT_LIB nlopt REQUIRED)

# ✅ Add TRAC-IK subdirectory (if not already installed via apt or colcon)
add_subdirectory("/home/vansh/ros2_ws/src/trac_ik/trac_ik_lib" trac_ik_lib_build)

# Include directories
include_directories(
  include
  /home/vansh/ros2_ws/src/trac_ik/trac_ik_lib/include
  ${orocos_kdl_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# === Executables ===

# KDL-based IK node
add_executable(kdl_ik_solver_node src/kdl_ik_solver_node.cpp)
ament_target_dependencies(kdl_ik_solver_node
  rclcpp sensor_msgs visualization_msgs geometry_msgs
  tf2 tf2_ros urdf kdl_parser orocos_kdl
)
target_include_directories(kdl_ik_solver_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

# TRAC-IK-based IK node
add_executable(ik_tracik_node src/track_ik_solver_node.cpp)
ament_target_dependencies(ik_tracik_node
  rclcpp sensor_msgs visualization_msgs geometry_msgs
  tf2 tf2_ros urdf kdl_parser orocos_kdl  interactive_markers
)
target_link_libraries(ik_tracik_node
  trac_ik_lib
  ${NLOPT_LIB}
)
target_include_directories(ik_tracik_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

# === Install ===
install(TARGETS
  kdl_ik_solver_node
  ik_tracik_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Testing ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
