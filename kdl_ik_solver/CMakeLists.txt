cmake_minimum_required(VERSION 3.10)  # Safer min version for modern CMake policies
project(kdl_ik_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

# Declare executable
add_executable(kdl_ik_solver_node src/kdl_ik_solver_node.cpp)

# Link dependencies
ament_target_dependencies(kdl_ik_solver_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  tf2_ros
  tf2
  urdf
  kdl_parser
  orocos_kdl
)



# Install
install(TARGETS
  kdl_ik_solver_node
  DESTINATION lib/${PROJECT_NAME})

# Include Eigen headers and link library
target_include_directories(kdl_ik_solver_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
