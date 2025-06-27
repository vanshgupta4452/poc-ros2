cmake_minimum_required(VERSION 3.10)
project(fcl_coll)

# Set policy to suppress Boost warning
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(resource_retriever REQUIRED)

# Find Assimp using PkgConfig (more reliable)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ASSIMP REQUIRED assimp)

# Add executable
add_executable(fcl_self_collision_node src/fcl_self_collision.cpp)
add_executable(self_node src/self.cpp)

# Set target dependencies
ament_target_dependencies(fcl_self_collision_node
  rclcpp
  geometry_msgs
  visualization_msgs
  urdf
  geometric_shapes
  Eigen3
  ament_index_cpp
  tf2_ros
  tf2_geometry_msgs
  resource_retriever
)
ament_target_dependencies(self_node
  rclcpp
  geometry_msgs
  visualization_msgs
  urdf
  geometric_shapes
  Eigen3
  ament_index_cpp
  tf2_ros
  tf2_geometry_msgs
  resource_retriever
)

# Link libraries
target_link_libraries(fcl_self_collision_node
  fcl
  ${ASSIMP_LIBRARIES}
)
target_link_libraries(self_node
  fcl
  ${ASSIMP_LIBRARIES}
)

# Include directories for Assimp
target_include_directories(fcl_self_collision_node PRIVATE 
  ${ASSIMP_INCLUDE_DIRS}
)
target_include_directories(self_node PRIVATE 
  ${ASSIMP_INCLUDE_DIRS}
)

# Compile options for Assimp
target_compile_options(fcl_self_collision_node PRIVATE 
  ${ASSIMP_CFLAGS_OTHER}
)

target_compile_options(self_node PRIVATE 
  ${ASSIMP_CFLAGS_OTHER}
)
# Install directories
install(
  DIRECTORY urdf src urdf2 meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install targets
install(TARGETS
  fcl_self_collision_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  self_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()