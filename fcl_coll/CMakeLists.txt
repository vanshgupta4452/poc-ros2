cmake_minimum_required(VERSION 3.8)
project(fcl_coll)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(resource_retriever REQUIRED)

add_executable(fcl_self_collision_node src/fcl_self_collision.cpp)

ament_target_dependencies(fcl_self_collision_node
  rclcpp
  urdf
  geometric_shapes
  Eigen3
  ament_index_cpp
  tf2_ros
  tf2_geometry_msgs
  resource_retriever
)
target_link_libraries(fcl_self_collision_node
  fcl
)

install(
  DIRECTORY  urdf src urdf2
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  fcl_self_collision_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
