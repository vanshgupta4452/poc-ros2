cmake_minimum_required(VERSION 3.8)
project(mr_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(fcl REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(assimp REQUIRED)




link_directories(/usr/lib/x86_64-linux-gnu)



include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometric_shapes_INCLUDE_DIRS}

  ${fcl_INCLUDE_DIRS}

)


# Add your executable
add_executable(collision_checker_node src/collision_checker_node.cpp)
target_link_libraries(collision_checker_node fcl ccd)

add_executable(urdf_to_fcl src/urdf_to_fcl.cpp)
target_link_libraries(urdf_to_fcl ${geometric_shapes_LIBRARIES})



# Link dependencies
ament_target_dependencies(collision_checker_node
  rclcpp
  fcl
  geometric_shapes
  nav_msgs
  tf2_ros
  urdf
  tf2_geometry_msgs
 
  
)
ament_target_dependencies(urdf_to_fcl
  rclcpp
  fcl
  geometric_shapes
  nav_msgs
  tf2_ros
  urdf
  tf2_geometry_msgs
  assimp
  geometric_shapes 
  
)


install(
  TARGETS collision_checker_node urdf_to_fcl
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds scripts src
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
