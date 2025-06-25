#include <rclcpp/rclcpp.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <fcl/fcl.h>
#include <resource_retriever/retriever.h>
#include <Eigen/Dense>

class FCLMeshCollisionChecker : public rclcpp::Node {
public:
  FCLMeshCollisionChecker() : Node("mesh_collision_checker") {
    using namespace fcl;
    using namespace Eigen;

    // Load mesh using resource_retriever
    resource_retriever::Retriever retriever;
    std::string mesh_path = "package://mr_robot_description/meshes/base_link.stl";

    resource_retriever::MemoryResource res;
    try {
      res = retriever.get(mesh_path);
      RCLCPP_INFO(this->get_logger(), "✅ Mesh file loaded.");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to load mesh file: %s", e.what());
      return;
    }

    // Convert to shape::Mesh (binary STL support)
    shapes::Mesh* mesh_shape = shapes::createMeshFromBinary(
      reinterpret_cast<const char*>(res.data.get()), res.size, "stl");

    if (!mesh_shape) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to convert mesh to shape::Mesh.");
      return;
    }

    // Apply scaling from URDF
    mesh_shape->scale(0.001);

    // Convert to FCL Collision Geometry
    auto shape_ptr = std::shared_ptr<shapes::Shape>(mesh_shape);
    auto base_geom = shapes::constructFCLCollisionGeometry(shape_ptr);

    if (!base_geom) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to construct FCL geometry from mesh.");
      return;
    }

    auto base_obj = std::make_shared<CollisionObjectd>(base_geom);

    // Set robot position in world
    Isometry3d robot_pose = Isometry3d::Identity();
    robot_pose.translation() = Vector3d(0.0, 0.0, 0.05);  // e.g. 5 cm above ground
    base_obj->setTransform(robot_pose);

    // Define obstacle (a 1m cube) at a specific location
    auto obs_shape = std::make_shared<shapes::Box>(1.0, 1.0, 1.0);
    auto obs_geom = shapes::constructFCLCollisionGeometry(obs_shape);
    auto obs_obj = std::make_shared<CollisionObjectd>(obs_geom);

    Isometry3d obstacle_pose = Isometry3d::Identity();
    obstacle_pose.translation() = Vector3d(1.4935, 0.029089, 0.5);
    obs_obj->setTransform(obstacle_pose);

    // Perform collision check
    CollisionRequestd request;
    CollisionResultd result;
    collide(base_obj.get(), obs_obj.get(), request, result);

    if (result.isCollision()) {
      RCLCPP_WARN(this->get_logger(), "⚠️ Collision Detected between base_link and obstacle!");
    } else {
      RCLCPP_INFO(this->get_logger(), "✅ No collision.");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FCLMeshCollisionChecker>());
  rclcpp::shutdown();
  return 0;
}
