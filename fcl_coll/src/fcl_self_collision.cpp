#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>
#include <fcl/fcl.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <filesystem>
#include <fstream>

class FCLSelfCollisionNode : public rclcpp::Node {
public:
  FCLSelfCollisionNode() : Node("fcl_self_collision_node") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "Loading URDF...");

    std::string package_share = ament_index_cpp::get_package_share_directory("fcl_coll");
    std::string urdf_path = package_share + "/urdf2/mr_robot.xacro";
    package_share_dir_ = package_share;

    if (!model_.initFile(urdf_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
      return;
    }

    setupCollisionObjects();

    auto sphere_geom = std::make_shared<fcl::Sphered>(0.05);
    Eigen::Isometry3d sphere_tf = Eigen::Isometry3d::Identity();
    sphere_tf.translation() << -0.31, 0.0, 0.0; // start near the robot
    moving_sphere_ = std::make_shared<fcl::CollisionObjectd>(sphere_geom, sphere_tf);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FCLSelfCollisionNode::publishMarkersAndTFs, this)
    );

    RCLCPP_INFO(this->get_logger(), "URDF loaded. Ready to publish.");
  }

private:
  urdf::Model model_;
  std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> collision_objects_;
  std::shared_ptr<fcl::CollisionObjectd> moving_sphere_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string package_share_dir_;
  Assimp::Importer importer_;

  bool isAbsolutePath(const std::string& path) {
    // Check for absolute paths on Unix/Linux (starts with /)
    if (!path.empty() && path[0] == '/') {
      return true;
    }
    // Check for Windows absolute paths (C:\ or similar)
    if (path.length() >= 3 && path[1] == ':' && (path[2] == '\\' || path[2] == '/')) {
      return true;
    }
    return false;
  }

  bool fileExists(const std::string& path) {
    std::ifstream file(path);
    return file.good();
  }

  std::string resolveMeshPath(const std::string& mesh_filename) {
    // Handle package:// URIs
    if (mesh_filename.find("package://") == 0) {
      std::string relative_path = mesh_filename.substr(10); // Remove "package://"
      size_t first_slash = relative_path.find('/');
      if (first_slash != std::string::npos) {
        std::string package_name = relative_path.substr(0, first_slash);
        std::string file_path = relative_path.substr(first_slash + 1);
        
        try {
          std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
          return package_path + "/" + file_path;
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "Failed to resolve package path for %s: %s", 
                      package_name.c_str(), e.what());
        }
      }
    }
    
    // Handle file:// URIs
    if (mesh_filename.find("file://") == 0) {
      return mesh_filename.substr(7); // Remove "file://"
    }
    
    // Handle relative paths - assume they're relative to the package
    if (!isAbsolutePath(mesh_filename)) {
      return package_share_dir_ + "/" + mesh_filename;
    }
    
    // Return as-is for absolute paths
    return mesh_filename;
  }

  std::shared_ptr<fcl::CollisionGeometryd> loadMeshGeometry(const std::string& mesh_filename, 
                                                            const urdf::Vector3& scale) {
    std::string resolved_path = resolveMeshPath(mesh_filename);
    
    if (!fileExists(resolved_path)) {
      RCLCPP_ERROR(this->get_logger(), "Mesh file not found: %s", resolved_path.c_str());
      return nullptr;
    }

    const aiScene* scene = importer_.ReadFile(resolved_path, 
                                              aiProcess_Triangulate | 
                                              aiProcess_GenNormals |
                                              aiProcess_JoinIdenticalVertices);
    
    if (!scene || !scene->mMeshes || scene->mNumMeshes == 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load mesh: %s", resolved_path.c_str());
      return nullptr;
    }

    // For simplicity, use the first mesh
    const aiMesh* mesh = scene->mMeshes[0];
    
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;

    // Extract vertices with scaling
    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
      const aiVector3D& v = mesh->mVertices[i];
      vertices.emplace_back(v.x * scale.x, v.y * scale.y, v.z * scale.z);
    }

    // Extract triangles
    for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
      const aiFace& face = mesh->mFaces[i];
      if (face.mNumIndices == 3) {
        triangles.emplace_back(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
      }
    }

    if (vertices.empty() || triangles.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Mesh has no valid geometry: %s", resolved_path.c_str());
      return nullptr;
    }

    auto bvh_model = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
    bvh_model->beginModel();
    bvh_model->addSubModel(vertices, triangles);
    bvh_model->endModel();

    RCLCPP_INFO(this->get_logger(), "Loaded mesh: %s (%zu vertices, %zu triangles)", 
                resolved_path.c_str(), vertices.size(), triangles.size());

    return bvh_model;
  }

  void setupCollisionObjects() {
    for (const auto &link_pair : model_.links_) {
      const auto &link = link_pair.second;
      if (!link->collision || !link->collision->geometry) continue;

      std::shared_ptr<fcl::CollisionGeometryd> fcl_geom;

      switch (link->collision->geometry->type) {
        case urdf::Geometry::BOX: {
          auto box = dynamic_cast<urdf::Box*>(link->collision->geometry.get());
          fcl_geom = std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
          break;
        }
        case urdf::Geometry::CYLINDER: {
          auto cyl = dynamic_cast<urdf::Cylinder*>(link->collision->geometry.get());
          fcl_geom = std::make_shared<fcl::Cylinderd>(cyl->radius, cyl->length);
          break;
        }
        case urdf::Geometry::SPHERE: {
          auto sph = dynamic_cast<urdf::Sphere*>(link->collision->geometry.get());
          fcl_geom = std::make_shared<fcl::Sphered>(sph->radius);
          break;
        }
        case urdf::Geometry::MESH: {
          std::cout << "Geometry type: " << link->visual->geometry->type << std::endl;

          std::shared_ptr<urdf::Mesh> mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
          if(mesh) {
            std::cout << "✅ Mesh filename: " << mesh->filename << std::endl;
            std::cout << "✅ Scale: " << mesh->scale.x << ", " << mesh->scale.y << ", " << mesh->scale.z << std::endl;
          };
          if (mesh) {
            RCLCPP_INFO(this->get_logger(), "Loading mesh for link %s: %s", 
                        link->name.c_str(), mesh->filename.c_str());
            fcl_geom = loadMeshGeometry(mesh->filename, mesh->scale);
            if (!fcl_geom) {
              RCLCPP_WARN(this->get_logger(), "Failed to load mesh for link: %s (file: %s), skipping", 
                          link->name.c_str(), mesh->filename.c_str());
              continue;
            }
          } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid mesh geometry for link: %s", link->name.c_str());
            continue;
          }
          break;
        }
        default:
          RCLCPP_WARN(this->get_logger(), "Unsupported geometry type in link: %s", 
                      link->name.c_str());
          continue;
      }

      // Set up transform from URDF collision origin
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation() << link->collision->origin.position.x,
                          link->collision->origin.position.y,
                          link->collision->origin.position.z;
      
      // Set rotation from URDF quaternion
      Eigen::Quaterniond quat(link->collision->origin.rotation.w,
                              link->collision->origin.rotation.x,
                              link->collision->origin.rotation.y,
                              link->collision->origin.rotation.z);
      tf.linear() = quat.toRotationMatrix();

      collision_objects_[link->name] = std::make_shared<fcl::CollisionObjectd>(fcl_geom, tf);
      
      RCLCPP_INFO(this->get_logger(), "Added collision object for link: %s", link->name.c_str());
    }
  }

  void publishMarkersAndTFs() {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    // Move the sphere
    Eigen::Isometry3d sphere_tf = moving_sphere_->getTransform();
    sphere_tf.translation().x() += 0.01; // move along x
    moving_sphere_->setTransform(sphere_tf);

    bool sphere_collision = false;
    std::string collided_link;
    double min_distance = std::numeric_limits<double>::max();

    // Check collisions with all collision objects
    for (auto& [link_name, obj] : collision_objects_) {
      fcl::CollisionRequestd request;
      fcl::CollisionResultd result;

      fcl::DistanceRequestd distance_request(true);
      fcl::DistanceResultd distance_result;

      fcl::collide(moving_sphere_.get(), obj.get(), request, result);
      fcl::distance(moving_sphere_.get(), obj.get(), distance_request, distance_result);
      
      min_distance = std::min(min_distance, distance_result.min_distance);

      if (result.isCollision()) {
        sphere_collision = true;
        collided_link = link_name;
        RCLCPP_WARN(this->get_logger(), "Collision detected with link: %s", link_name.c_str());
        break;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Min Distance = %.4f", min_distance);

    // Create markers for all links
    for (const auto &link_pair : model_.links_) {
      const auto &link = link_pair.second;
      if (!link->collision || !link->collision->geometry) continue;

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = this->now();
      marker.ns = "robot";
      marker.id = id++;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = link->collision->origin.position.x;
      marker.pose.position.y = link->collision->origin.position.y;
      marker.pose.position.z = link->collision->origin.position.z;
      marker.pose.orientation.x = link->collision->origin.rotation.x;
      marker.pose.orientation.y = link->collision->origin.rotation.y;
      marker.pose.orientation.z = link->collision->origin.rotation.z;
      marker.pose.orientation.w = link->collision->origin.rotation.w;

      // Set marker type and scale based on geometry
      switch (link->collision->geometry->type) {
        case urdf::Geometry::BOX: {
          auto box = dynamic_cast<urdf::Box*>(link->collision->geometry.get());
          marker.type = visualization_msgs::msg::Marker::CUBE;
          marker.scale.x = box->dim.x;
          marker.scale.y = box->dim.y;
          marker.scale.z = box->dim.z;
          break;
        }
        case urdf::Geometry::CYLINDER: {
          auto cyl = dynamic_cast<urdf::Cylinder*>(link->collision->geometry.get());
          marker.type = visualization_msgs::msg::Marker::CYLINDER;
          marker.scale.x = cyl->radius * 2.0;
          marker.scale.y = cyl->radius * 2.0;
          marker.scale.z = cyl->length;
          break;
        }
        case urdf::Geometry::SPHERE: {
          auto sph = dynamic_cast<urdf::Sphere*>(link->collision->geometry.get());
          marker.type = visualization_msgs::msg::Marker::SPHERE;
          marker.scale.x = sph->radius * 2.0;
          marker.scale.y = sph->radius * 2.0;
          marker.scale.z = sph->radius * 2.0;
          break;
        }
        case urdf::Geometry::MESH: {
          auto mesh = dynamic_cast<urdf::Mesh*>(link->collision->geometry.get());
          if (mesh) {
            marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            marker.mesh_resource = "file://" + resolveMeshPath(mesh->filename);
            marker.scale.x = mesh->scale.x;
            marker.scale.y = mesh->scale.y;
            marker.scale.z = mesh->scale.z;
            RCLCPP_DEBUG(this->get_logger(), "Mesh marker for %s: %s", 
                        link->name.c_str(), marker.mesh_resource.c_str());
          } else {
            continue;
          }
          break;
        }
        default:
          continue;
      }

      // Set color based on collision state
      if (sphere_collision && link->name == collided_link) {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
      } else {
        marker.color.r = 0.5f;
        marker.color.g = 0.5f;
        marker.color.b = 0.5f;
        marker.color.a = 1.0f;
      }

      marker_array.markers.push_back(marker);

      // Publish transform
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = this->now();
      tf.header.frame_id = "base_link";
      tf.child_frame_id = link->name;
      tf.transform.translation.x = marker.pose.position.x;
      tf.transform.translation.y = marker.pose.position.y;
      tf.transform.translation.z = marker.pose.position.z;
      tf.transform.rotation = marker.pose.orientation;

      tf_broadcaster_->sendTransform(tf);
    }

    // Create moving sphere marker
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header.frame_id = "base_link";
    sphere_marker.header.stamp = this->now();
    sphere_marker.ns = "sphere";
    sphere_marker.id = id++;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker.scale.x = 0.1;
    sphere_marker.scale.y = 0.1;
    sphere_marker.scale.z = 0.1;
    sphere_marker.pose.position.x = moving_sphere_->getTransform().translation().x();
    sphere_marker.pose.position.y = moving_sphere_->getTransform().translation().y();
    sphere_marker.pose.position.z = moving_sphere_->getTransform().translation().z();
    sphere_marker.pose.orientation.w = 1.0;

    if (sphere_collision) {
      sphere_marker.color.r = 1.0f;
      sphere_marker.color.g = 0.0f;
      sphere_marker.color.b = 0.0f;
    } else {
      sphere_marker.color.r = 0.0f;
      sphere_marker.color.g = 1.0f;
      sphere_marker.color.b = 0.0f;
    }
    sphere_marker.color.a = 1.0f;
    marker_array.markers.push_back(sphere_marker);

    marker_pub_->publish(marker_array);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FCLSelfCollisionNode>());
  rclcpp::shutdown();
  return 0;
}