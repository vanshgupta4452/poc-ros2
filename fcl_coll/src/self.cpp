#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <fcl/fcl.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <tf2_ros/transform_broadcaster.h>
#include <fstream>
#include <set>
#include <string>

class SelfCollisionChecker : public rclcpp::Node {
public:
  SelfCollisionChecker() : Node("self_collision_checker") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("self_collision_markers", 10);

    std::string urdf_path = ament_index_cpp::get_package_share_directory("fcl_coll") + "/urdf2/mr_robot.xacro";
    if (!model_.initFile(urdf_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load URDF: %s", urdf_path.c_str());
      return;
    }

    package_share_dir_ = ament_index_cpp::get_package_share_directory("fcl_coll");
    setupCollisionObjects();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&SelfCollisionChecker::checkSelfCollision, this));
  }

private:
  urdf::Model model_;
  Assimp::Importer importer_;
  std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> collision_objects_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string package_share_dir_;

  std::string resolveMeshPath(const std::string &mesh_filename) {
    if (mesh_filename.find("package://") == 0) {
      std::string relative_path = mesh_filename.substr(10);
      size_t slash_pos = relative_path.find('/');
      std::string pkg = relative_path.substr(0, slash_pos);
      std::string file = relative_path.substr(slash_pos + 1);
      return ament_index_cpp::get_package_share_directory(pkg) + "/" + file;
    } else if (mesh_filename.find("file://") == 0) {
      return mesh_filename.substr(7);
    } else {
      return package_share_dir_ + "/" + mesh_filename;
    }
  }

  void setupCollisionObjects() {
    for (const auto &pair : model_.links_) {
      const auto &link = pair.second;
      if (!link->collision || !link->collision->geometry) continue;

      std::shared_ptr<fcl::CollisionGeometryd> geom;
      auto origin = link->collision->origin.position;
      auto rotation = link->collision->origin.rotation;

      switch (link->collision->geometry->type) {
        case urdf::Geometry::BOX: {
          auto *box = dynamic_cast<urdf::Box *>(link->collision->geometry.get());
          geom = std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
          break;
        }
        case urdf::Geometry::CYLINDER: {
          auto *cyl = dynamic_cast<urdf::Cylinder *>(link->collision->geometry.get());
          geom = std::make_shared<fcl::Cylinderd>(cyl->radius, cyl->length);
          break;
        }
        case urdf::Geometry::SPHERE: {
          auto *sph = dynamic_cast<urdf::Sphere *>(link->collision->geometry.get());
          geom = std::make_shared<fcl::Sphered>(sph->radius);
          break;
        }
        case urdf::Geometry::MESH: {
          auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);
          if (!mesh) continue;

          std::string mesh_path = resolveMeshPath(mesh->filename);
          const aiScene *scene = importer_.ReadFile(mesh_path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
          if (!scene || !scene->mMeshes || scene->mNumMeshes == 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to load mesh for %s: %s", link->name.c_str(), mesh_path.c_str());
            continue;
          }

          const aiMesh *ai_mesh = scene->mMeshes[0];
          std::vector<fcl::Vector3d> vertices;
          std::vector<fcl::Triangle> triangles;

          for (unsigned int i = 0; i < ai_mesh->mNumVertices; ++i) {
            const aiVector3D &v = ai_mesh->mVertices[i];
            vertices.emplace_back(v.x * mesh->scale.x, v.y * mesh->scale.y, v.z * mesh->scale.z);
          }

          for (unsigned int i = 0; i < ai_mesh->mNumFaces; ++i) {
            const aiFace &face = ai_mesh->mFaces[i];
            if (face.mNumIndices == 3) {
              triangles.emplace_back(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
            }
          }

          auto bvh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
          bvh->beginModel();
          bvh->addSubModel(vertices, triangles);
          bvh->endModel();
          geom = bvh;
          break;
        }
        default:
          continue;
      }

      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation() << origin.x, origin.y, origin.z;
      Eigen::Quaterniond quat(rotation.w, rotation.x, rotation.y, rotation.z);
      tf.linear() = quat.toRotationMatrix();

      collision_objects_[link->name] = std::make_shared<fcl::CollisionObjectd>(geom, tf);
    }
  }

  void checkSelfCollision() {
    std::set<std::string> collided_links;
    std::vector<std::pair<std::string, std::string>> collided_pairs;

    for (auto it1 = collision_objects_.begin(); it1 != collision_objects_.end(); ++it1) {
      for (auto it2 = std::next(it1); it2 != collision_objects_.end(); ++it2) {
        fcl::CollisionRequestd request;
        fcl::CollisionResultd result;
        fcl::collide(it1->second.get(), it2->second.get(), request, result);
        if (result.isCollision()) {
          collided_links.insert(it1->first);
          collided_links.insert(it2->first);
          collided_pairs.push_back({it1->first, it2->first});
        }
      }
    }

    if (!collided_pairs.empty()) {
      RCLCPP_INFO(this->get_logger(), "[COLLISION] Links involved:");
      for (const auto &[l1, l2] : collided_pairs) {
        RCLCPP_INFO(this->get_logger(), "- %s <-> %s", l1.c_str(), l2.c_str());
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "No collisions detected.");
    }

    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto &[link_name, obj] : collision_objects_) {
      auto *urdf_link = model_.getLink(link_name).get();
      auto &origin = urdf_link->collision->origin;

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "self_collision";
      marker.id = id++;
      marker.action = marker.ADD;
      marker.pose.position.x = origin.position.x;
      marker.pose.position.y = origin.position.y;
      marker.pose.position.z = origin.position.z;
      marker.pose.orientation.x = origin.rotation.x;
      marker.pose.orientation.y = origin.rotation.y;
      marker.pose.orientation.z = origin.rotation.z;
      marker.pose.orientation.w = origin.rotation.w;

      auto *geom = urdf_link->collision->geometry.get();
      if (geom->type == urdf::Geometry::BOX) {
        auto *box = dynamic_cast<urdf::Box *>(geom);
        marker.type = marker.CUBE;
        marker.scale.x = box->dim.x;
        marker.scale.y = box->dim.y;
        marker.scale.z = box->dim.z;
      } else if (geom->type == urdf::Geometry::CYLINDER) {
        auto *cyl = dynamic_cast<urdf::Cylinder *>(geom);
        marker.type = marker.CYLINDER;
        marker.scale.x = cyl->radius * 2;
        marker.scale.y = cyl->radius * 2;
        marker.scale.z = cyl->length;
      } else if (geom->type == urdf::Geometry::SPHERE) {
        auto *sph = dynamic_cast<urdf::Sphere *>(geom);
        marker.type = marker.SPHERE;
        marker.scale.x = sph->radius * 2;
        marker.scale.y = sph->radius * 2;
        marker.scale.z = sph->radius * 2;
      } else if (geom->type == urdf::Geometry::MESH) {
        auto *mesh = dynamic_cast<urdf::Mesh *>(geom);
        marker.type = marker.MESH_RESOURCE;
        marker.mesh_resource = "file://" + resolveMeshPath(mesh->filename);
        marker.scale.x = mesh->scale.x;
        marker.scale.y = mesh->scale.y;
        marker.scale.z = mesh->scale.z;
      }

      if (collided_links.count(link_name)) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      } else {
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
      }
      marker.color.a = 1.0;

      marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelfCollisionChecker>());
  rclcpp::shutdown();
  return 0;
}
