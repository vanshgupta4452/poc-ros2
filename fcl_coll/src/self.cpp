#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <fcl/fcl.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>
#include <fstream>
#include <set>
#include <string>

class SelfCollisionChecker : public rclcpp::Node {
public:
  SelfCollisionChecker() : Node("self_collision_checker") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("self_collision_markers", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // Subscribe to joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&SelfCollisionChecker::jointStateCallback, this, std::placeholders::_1));

    std::string urdf_path = ament_index_cpp::get_package_share_directory("PXA-100_description") + "/urdf/PXA-100.urdf";
    if (!model_.initFile(urdf_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load URDF: %s", urdf_path.c_str());
      return;
    }

    package_share_dir_ = ament_index_cpp::get_package_share_directory("PXA-100_description");
    setupCollisionObjects();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SelfCollisionChecker::checkSelfCollision, this));
  }

private:
  urdf::Model model_;
  Assimp::Importer importer_;
  std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> collision_objects_;
  std::map<std::string, std::shared_ptr<fcl::CollisionGeometryd>> collision_geometries_;
  std::map<std::string, Eigen::Isometry3d> link_transforms_;
  std::map<std::string, double> current_joint_positions_;
  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string package_share_dir_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Update joint positions
    for (size_t i = 0; i < msg->name.size(); ++i) {
      current_joint_positions_[msg->name[i]] = msg->position[i];
    }
    
    // Recalculate all link transforms based on current joint positions
    updateLinkTransforms();
  }

  void updateLinkTransforms() {
    // Start from base_link (root)
    auto base_link = model_.getLink("base_link");
    if (base_link) {
      link_transforms_["base_link"] = Eigen::Isometry3d::Identity();
      updateChildLinkTransforms(base_link, Eigen::Isometry3d::Identity());
    }
    
    // Update collision objects with new transforms
    for (auto &[link_name, obj] : collision_objects_) {
      if (link_transforms_.count(link_name)) {
        // Apply collision origin offset
        auto urdf_link = model_.getLink(link_name);
        if (urdf_link && urdf_link->collision) {
          Eigen::Isometry3d collision_offset = Eigen::Isometry3d::Identity();
          collision_offset.translation() << urdf_link->collision->origin.position.x,
                                           urdf_link->collision->origin.position.y,
                                           urdf_link->collision->origin.position.z;
          Eigen::Quaterniond q(urdf_link->collision->origin.rotation.w,
                               urdf_link->collision->origin.rotation.x,
                               urdf_link->collision->origin.rotation.y,
                               urdf_link->collision->origin.rotation.z);
          collision_offset.linear() = q.toRotationMatrix();
          
          Eigen::Isometry3d final_transform = link_transforms_[link_name] * collision_offset;
          obj->setTransform(final_transform);
        }
      }
    }
  }

  void updateChildLinkTransforms(const urdf::LinkConstSharedPtr& parent_link, const Eigen::Isometry3d& parent_transform) {
    // Get all child joints of this link
    for (const auto& joint_pair : model_.joints_) {
      const auto& joint = joint_pair.second;
      
      if (joint->parent_link_name == parent_link->name) {
        // Calculate joint transform
        Eigen::Isometry3d joint_transform = Eigen::Isometry3d::Identity();
        
        // Apply joint origin transform
        const urdf::Pose& joint_origin = joint->parent_to_joint_origin_transform;
        joint_transform.translation() << joint_origin.position.x, joint_origin.position.y, joint_origin.position.z;
        Eigen::Quaterniond q(joint_origin.rotation.w, joint_origin.rotation.x, joint_origin.rotation.y, joint_origin.rotation.z);
        joint_transform.linear() = q.toRotationMatrix();
        
        // Apply joint angle if it's a revolute or prismatic joint
        if (current_joint_positions_.count(joint->name)) {
          double joint_angle = current_joint_positions_[joint->name];
          
          if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
            // Apply rotation around joint axis
            Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
            Eigen::AngleAxisd rotation(joint_angle, axis);
            joint_transform.linear() = joint_transform.linear() * rotation.toRotationMatrix();
          } else if (joint->type == urdf::Joint::PRISMATIC) {
            // Apply translation along joint axis
            Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
            joint_transform.translation() += joint_angle * axis;
          }
        }
        
        // Calculate child link transform
        Eigen::Isometry3d child_transform = parent_transform * joint_transform;
        
        // Get child link
        auto child_link = model_.getLink(joint->child_link_name);
        if (child_link) {
          link_transforms_[child_link->name] = child_transform;
          
          // Recursively update child links
          updateChildLinkTransforms(child_link, child_transform);
        }
      }
    }
  }

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

      collision_geometries_[link->name] = geom;
      collision_objects_[link->name] = std::make_shared<fcl::CollisionObjectd>(geom, Eigen::Isometry3d::Identity());
    }
  }

  void checkSelfCollision() {
    std::set<std::string> collided_links;
    std::vector<std::pair<std::string, std::string>> collided_pairs;
    std::set<std::pair<std::string, std::string>> adjacent_pairs;

    // Build adjacent pairs (parent-child relationships)
    for (const auto &joint : model_.joints_) {
      const std::string &parent = joint.second->parent_link_name;
      const std::string &child = joint.second->child_link_name;
      adjacent_pairs.insert({parent, child});
      adjacent_pairs.insert({child, parent});
    }

    // Check for collisions
    for (auto it1 = collision_objects_.begin(); it1 != collision_objects_.end(); ++it1) {
      for (auto it2 = std::next(it1); it2 != collision_objects_.end(); ++it2) {
        // Skip adjacent links
        if (adjacent_pairs.count({it1->first, it2->first})) continue;

        fcl::CollisionRequestd request;
        fcl::CollisionResultd result;
        fcl::collide(it1->second.get(), it2->second.get(), request, result);

        if (result.isCollision()) {
          collided_links.insert(it1->first);
          collided_links.insert(it2->first);
          collided_pairs.emplace_back(it1->first, it2->first);
        }
      }
    }

    // Log collision results
    if (!collided_pairs.empty()) {
      RCLCPP_INFO(this->get_logger(), "[COLLISION] Links involved:");
      for (const auto &[l1, l2] : collided_pairs) {
        RCLCPP_INFO(this->get_logger(), "- %s <-> %s", l1.c_str(), l2.c_str());
      }
    }

    // Create visualization markers
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto &[link_name, obj] : collision_objects_) {
      if (!link_transforms_.count(link_name)) continue;

      auto urdf_link = model_.getLink(link_name);
      if (!urdf_link || !urdf_link->collision) continue;

      visualization_msgs::msg::Marker marker;
      marker.header.stamp = this->now();
      marker.header.frame_id = "base_link";
      marker.ns = "self_collision";
      marker.id = id++;
      marker.action = marker.ADD;

      // Set pose from link transform
      Eigen::Isometry3d tf = link_transforms_[link_name];
      
      // Apply collision origin offset
      Eigen::Isometry3d collision_offset = Eigen::Isometry3d::Identity();
      collision_offset.translation() << urdf_link->collision->origin.position.x,
                                       urdf_link->collision->origin.position.y,
                                       urdf_link->collision->origin.position.z;
      Eigen::Quaterniond q(urdf_link->collision->origin.rotation.w,
                           urdf_link->collision->origin.rotation.x,
                           urdf_link->collision->origin.rotation.y,
                           urdf_link->collision->origin.rotation.z);
      collision_offset.linear() = q.toRotationMatrix();
      
      Eigen::Isometry3d final_tf = tf * collision_offset;
      Eigen::Vector3d pos = final_tf.translation();
      Eigen::Quaterniond quat(final_tf.rotation());

      marker.pose.position.x = pos.x();
      marker.pose.position.y = pos.y();
      marker.pose.position.z = pos.z();
      marker.pose.orientation.x = quat.x();
      marker.pose.orientation.y = quat.y();
      marker.pose.orientation.z = quat.z();
      marker.pose.orientation.w = quat.w();

      // Set geometry
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

      // Set color based on collision status
      if (collided_links.count(link_name)) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      } else {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      }
      marker.color.a = 0.7;

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