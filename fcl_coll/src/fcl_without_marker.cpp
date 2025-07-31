#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <fcl/fcl.h>
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
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // Subscribe to joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, std::bind(&SelfCollisionChecker::jointStateCallback, this, std::placeholders::_1));

    // Load arm URDF
    std::string urdf_path = ament_index_cpp::get_package_share_directory("ajgar_description") + "/urdf/arm.urdf";
    if (!arm_model_.initFile(urdf_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load arm URDF: %s", urdf_path.c_str());
      return;
    }

    // Load station URDF
    std::string station_urdf = ament_index_cpp::get_package_share_directory("Station_description") + "/urdf/Station.urdf";
    if (!station_model_.initFile(station_urdf)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load station URDF: %s", station_urdf.c_str());
      return;
    }

    std::string table_urdf = ament_index_cpp::get_package_share_directory("fixture_table_description") + "/urdf/fixture_table.urdf";
    if (!table_model_.initFile(table_urdf)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load table URDF: %s", table_urdf.c_str());
      return;
    }

    arm_package_share_dir_ = ament_index_cpp::get_package_share_directory("ajgar_description");
    station_package_share_dir_ = ament_index_cpp::get_package_share_directory("Station_description");
    table_package_share_dir_ = ament_index_cpp::get_package_share_directory("fixture_table_description");
    
    setupArmCollisionObjects();
    setupStationCollisionObjects();
    setupTableCollisionObjects();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                    std::bind(&SelfCollisionChecker::checkCollisions, this));
  }

private:
  urdf::Model arm_model_;
  urdf::Model station_model_;
  urdf::Model table_model_;
  Assimp::Importer importer_;
  
  // Arm collision objects
  std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> arm_collision_objects_;
  std::map<std::string, std::shared_ptr<fcl::CollisionGeometryd>> arm_collision_geometries_;
  std::map<std::string, Eigen::Isometry3d> arm_link_transforms_;
  
  // Station collision objects (static)
  std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> station_collision_objects_;
  std::map<std::string, std::shared_ptr<fcl::CollisionGeometryd>> station_collision_geometries_;

  std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> table_collision_objects_;
  std::map<std::string, std::shared_ptr<fcl::CollisionGeometryd>> table_collision_geometries_;
  
  std::map<std::string, double> current_joint_positions_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string arm_package_share_dir_;
  std::string station_package_share_dir_;
  std::string table_package_share_dir_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      current_joint_positions_[msg->name[i]] = msg->position[i];
    }
    
    updateArmLinkTransforms();
  }

  void updateArmLinkTransforms() {
    auto base_link = arm_model_.getLink("base_link");
    if (base_link) {
      arm_link_transforms_["base_link"] = Eigen::Isometry3d::Identity();
      updateChildLinkTransforms(base_link, Eigen::Isometry3d::Identity());
    }
    
    // Update arm collision objects with new transforms
    for (auto &[link_name, obj] : arm_collision_objects_) {
      if (arm_link_transforms_.count(link_name)) {
        // Apply collision origin offset
        auto urdf_link = arm_model_.getLink(link_name);
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
          
          Eigen::Isometry3d final_transform = arm_link_transforms_[link_name] * collision_offset;
          obj->setTransform(final_transform);
        }
      }
    }
  }

  void updateChildLinkTransforms(const urdf::LinkConstSharedPtr& parent_link, const Eigen::Isometry3d& parent_transform) {
    for (const auto& joint_pair : arm_model_.joints_) {
      const auto& joint = joint_pair.second;
      
      if (joint->parent_link_name == parent_link->name) {
        Eigen::Isometry3d joint_transform = Eigen::Isometry3d::Identity();
        
        const urdf::Pose& joint_origin = joint->parent_to_joint_origin_transform;
        joint_transform.translation() << joint_origin.position.x, joint_origin.position.y, joint_origin.position.z;
        Eigen::Quaterniond q(joint_origin.rotation.w, joint_origin.rotation.x, joint_origin.rotation.y, joint_origin.rotation.z);
        joint_transform.linear() = q.toRotationMatrix();
        
        if (current_joint_positions_.count(joint->name)) {
          double joint_angle = current_joint_positions_[joint->name];
          
          if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
            Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
            Eigen::AngleAxisd rotation(joint_angle, axis);
            joint_transform.linear() = joint_transform.linear() * rotation.toRotationMatrix();
          } else if (joint->type == urdf::Joint::PRISMATIC) {
            Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
            joint_transform.translation() += joint_angle * axis;
          }
        }
        
        Eigen::Isometry3d child_transform = parent_transform * joint_transform;
        
        auto child_link = arm_model_.getLink(joint->child_link_name);
        if (child_link) {
          arm_link_transforms_[child_link->name] = child_transform;
          
          updateChildLinkTransforms(child_link, child_transform);
        }
      }
    }
  }

  std::string resolveMeshPath(const std::string &mesh_filename, const std::string &package_dir) {
    if (mesh_filename.find("package://") == 0) {
      std::string relative_path = mesh_filename.substr(10);
      size_t slash_pos = relative_path.find('/');
      std::string pkg = relative_path.substr(0, slash_pos);
      std::string file = relative_path.substr(slash_pos + 1);
      return ament_index_cpp::get_package_share_directory(pkg) + "/" + file;
    } else if (mesh_filename.find("file://") == 0) {
      return mesh_filename.substr(7);
    } else {
      return package_dir + "/" + mesh_filename;
    }
  }

  void setupArmCollisionObjects() {
    for (const auto &pair : arm_model_.links_) {
      const auto &link = pair.second;
      if (!link->collision || !link->collision->geometry) continue;

      std::shared_ptr<fcl::CollisionGeometryd> geom = createGeometry(link->collision->geometry.get(), arm_package_share_dir_);
      if (!geom) continue;

      arm_collision_geometries_[link->name] = geom;
      arm_collision_objects_[link->name] = std::make_shared<fcl::CollisionObjectd>(geom, Eigen::Isometry3d::Identity());
    }
  }

  void setupStationCollisionObjects() {
    for (const auto &pair : station_model_.links_) {
      const auto &link = pair.second;
      if (!link->collision || !link->collision->geometry) continue;

      std::shared_ptr<fcl::CollisionGeometryd> geom = createGeometry(link->collision->geometry.get(), station_package_share_dir_);
      if (!geom) continue;

      station_collision_geometries_[link->name] = geom;
      
      Eigen::Isometry3d station_transform = Eigen::Isometry3d::Identity();
      
      // Apply collision origin offset for station
      if (link->collision) {
        Eigen::Isometry3d collision_offset = Eigen::Isometry3d::Identity();
        collision_offset.translation() << link->collision->origin.position.x,
                                         link->collision->origin.position.y,
                                         link->collision->origin.position.z;
        Eigen::Quaterniond q(link->collision->origin.rotation.w,
                             link->collision->origin.rotation.x,
                             link->collision->origin.rotation.y,
                             link->collision->origin.rotation.z);
        collision_offset.linear() = q.toRotationMatrix();
        
        station_transform = station_transform * collision_offset;
      }
      
      station_collision_objects_[link->name] = std::make_shared<fcl::CollisionObjectd>(geom, station_transform);
    }
  }

  void setupTableCollisionObjects() {
    for (const auto &pair : table_model_.links_) {
      const auto &link = pair.second;
      if (!link->collision || !link->collision->geometry) continue;

      std::shared_ptr<fcl::CollisionGeometryd> geom = createGeometry(link->collision->geometry.get(), table_package_share_dir_);
      if (!geom) continue;

      table_collision_geometries_[link->name] = geom;
      
      Eigen::Isometry3d table_transform = Eigen::Isometry3d::Identity();
      
      if (link->collision) {
        Eigen::Isometry3d collision_offset = Eigen::Isometry3d::Identity();
        collision_offset.translation() << link->collision->origin.position.x,
                                         link->collision->origin.position.y,
                                         link->collision->origin.position.z;
        Eigen::Quaterniond q(link->collision->origin.rotation.w,
                             link->collision->origin.rotation.x,
                             link->collision->origin.rotation.y,
                             link->collision->origin.rotation.z);
        collision_offset.linear() = q.toRotationMatrix();
        
        table_transform = table_transform * collision_offset;
      }
      
      table_collision_objects_[link->name] = std::make_shared<fcl::CollisionObjectd>(geom, table_transform);
    }
  }

  std::shared_ptr<fcl::CollisionGeometryd> createGeometry(urdf::Geometry* geometry, const std::string& package_dir) {
    std::shared_ptr<fcl::CollisionGeometryd> geom;

    switch (geometry->type) {
      case urdf::Geometry::BOX: {
        auto *box = dynamic_cast<urdf::Box *>(geometry);
        geom = std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
        break;
      }
      case urdf::Geometry::CYLINDER: {
        auto *cyl = dynamic_cast<urdf::Cylinder *>(geometry);
        geom = std::make_shared<fcl::Cylinderd>(cyl->radius, cyl->length);
        break;
      }
      case urdf::Geometry::SPHERE: {
        auto *sph = dynamic_cast<urdf::Sphere *>(geometry);
        geom = std::make_shared<fcl::Sphered>(sph->radius);
        break;
      }
      case urdf::Geometry::MESH: {
        auto *mesh = dynamic_cast<urdf::Mesh *>(geometry);
        if (!mesh) break;

        std::string mesh_path = resolveMeshPath(mesh->filename, package_dir);
        const aiScene *scene = importer_.ReadFile(mesh_path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
        if (!scene || !scene->mMeshes || scene->mNumMeshes == 0) {
          RCLCPP_WARN(this->get_logger(), "Failed to load mesh: %s", mesh_path.c_str());
          break;
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
        break;
    }
    return geom;
  }

  void checkCollisions() {
    std::set<std::string> arm_collided_links;
    std::set<std::string> station_collided_links;
    std::set<std::string> table_collided_links;
    std::vector<std::pair<std::string, std::string>> arm_self_collided_pairs;
    std::vector<std::pair<std::string, std::string>> arm_station_collided_pairs;
    std::vector<std::pair<std::string, std::string>> arm_table_collided_pairs;
    std::set<std::pair<std::string, std::string>> adjacent_pairs;
    std::set<std::string> arm_predicted_links;
    std::set<std::string> station_predicted_links;
    std::set<std::string> table_predicted_links;

    for (const auto &joint : arm_model_.joints_) {
      const std::string &parent = joint.second->parent_link_name;
      const std::string &child = joint.second->child_link_name;
      adjacent_pairs.insert({parent, child});
      adjacent_pairs.insert({child, parent});
    }

    fcl::CollisionRequestd request;
    request.enable_contact = true;
    request.num_max_contacts = 100;

    fcl::DistanceRequestd requestd;

    for (const auto& [link1_name, link1_obj] : arm_collision_objects_) {

      for (const auto& [link2_name, link2_obj] : arm_collision_objects_) {
        if (link1_name >= link2_name) continue;

        bool is_adjacent = adjacent_pairs.count({link1_name, link2_name});

        fcl::CollisionResultd result;
        fcl::collide(link1_obj.get(), link2_obj.get(), request, result);

        fcl::DistanceResultd resultd;
        fcl::distance(link1_obj.get(), link2_obj.get(), requestd, resultd);

        // Prediction check
        if (resultd.min_distance < 0.005 && !is_adjacent) {
          arm_predicted_links.insert(link1_name);
          arm_predicted_links.insert(link2_name);
          RCLCPP_WARN(this->get_logger(), "[ARM PREDICT] %s <-> %s, distance = %.3f m",
                      link1_name.c_str(), link2_name.c_str(), resultd.min_distance);
        }

        // Collision check
        if (result.isCollision()) {
          if (is_adjacent) {
            std::vector<fcl::Contactd> contacts;
            result.getContacts(contacts);

            double max_penetration = 0.0;
            for (const auto& contact : contacts) {
              max_penetration = std::max(max_penetration, contact.penetration_depth);
            }

            if (max_penetration < 0.01) continue;  // Ignore minor contact
          }

          arm_collided_links.insert(link1_name);
          arm_collided_links.insert(link2_name);
          arm_self_collided_pairs.emplace_back(link1_name, link2_name);

        }

        if (!arm_self_collided_pairs.empty()) {
          RCLCPP_INFO(this->get_logger(), "[ARM SELF-COLLISION] Links involved:");
          for (const auto &[l1, l2] : arm_self_collided_pairs) {
            RCLCPP_INFO(this->get_logger(), "- %s <-> %s", l1.c_str(), l2.c_str());
          }
        }
      }

      auto table_link_name = "base_link";  // Replace with correct table link
      auto table_obj = table_collision_objects_.at(table_link_name);

      fcl::CollisionResultd table_result;
      fcl::collide(link1_obj.get(), table_obj.get(), request, table_result);

      fcl::DistanceResultd table_resultd;
      fcl::distance(link1_obj.get(), table_obj.get(), requestd, table_resultd);

      if (table_resultd.min_distance < 0.01) {
        arm_predicted_links.insert(link1_name);
        table_predicted_links.insert(table_link_name);
        RCLCPP_WARN(this->get_logger(), "[ARM-TABLE PREDICT] %s <-> %s, dist = %.3f m",
                    link1_name.c_str(), table_link_name, table_resultd.min_distance);
      }

      if (table_result.isCollision()) {
        arm_collided_links.insert(link1_name);
        table_collided_links.insert(table_link_name);
        arm_table_collided_pairs.emplace_back(link1_name, table_link_name);
        if (!arm_table_collided_pairs.empty()) {
          RCLCPP_INFO(this->get_logger(), "[ARM-TABLE COLLISION] Links involved:");
          for (const auto &[arm_link, table_link] : arm_table_collided_pairs) {
            RCLCPP_INFO(this->get_logger(), "- ARM:%s <-> TABLE:%s", arm_link.c_str(), table_link.c_str());
          }
        }
      }


      auto station_link_name = "base_link"; 
      auto station_obj = station_collision_objects_.at(station_link_name);

      fcl::CollisionResultd station_result;
      fcl::collide(link1_obj.get(), station_obj.get(), request, station_result);

      fcl::DistanceResultd station_resultd;
      fcl::distance(link1_obj.get(), station_obj.get(), requestd, station_resultd);

      if (station_resultd.min_distance < 0.05) {
        arm_predicted_links.insert(link1_name);
        station_predicted_links.insert(station_link_name);
        RCLCPP_WARN(this->get_logger(), "[ARM-STATION PREDICT] %s <-> %s, dist = %.3f m",
                    link1_name.c_str(), station_link_name, station_resultd.min_distance);
      }

      if (station_result.isCollision()) {
        arm_collided_links.insert(link1_name);
        station_collided_links.insert(station_link_name);
        arm_station_collided_pairs.emplace_back(link1_name, station_link_name);

        if (!arm_station_collided_pairs.empty()) {
        RCLCPP_INFO(this->get_logger(), "[ARM-STATION COLLISION] Links involved:");
        for (const auto &[arm_link, station_link] : arm_station_collided_pairs) {
          RCLCPP_INFO(this->get_logger(), "- ARM:%s <-> STATION:%s", arm_link.c_str(), station_link.c_str());
        }
      }
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelfCollisionChecker>());
  rclcpp::shutdown();
  return 0;
}