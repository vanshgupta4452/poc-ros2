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
    station_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("station_markers", 10);
    table_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("table_markers", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // Subscribe to joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&SelfCollisionChecker::jointStateCallback, this, std::placeholders::_1));

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
  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr station_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr table_marker_pub_;
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


    // Build adjacent pairs for arm (parent-child relationships)
    for (const auto &joint : arm_model_.joints_) {
      const std::string &parent = joint.second->parent_link_name;
      const std::string &child = joint.second->child_link_name;
      adjacent_pairs.insert({parent, child});
      adjacent_pairs.insert({child, parent});
    }

    // Check arm self-collisions
    for (auto it1 = arm_collision_objects_.begin(); it1 != arm_collision_objects_.end(); ++it1) {
      for (auto it2 = std::next(it1); it2 != arm_collision_objects_.end(); ++it2) {
        fcl::CollisionRequestd request;
        request.enable_contact = true;
        request.num_max_contacts = 100;

        fcl::CollisionResultd result;
        fcl::collide(it1->second.get(), it2->second.get(), request, result);

        fcl::DistanceRequestd requestd;
        fcl::DistanceResultd resultd;
        fcl::distance(it1->second.get(), it2->second.get(), requestd, resultd);

        bool is_adjacent = adjacent_pairs.count({it1->first, it2->first});
        
        // Prediction check
        if (resultd.min_distance < 0.01 && !is_adjacent) {
          arm_predicted_links.insert(it1->first);
          arm_predicted_links.insert(it2->first);
          RCLCPP_WARN(this->get_logger(), "[ARM PREDICT] %s <-> %s, distance = %.3f m",
                     it1->first.c_str(), it2->first.c_str(), resultd.min_distance);
        }

        // Collision check
        if (result.isCollision()) {
          if (is_adjacent) {
            std::vector<fcl::Contactd> contacts;
            result.getContacts(contacts);

            double max_penetration = 0.0;
            for (const auto& contact : contacts) {
              if (contact.penetration_depth > max_penetration) {
                max_penetration = contact.penetration_depth;
              }
            }

            if (max_penetration < 0.01) continue;  // Skip if too small
          }
          
          arm_collided_links.insert(it1->first);
          arm_collided_links.insert(it2->first);
          arm_self_collided_pairs.emplace_back(it1->first, it2->first);
        }
      }
    }

    // Check arm-station collisions
    for (const auto& [arm_link_name, arm_obj] : arm_collision_objects_) {
      for (const auto& [station_link_name, station_obj] : station_collision_objects_) {
        fcl::CollisionRequestd request;
        request.enable_contact = true;
        request.num_max_contacts = 100;

        fcl::CollisionResultd result;
        fcl::collide(arm_obj.get(), station_obj.get(), request, result);

        fcl::DistanceRequestd requestd;
        fcl::DistanceResultd resultd;
        fcl::distance(arm_obj.get(), station_obj.get(), requestd, resultd);

        // Prediction check
        if (resultd.min_distance < 0.02) { // 2 cm threshold for arm-station
          arm_predicted_links.insert(arm_link_name);
          station_predicted_links.insert(station_link_name);
          RCLCPP_WARN(this->get_logger(), "[ARM-STATION PREDICT] %s <-> %s, distance = %.3f m",
                     arm_link_name.c_str(), station_link_name.c_str(), resultd.min_distance);
        }

        // Collision check
        if (result.isCollision()) {
          arm_collided_links.insert(arm_link_name);
          station_collided_links.insert(station_link_name);
          arm_station_collided_pairs.emplace_back(arm_link_name, station_link_name);
        }
      }
    }

    for (const auto& [arm_link_name, arm_obj] : arm_collision_objects_) {
      for (const auto& [table_link_name, table_obj] : table_collision_objects_) {
        fcl::CollisionRequestd request;
        request.enable_contact = true;
        request.num_max_contacts = 100;

        fcl::CollisionResultd result;
        fcl::collide(arm_obj.get(), table_obj.get(), request, result);

        fcl::DistanceRequestd requestd;
        fcl::DistanceResultd resultd;
        fcl::distance(arm_obj.get(), table_obj.get(), requestd, resultd);

        
        if (resultd.min_distance < 0.02) { // 2 cm threshold for arm-station
          arm_predicted_links.insert(arm_link_name);
          table_predicted_links.insert(table_link_name);
          RCLCPP_WARN(this->get_logger(), "[ARM-TABLE PREDICT] %s <-> %s, distance = %.3f m",
                     arm_link_name.c_str(), table_link_name.c_str(), resultd.min_distance);
        }

        // Collision check
        if (result.isCollision()) {
          arm_collided_links.insert(arm_link_name);
          table_collided_links.insert(table_link_name);
          arm_table_collided_pairs.emplace_back(arm_link_name, table_link_name);
        }
      }
    }



    // Log collision results
    if (!arm_self_collided_pairs.empty()) {
      RCLCPP_INFO(this->get_logger(), "[ARM SELF-COLLISION] Links involved:");
      for (const auto &[l1, l2] : arm_self_collided_pairs) {
        RCLCPP_INFO(this->get_logger(), "- %s <-> %s", l1.c_str(), l2.c_str());
      }
    }

    if (!arm_station_collided_pairs.empty()) {
      RCLCPP_INFO(this->get_logger(), "[ARM-STATION COLLISION] Links involved:");
      for (const auto &[arm_link, station_link] : arm_station_collided_pairs) {
        RCLCPP_INFO(this->get_logger(), "- ARM:%s <-> STATION:%s", arm_link.c_str(), station_link.c_str());
      }
    }

    if (!arm_table_collided_pairs.empty()) {
      RCLCPP_INFO(this->get_logger(), "[ARM-TABLE COLLISION] Links involved:");
      for (const auto &[arm_link, table_link] : arm_table_collided_pairs) {
        RCLCPP_INFO(this->get_logger(), "- ARM:%s <-> TABLE:%s", arm_link.c_str(), table_link.c_str());
      }
    }

    // Create arm visualization markers
    publishArmMarkers(arm_collided_links, arm_predicted_links);
    
    // Create station visualization markers
    publishStationMarkers(station_collided_links, station_predicted_links);

    publishTableMarkers(table_collided_links, table_predicted_links);


  }

  void publishArmMarkers(const std::set<std::string>& collided_links, const std::set<std::string>& predicted_links) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    
    for (const auto &[link_name, obj] : arm_collision_objects_) {
      if (!arm_link_transforms_.count(link_name)) continue;

      auto urdf_link = arm_model_.getLink(link_name);
      if (!urdf_link || !urdf_link->collision) continue;

      visualization_msgs::msg::Marker marker = createMarker(
        urdf_link, arm_link_transforms_[link_name], id++, "arm_collision", arm_package_share_dir_);
      
      // Set color based on collision status
      if (collided_links.count(link_name)) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      } else if (predicted_links.count(link_name)) {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0; // Orange for prediction
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

  void publishStationMarkers(const std::set<std::string>& collided_links, const std::set<std::string>& predicted_links) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 1000; // Different ID range for station
    
    for (const auto &pair : station_model_.links_) {
      const auto &link = pair.second;
      if (!link->collision || !link->collision->geometry) continue;

      visualization_msgs::msg::Marker marker = createMarker(
        link, Eigen::Isometry3d::Identity(), id++, "station_collision", station_package_share_dir_);
      
      // Set color based on collision status
      if (collided_links.count(link->name)) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      } else if (predicted_links.count(link->name)) {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0; // Orange for prediction
      } else {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0; // Gray for station
      }
      marker.color.a = 0.5;

      marker_array.markers.push_back(marker);
    }

    station_marker_pub_->publish(marker_array);
  }

   void publishTableMarkers(const std::set<std::string>& collided_links, const std::set<std::string>& predicted_links) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 1000; 
    
    for (const auto &pair : table_model_.links_) {
      const auto &link = pair.second;
      if (!link->collision || !link->collision->geometry) continue;

      visualization_msgs::msg::Marker marker = createMarker(
        link, Eigen::Isometry3d::Identity(), id++, "table_collision", table_package_share_dir_);
      
      // Set color based on collision status
      if (collided_links.count(link->name)) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      } else if (predicted_links.count(link->name)) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0; 
      } else {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0; 
      }
      marker.color.a = 0.5;

      marker_array.markers.push_back(marker);
    }

    table_marker_pub_->publish(marker_array);
  }

  visualization_msgs::msg::Marker createMarker(const urdf::LinkConstSharedPtr& urdf_link, 
                                               const Eigen::Isometry3d& link_transform,
                                               int id, const std::string& ns,
                                               const std::string& package_dir) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = "base_link";
    marker.ns = ns;
    marker.id = id;
    marker.action = marker.ADD;

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
    
    Eigen::Isometry3d final_tf = link_transform * collision_offset;
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
      marker.mesh_resource = "file://" + resolveMeshPath(mesh->filename, package_dir);
      marker.scale.x = mesh->scale.x;
      marker.scale.y = mesh->scale.y;
      marker.scale.z = mesh->scale.z;
    }

    return marker;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SelfCollisionChecker>());
  rclcpp::shutdown();
  return 0;
}