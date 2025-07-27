#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>
#include <fstream>
#include <string>
#include <map>
#include <vector>

class URDFVisualizationMarker : public rclcpp::Node {
public:
  URDFVisualizationMarker() : Node("urdf_visualization_marker") {
    
    // Publishers for different models
    arm_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("arm_visual_markers", 10);
    station_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("station_visual_markers", 10);
    table_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("table_visual_markers", 10);
    
    // Subscribe to joint states for arm kinematics
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&URDFVisualizationMarker::jointStateCallback, this, std::placeholders::_1));

    // Load URDF models
    loadURDFModels();
    
    // Setup TF listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timer for publishing markers
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                    std::bind(&URDFVisualizationMarker::publishMarkers, this));
                                  
    RCLCPP_INFO(this->get_logger(), "URDF Visualization Marker node initialized");
  }

private:
  // URDF models
  urdf::Model arm_model_;
  urdf::Model station_model_;
  urdf::Model table_model_;
  
  // Current joint positions
  std::map<std::string, double> current_joint_positions_;
  
  // Link transforms for arm (updated with joint states)
  std::map<std::string, Eigen::Isometry3d> arm_link_transforms_;
  
  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr arm_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr station_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr table_marker_pub_;
  
  // Subscribers and timers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Package directories
  std::string arm_package_share_dir_;
  std::string station_package_share_dir_;
  std::string table_package_share_dir_;

  void loadURDFModels() {
    // Load arm URDF
    try {
      arm_package_share_dir_ = ament_index_cpp::get_package_share_directory("ajgar_description");
      std::string arm_urdf_path = arm_package_share_dir_ + "/urdf/arm.urdf";
      if (!arm_model_.initFile(arm_urdf_path)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load arm URDF: %s", arm_urdf_path.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully loaded arm URDF");
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Could not load arm URDF: %s", e.what());
    }

    // Load station URDF
    try {
      station_package_share_dir_ = ament_index_cpp::get_package_share_directory("Station_description");
      std::string station_urdf_path = station_package_share_dir_ + "/urdf/Station.urdf";
      if (!station_model_.initFile(station_urdf_path)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load station URDF: %s", station_urdf_path.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully loaded station URDF");
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Could not load station URDF: %s", e.what());
    }

    // Load table URDF
    try {
      table_package_share_dir_ = ament_index_cpp::get_package_share_directory("fixture_table_description");
      std::string table_urdf_path = table_package_share_dir_ + "/urdf/fixture_table.urdf";
      if (!table_model_.initFile(table_urdf_path)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load table URDF: %s", table_urdf_path.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully loaded table URDF");
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Could not load table URDF: %s", e.what());
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Update joint positions
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
  }

  void updateChildLinkTransforms(const urdf::LinkConstSharedPtr& parent_link, const Eigen::Isometry3d& parent_transform) {
    for (const auto& joint_pair : arm_model_.joints_) {
      const auto& joint = joint_pair.second;
      
      if (joint->parent_link_name == parent_link->name) {
        Eigen::Isometry3d joint_transform = Eigen::Isometry3d::Identity();
        
        // Apply joint origin transform
        const urdf::Pose& joint_origin = joint->parent_to_joint_origin_transform;
        joint_transform.translation() << joint_origin.position.x, joint_origin.position.y, joint_origin.position.z;
        Eigen::Quaterniond q(joint_origin.rotation.w, joint_origin.rotation.x, joint_origin.rotation.y, joint_origin.rotation.z);
        joint_transform.linear() = q.toRotationMatrix();
        
        // Apply joint motion
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
      try {
        return ament_index_cpp::get_package_share_directory(pkg) + "/" + file;
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Package not found: %s", pkg.c_str());
        return "";
      }
    } else if (mesh_filename.find("file://") == 0) {
      return mesh_filename.substr(7);
    } else {
      return package_dir + "/" + mesh_filename;
    }
  }

  void publishMarkers() {
    publishArmMarkers();
    publishStationMarkers();
    publishTableMarkers();
  }

  void publishArmMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    
    for (const auto &pair : arm_model_.links_) {
      const auto &link = pair.second;
      
      // Create visual markers
      if (link->visual && link->visual->geometry) {
        visualization_msgs::msg::Marker marker = createVisualMarker(
          link, arm_link_transforms_[link->name], id++, "arm_visual", arm_package_share_dir_, true);
        marker_array.markers.push_back(marker);
      }
      
      // Create collision markers (optional - can be enabled/disabled)
      if (link->collision && link->collision->geometry) {
        visualization_msgs::msg::Marker collision_marker = createCollisionMarker(
          link, arm_link_transforms_[link->name], id++, "arm_collision", arm_package_share_dir_);
        marker_array.markers.push_back(collision_marker);
      }
    }

    arm_marker_pub_->publish(marker_array);
  }

  void publishStationMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 1000; // Different ID range
    
    for (const auto &pair : station_model_.links_) {
      const auto &link = pair.second;
      
      if (link->visual && link->visual->geometry) {
        visualization_msgs::msg::Marker marker = createVisualMarker(
          link, Eigen::Isometry3d::Identity(), id++, "station_visual", station_package_share_dir_, false);
        marker_array.markers.push_back(marker);
      }
    }

    station_marker_pub_->publish(marker_array);
  }

  void publishTableMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 2000; // Different ID range
    
    for (const auto &pair : table_model_.links_) {
      const auto &link = pair.second;
      
      if (link->visual && link->visual->geometry) {
        visualization_msgs::msg::Marker marker = createVisualMarker(
          link, Eigen::Isometry3d::Identity(), id++, "table_visual", table_package_share_dir_, false);
        marker_array.markers.push_back(marker);
      }
    }

    table_marker_pub_->publish(marker_array);
  }

  visualization_msgs::msg::Marker createVisualMarker(const urdf::LinkConstSharedPtr& urdf_link, 
                                                    const Eigen::Isometry3d& link_transform,
                                                    int id, const std::string& ns,
                                                    const std::string& package_dir,
                                                    bool use_link_transform) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = "base_link";
    marker.ns = ns;
    marker.id = id;
    marker.action = marker.ADD;

    // Apply visual origin offset
    Eigen::Isometry3d visual_offset = Eigen::Isometry3d::Identity();
    if (urdf_link->visual) {
      visual_offset.translation() << urdf_link->visual->origin.position.x,
                                    urdf_link->visual->origin.position.y,
                                    urdf_link->visual->origin.position.z;
      Eigen::Quaterniond q(urdf_link->visual->origin.rotation.w,
                          urdf_link->visual->origin.rotation.x,
                          urdf_link->visual->origin.rotation.y,
                          urdf_link->visual->origin.rotation.z);
      visual_offset.linear() = q.toRotationMatrix();
    }
    
    Eigen::Isometry3d final_tf;
    if (use_link_transform) {
      final_tf = link_transform * visual_offset;
    } else {
      final_tf = visual_offset;
    }
    
    Eigen::Vector3d pos = final_tf.translation();
    Eigen::Quaterniond quat(final_tf.rotation());

    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = pos.z();
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    // Set geometry from visual
    auto *geom = urdf_link->visual->geometry.get();
    setMarkerGeometry(marker, geom, package_dir);

    // Set material color if available
    if (urdf_link->visual->material) {
      marker.color.r = urdf_link->visual->material->color.r;
      marker.color.g = urdf_link->visual->material->color.g;
      marker.color.b = urdf_link->visual->material->color.b;
      marker.color.a = urdf_link->visual->material->color.a;
    } else {
      // Default color
      marker.color.r = 0.7;
      marker.color.g = 0.7;
      marker.color.b = 0.7;
      marker.color.a = 1.0;
    }

    return marker;
  }

  visualization_msgs::msg::Marker createCollisionMarker(const urdf::LinkConstSharedPtr& urdf_link, 
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

    // Set geometry from collision
    auto *geom = urdf_link->collision->geometry.get();
    setMarkerGeometry(marker, geom, package_dir);

    // Collision markers are semi-transparent red
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3;

    return marker;
  }

  void setMarkerGeometry(visualization_msgs::msg::Marker& marker, urdf::Geometry* geom, const std::string& package_dir) {
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
      std::string mesh_path = resolveMeshPath(mesh->filename, package_dir);
      if (!mesh_path.empty()) {
        marker.mesh_resource = "file://" + mesh_path;
        marker.scale.x = mesh->scale.x;
        marker.scale.y = mesh->scale.y;
        marker.scale.z = mesh->scale.z;
      } else {
        // Fallback to cube if mesh not found
        marker.type = marker.CUBE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<URDFVisualizationMarker>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}