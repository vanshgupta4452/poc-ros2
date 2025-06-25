#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>
#include <fcl/fcl.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class FCLSelfCollisionNode : public rclcpp::Node {
public:
  FCLSelfCollisionNode() : Node("fcl_self_collision_node") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "Loading URDF...");

    std::string package_share = ament_index_cpp::get_package_share_directory("fcl_coll");
    std::string urdf_path = package_share + "/urdf/robot_arm.urdf";

    if (!model_.initFile(urdf_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
      return;
    }

    setupCollisionObjects();

    auto sphere_geom = std::make_shared<fcl::Sphered>(0.05);
    Eigen::Isometry3d sphere_tf = Eigen::Isometry3d::Identity();
    sphere_tf.translation() << -0.7, 0.0, 0.2; // start near the robot
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

  void setupCollisionObjects() {
    for (const auto &link_pair : model_.links_) {
      const auto &link = link_pair.second;
      if (!link->collision || !link->collision->geometry) continue;

      std::shared_ptr<fcl::CollisionGeometryd> fcl_geom;

      if (link->collision->geometry->type == urdf::Geometry::BOX) {
        auto box = dynamic_cast<urdf::Box*>(link->collision->geometry.get());
        fcl_geom = std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
      } else if (link->collision->geometry->type == urdf::Geometry::CYLINDER) {
        auto cyl = dynamic_cast<urdf::Cylinder*>(link->collision->geometry.get());
        fcl_geom = std::make_shared<fcl::Cylinderd>(cyl->radius, cyl->length);
      } else if (link->collision->geometry->type == urdf::Geometry::SPHERE) {
        auto sph = dynamic_cast<urdf::Sphere*>(link->collision->geometry.get());
        fcl_geom = std::make_shared<fcl::Sphered>(sph->radius);
      } else {
        RCLCPP_WARN(this->get_logger(), "Unsupported geometry in link: %s", link->name.c_str());
        continue;
      }

      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation() << link->collision->origin.position.x,
                          link->collision->origin.position.y,
                          link->collision->origin.position.z;

      collision_objects_[link->name] = std::make_shared<fcl::CollisionObjectd>(fcl_geom, tf);
    }
  }

  void publishMarkersAndTFs() {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    Eigen::Isometry3d sphere_tf = moving_sphere_->getTransform();
    sphere_tf.translation().x() += 0.01; // move along x
    moving_sphere_->setTransform(sphere_tf);

    bool sphere_collision = false;
    std::string collided_link;

    for (auto& [link_name, obj] : collision_objects_) {
      fcl::CollisionRequestd request;
      fcl::CollisionResultd result;
      fcl::collide(moving_sphere_.get(), obj.get(), request, result);
      if (result.isCollision()) {
        sphere_collision = true;
        collided_link = link_name;
        break;
      }
    }

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

      if (link->collision->geometry->type == urdf::Geometry::BOX) {
        auto box = dynamic_cast<urdf::Box*>(link->collision->geometry.get());
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = box->dim.x;
        marker.scale.y = box->dim.y;
        marker.scale.z = box->dim.z;
      } else if (link->collision->geometry->type == urdf::Geometry::CYLINDER) {
        auto cyl = dynamic_cast<urdf::Cylinder*>(link->collision->geometry.get());
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.scale.x = cyl->radius * 2.0;
        marker.scale.y = cyl->radius * 2.0;
        marker.scale.z = cyl->length;
      } else if (link->collision->geometry->type == urdf::Geometry::SPHERE) {
        auto sph = dynamic_cast<urdf::Sphere*>(link->collision->geometry.get());
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = sph->radius * 2.0;
        marker.scale.y = sph->radius * 2.0;
        marker.scale.z = sph->radius * 2.0;
      } else {
        continue;
      }

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

    // Sphere Marker
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
