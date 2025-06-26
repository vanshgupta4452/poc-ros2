#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <fcl/fcl.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <set>
#include <string>
#include <iostream>

class SelfCollisionChecker : public rclcpp::Node {
public:
  SelfCollisionChecker() : Node("self_collision_checker") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("self_collision_markers", 10);

    std::string urdf_path = ament_index_cpp::get_package_share_directory("fcl_coll") + "/urdf2/mr_robot.xacro";
    if (!model_.initFile(urdf_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load URDF: %s", urdf_path.c_str());
      return;
    }

    setupCollisionObjects();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&SelfCollisionChecker::checkSelfCollision, this));
  }

private:
  urdf::Model model_;
  std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> collision_objects_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void setupCollisionObjects() {
    for (const auto &pair : model_.links_) {
      const auto &link = pair.second;
      if (!link->collision || !link->collision->geometry) continue;

      std::shared_ptr<fcl::CollisionGeometryd> geom;
      auto origin = link->collision->origin.position;

      if (link->collision->geometry->type == urdf::Geometry::BOX) {
        auto *box = dynamic_cast<urdf::Box*>(link->collision->geometry.get());
        geom = std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
      } else if (link->collision->geometry->type == urdf::Geometry::CYLINDER) {
        auto *cyl = dynamic_cast<urdf::Cylinder*>(link->collision->geometry.get());
        geom = std::make_shared<fcl::Cylinderd>(cyl->radius, cyl->length);
      } else if (link->collision->geometry->type == urdf::Geometry::SPHERE) {
        auto *sph = dynamic_cast<urdf::Sphere*>(link->collision->geometry.get());
        geom = std::make_shared<fcl::Sphered>(sph->radius);
      } else {
        RCLCPP_WARN(this->get_logger(), "Skipping unsupported geometry for link: %s", link->name.c_str());
        continue;
      }

      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation() << origin.x, origin.y, origin.z;
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

    // Print to terminal
    if (!collided_pairs.empty()) {
      RCLCPP_INFO(this->get_logger(), "💥 Collisions detected:");
      for (const auto &[link1, link2] : collided_pairs) {
        RCLCPP_INFO(this->get_logger(), "- %s <--> %s", link1.c_str(), link2.c_str());
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "✅ No self-collision detected.");
    }

    // RViz marker publishing
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    for (const auto &[link_name, obj] : collision_objects_) {
      auto *urdf_link = model_.getLink(link_name).get();
      auto &origin = urdf_link->collision->origin;
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
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
        auto *box = dynamic_cast<urdf::Box*>(geom);
        marker.type = marker.CUBE;
        marker.scale.x = box->dim.x;
        marker.scale.y = box->dim.y;
        marker.scale.z = box->dim.z;
      } else if (geom->type == urdf::Geometry::CYLINDER) {
        auto *cyl = dynamic_cast<urdf::Cylinder*>(geom);
        marker.type = marker.CYLINDER;
        marker.scale.x = cyl->radius * 2;
        marker.scale.y = cyl->radius * 2;
        marker.scale.z = cyl->length;
      } else if (geom->type == urdf::Geometry::SPHERE) {
        auto *sph = dynamic_cast<urdf::Sphere*>(geom);
        marker.type = marker.SPHERE;
        marker.scale.x = sph->radius * 2;
        marker.scale.y = sph->radius * 2;
        marker.scale.z = sph->radius * 2;
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
