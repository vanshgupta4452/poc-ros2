#include <rclcpp/rclcpp.hpp>
#include <fcl/fcl.h>
#include <visualization_msgs/msg/marker_array.hpp>

class CollisionChecker : public rclcpp::Node {
public:
  CollisionChecker() : Node("collision_checker"), step_(0.0), direction_(1) {
    RCLCPP_INFO(this->get_logger(), "Collision Checker with Animation Started");

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("collision_markers", 10);

    // Static box at (0, 0, 0.5)
    object1_ = std::make_shared<fcl::CollisionObjectd>(
      std::make_shared<fcl::Boxd>(1.0, 1.0, 1.0),
      fcl::Transform3d(Eigen::Translation3d(0.0, 0.0, 0.5))
    );

    // Moving sphere — initial position far
    object2_ = std::make_shared<fcl::CollisionObjectd>(
      std::make_shared<fcl::Sphered>(0.5),
      fcl::Transform3d(Eigen::Translation3d(0.0, 0.0, 3.0))
    );

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&CollisionChecker::checkCollision, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::shared_ptr<fcl::CollisionObjectd> object1_;
  std::shared_ptr<fcl::CollisionObjectd> object2_;
  double step_;
  int direction_;  // 1 for moving closer, -1 for moving away

  void checkCollision() {
    // Move sphere in Z-axis toward and away
    step_ += direction_ * 0.01;
    if (step_ >= 2.5) direction_ = -1;
    if (step_ <= 0.0) direction_ = 1;

    object2_->setTransform(fcl::Transform3d(Eigen::Translation3d(0.0, 0.0, 3.0 - step_)));

    fcl::CollisionRequestd request;
    request.enable_contact = true;
    request.num_max_contacts = 100;

    fcl::CollisionResultd result;
    fcl::collide(object1_.get(), object2_.get(), request, result);

    visualization_msgs::msg::MarkerArray markers;

    // Box marker
    markers.markers.push_back(createBoxMarker(object1_->getTransform(), 0, result.isCollision()));
    // Sphere marker
    markers.markers.push_back(createSphereMarker(object2_->getTransform(), 1, result.isCollision()));

    // Contact markers
    std::vector<fcl::Contactd> contacts;
    result.getContacts(contacts);

    int id = 100;
    for (const auto& contact : contacts) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = this->now();
      m.ns = "contacts";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = contact.pos[0];
      m.pose.position.y = contact.pos[1];
      m.pose.position.z = contact.pos[2];
      m.scale.x = m.scale.y = m.scale.z = 0.05;
      m.color.r = 0.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      markers.markers.push_back(m);
    }

    marker_pub_->publish(markers);
  }

  visualization_msgs::msg::Marker createBoxMarker(const fcl::Transform3d& tf, int id, bool collision) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "objects";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = tf.translation().x();
    marker.pose.position.y = tf.translation().y();
    marker.pose.position.z = tf.translation().z();

    Eigen::Quaterniond q(tf.rotation());
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = collision ? 1.0 : 0.0;
    marker.color.g = collision ? 0.0 : 0.5;
    marker.color.b = collision ? 0.0 : 1.0;
    marker.color.a = 0.8;

    return marker;
  }

  visualization_msgs::msg::Marker createSphereMarker(const fcl::Transform3d& tf, int id, bool collision) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "objects";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = tf.translation().x();
    marker.pose.position.y = tf.translation().y();
    marker.pose.position.z = tf.translation().z();

    Eigen::Quaterniond q(tf.rotation());
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = collision ? 1.0 : 0.2;
    marker.color.g = collision ? 0.0 : 0.8;
    marker.color.b = collision ? 0.0 : 0.2;
    marker.color.a = 0.8;

    return marker;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionChecker>());
  rclcpp::shutdown();
  return 0;
}
