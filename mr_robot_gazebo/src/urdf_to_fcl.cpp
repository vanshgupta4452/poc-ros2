#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <urdf/model.h>
#include <tf2/LinearMath/Quaternion.h>

class URDFVisualizer : public rclcpp::Node {
public:
    URDFVisualizer() : Node("urdf_to_fcl_node") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("urdf_visualization", 10);
        this->declare_parameter<std::string>("robot_description_param", "/robot_description");
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&URDFVisualizer::loadAndPublish, this));
    }

private:
    void loadAndPublish() {
        std::string urdf_string;
        if (!this->get_parameter("robot_description_param", urdf_param_name_)) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_description_param' not set.");
            return;
        }

        this->get_parameter_or<std::string>(urdf_param_name_, urdf_string, "");

        if (urdf_string.empty()) {
            RCLCPP_ERROR(this->get_logger(), "URDF string is empty.");
            return;
        }

        urdf::Model model;
        if (!model.initString(urdf_string)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        for (const auto& link_pair : model.links_) {
            const auto& link = link_pair.second;
            if (!link->collision || !link->collision->geometry) continue;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";  // or link->name
            marker.header.stamp = this->now();
            marker.ns = "urdf";
            marker.id = id++;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.lifetime = rclcpp::Duration::from_seconds(0);

            // Geometry types
            if (link->collision->geometry->type == urdf::Geometry::BOX) {
                auto box = std::dynamic_pointer_cast<urdf::Box>(link->collision->geometry);
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.scale.x = box->dim.x;
                marker.scale.y = box->dim.y;
                marker.scale.z = box->dim.z;
            } else if (link->collision->geometry->type == urdf::Geometry::SPHERE) {
                auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(link->collision->geometry);
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.scale.x = marker.scale.y = marker.scale.z = 2 * sphere->radius;
            } else if (link->collision->geometry->type == urdf::Geometry::CYLINDER) {
                auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(link->collision->geometry);
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.scale.x = marker.scale.y = 2 * cyl->radius;
                marker.scale.z = cyl->length;
            } else {
                RCLCPP_WARN(this->get_logger(), "Unsupported geometry type in link %s", link->name.c_str());
                continue;
            }

            // Position
            marker.pose.position.x = link->collision->origin.position.x;
            marker.pose.position.y = link->collision->origin.position.y;
            marker.pose.position.z = link->collision->origin.position.z;

            // Orientation
            tf2::Quaternion q;
            double x = link->collision->origin.rotation.x;
            double y = link->collision->origin.rotation.y;
            double z = link->collision->origin.rotation.z;
            double w = link->collision->origin.rotation.w;
            q = tf2::Quaternion(x, y, z, w);
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            // Color
            marker.color.r = 0.1f;
            marker.color.g = 0.7f;
            marker.color.b = 0.2f;
            marker.color.a = 0.8f;

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string urdf_param_name_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<URDFVisualizer>());
    rclcpp::shutdown();
    return 0;
}
