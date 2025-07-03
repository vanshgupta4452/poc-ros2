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
#include <map>

class FCLSelfCollisionNode : public rclcpp::Node {
public:
  FCLSelfCollisionNode() : Node("fcl_self_collision_node") {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    std::string package_share = ament_index_cpp::get_package_share_directory("PXA-100_description");
    std::string urdf_path = package_share + "/urdf/PXA-100.xacro";
    package_share_dir_ = package_share;

    if (!model_.initFile(urdf_path)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
      return;
    }

    setupCollisionObjects();

    auto sphere_geom = std::make_shared<fcl::Sphered>(0.05);
    Eigen::Isometry3d sphere_tf = Eigen::Isometry3d::Identity();
    sphere_tf.translation() << -0.31, 0.0, 0.2;
    moving_sphere_ = std::make_shared<fcl::CollisionObjectd>(sphere_geom, sphere_tf);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FCLSelfCollisionNode::publishMarkersAndTFs, this)
    );
  }

private:
  urdf::Model model_;
  std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> collision_objects_;
  std::map<std::string, Eigen::Isometry3d> link_transforms_;
  std::shared_ptr<fcl::CollisionObjectd> moving_sphere_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string package_share_dir_;
  Assimp::Importer importer_;

  Eigen::Isometry3d getFullLinkTransform(const urdf::LinkConstSharedPtr& link) {
    if (!link || !link->parent_joint) return Eigen::Isometry3d::Identity();

    const urdf::Pose& joint_origin = link->parent_joint->parent_to_joint_origin_transform;

    Eigen::Isometry3d joint_tf = Eigen::Isometry3d::Identity();
    joint_tf.translation() << joint_origin.position.x, joint_origin.position.y, joint_origin.position.z;
    Eigen::Quaterniond q(joint_origin.rotation.w, joint_origin.rotation.x, joint_origin.rotation.y, joint_origin.rotation.z);
    joint_tf.linear() = q.toRotationMatrix();

    return getFullLinkTransform(link->getParent()) * joint_tf;
  }

  std::string resolveMeshPath(const std::string& mesh_filename) {
    if (mesh_filename.find("package://") == 0) {
      std::string relative_path = mesh_filename.substr(10);
      size_t first_slash = relative_path.find('/');
      if (first_slash != std::string::npos) {
        std::string package_name = relative_path.substr(0, first_slash);
        std::string file_path = relative_path.substr(first_slash + 1);
        try {
          std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
          return package_path + "/" + file_path;
        } catch (...) {}
      }
    }
    return mesh_filename;
  }

  std::shared_ptr<fcl::CollisionGeometryd> loadMeshGeometry(const std::string& mesh_filename, const urdf::Vector3& scale) {
    std::string resolved_path = resolveMeshPath(mesh_filename);
    if (!std::ifstream(resolved_path)) return nullptr;

    const aiScene* scene = importer_.ReadFile(resolved_path,
        aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_JoinIdenticalVertices);
    if (!scene || !scene->mMeshes || scene->mNumMeshes == 0) return nullptr;

    const aiMesh* mesh = scene->mMeshes[0];
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;

    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
      const aiVector3D& v = mesh->mVertices[i];
      vertices.emplace_back(v.x * scale.x, v.y * scale.y, v.z * scale.z);
    }

    for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
      const aiFace& face = mesh->mFaces[i];
      if (face.mNumIndices == 3)
        triangles.emplace_back(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
    }

    auto bvh_model = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
    bvh_model->beginModel();
    bvh_model->addSubModel(vertices, triangles);
    bvh_model->endModel();
    return bvh_model;
  }

  void setupCollisionObjects() {
    for (const auto& link_pair : model_.links_) {
      const auto& link = link_pair.second;
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
          auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);
          fcl_geom = loadMeshGeometry(mesh->filename, mesh->scale);
          break;
        }
        default: continue;
      }

      Eigen::Isometry3d link_tf = getFullLinkTransform(link);

      Eigen::Isometry3d collision_tf = Eigen::Isometry3d::Identity();
      collision_tf.translation() << link->collision->origin.position.x,
                                   link->collision->origin.position.y,
                                   link->collision->origin.position.z;
      Eigen::Quaterniond q(link->collision->origin.rotation.w,
                           link->collision->origin.rotation.x,
                           link->collision->origin.rotation.y,
                           link->collision->origin.rotation.z);
      collision_tf.linear() = q.toRotationMatrix();

      Eigen::Isometry3d final_tf = link_tf * collision_tf;
      collision_objects_[link->name] = std::make_shared<fcl::CollisionObjectd>(fcl_geom, final_tf);
      link_transforms_[link->name] = final_tf;
    }
  }

  void publishMarkersAndTFs() {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;

    Eigen::Isometry3d sphere_tf = moving_sphere_->getTransform();
    sphere_tf.translation().x() += 0.01;
    moving_sphere_->setTransform(sphere_tf);

    bool sphere_collision = false;
    std::string collided_links;
    std::string collided_links_string;

    for (auto& [link_name, obj] : collision_objects_) {
      fcl::CollisionRequestd request;
      fcl::CollisionResultd result;
      fcl::collide(moving_sphere_.get(), obj.get(), request, result);

      if (result.isCollision()) {
        sphere_collision = true;
        collided_links = link_name;
        RCLCPP_WARN(this->get_logger(), "Collision detected with link: %s", link_name.c_str());
        break;
      }
    }

    


    for (const auto& link_pair : model_.links_) {
      const auto& link = link_pair.second;
      if (!link->collision || !link->collision->geometry) continue;
      if (!link_transforms_.count(link->name)) continue;

      Eigen::Isometry3d tf = link_transforms_[link->name];
      Eigen::Quaterniond q(tf.rotation());

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = this->now();
      marker.ns = "robot";
      marker.id = id++;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = tf.translation().x();
      marker.pose.position.y = tf.translation().y();
      marker.pose.position.z = tf.translation().z();
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();

      // // _INFO(this->get_logger(), "TF for %s: Position = [%.3f, %.3f, %.3f], Quaternion = [RCLCPP%.3f, %.3f, %.3f, %.3f]",
      //       link->name.c_str(),
      //       tf.translation().x(), tf.translation().y(), tf.translation().z(),
      //       q.x(), q.y(), q.z(), q.w());


      switch (link->collision->geometry->type) {
        case urdf::Geometry::BOX: {
          auto box = dynamic_cast<urdf::Box*>(link->collision->geometry.get());
          marker.type = marker.CUBE;
          marker.scale.x = box->dim.x;
          marker.scale.y = box->dim.y;
          marker.scale.z = box->dim.z;
          break;
        }
        case urdf::Geometry::CYLINDER: {
          auto cyl = dynamic_cast<urdf::Cylinder*>(link->collision->geometry.get());
          marker.type = marker.CYLINDER;
          marker.scale.x = cyl->radius * 2;
          marker.scale.y = cyl->radius * 2;
          marker.scale.z = cyl->length;
          break;
        }
        case urdf::Geometry::SPHERE: {
          auto sph = dynamic_cast<urdf::Sphere*>(link->collision->geometry.get());
          marker.type = marker.SPHERE;
          marker.scale.x = marker.scale.y = marker.scale.z = sph->radius * 2;
          break;
        }
        case urdf::Geometry::MESH: {
          auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);
          marker.type = marker.MESH_RESOURCE;
          marker.mesh_resource = "file://" + resolveMeshPath(mesh->filename);
          marker.scale.x = mesh->scale.x;
          marker.scale.y = mesh->scale.y;
          marker.scale.z = mesh->scale.z;
          break;
        }
        default: continue;
      }

     if (sphere_collision && link->name == collided_links) {
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
      // double x=link->collision->origin.position.x;
      // double y=link->collision->origin.position.y;

      // double z=link->collision->origin.position.z;

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = this->now();
      tf_msg.header.frame_id = "base_link";
      tf_msg.child_frame_id = link->name;
    
      tf_msg.transform.translation.x = link->collision->origin.position.x;
      tf_msg.transform.translation.y = -link->collision->origin.position.y;
      tf_msg.transform.translation.z = -link->collision->origin.position.z;
      tf_msg.transform.rotation.x = link->collision->origin.rotation.x;
      tf_msg.transform.rotation.y = link->collision->origin.rotation.y;
      tf_msg.transform.rotation.z = link->collision->origin.rotation.z;
      tf_msg.transform.rotation.w = link->collision->origin.rotation.w;
      tf_broadcaster_->sendTransform(tf_msg);
      RCLCPP_INFO(this->get_logger(), "Link name: testing haiu bhai %s", link->name.c_str());

    }

    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header.frame_id = "base_link";
    sphere_marker.header.stamp = this->now();
    sphere_marker.ns = "sphere";
    sphere_marker.id = id++;
    sphere_marker.type = sphere_marker.SPHERE;
    sphere_marker.action = sphere_marker.ADD;
    sphere_marker.scale.x = 0.1;
    sphere_marker.scale.y = 0.1;
    sphere_marker.scale.z = 0.1;
    sphere_marker.pose.position.x = sphere_tf.translation().x();
    sphere_marker.pose.position.y = sphere_tf.translation().y();
    sphere_marker.pose.position.z = sphere_tf.translation().z();
    sphere_marker.pose.orientation.w = 1.0;

    if (sphere_collision) {
      sphere_marker.color.r = 1.0;
      sphere_marker.color.g = 0.0;
      sphere_marker.color.b = 0.0;
    } else {
      sphere_marker.color.r = 0.0;
      sphere_marker.color.g = 1.0;
      sphere_marker.color.b = 0.0;
    }
    sphere_marker.color.a = 1.0;
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
