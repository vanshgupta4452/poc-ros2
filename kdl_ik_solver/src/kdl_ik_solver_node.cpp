#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <Eigen/Dense>

using namespace KDL;
using namespace std::chrono_literals;

// Custom position-only IK solver (fixed version)
class PositionOnlyIKSolver {
private:
    Chain chain_;
    ChainFkSolverPos_recursive fk_solver_;
    std::vector<std::pair<double, double>> joint_limits_;
    
public:
    PositionOnlyIKSolver(const Chain& chain) : chain_(chain), fk_solver_(chain) {}
    
    void setJointLimits(const std::vector<std::pair<double, double>>& limits) {
        joint_limits_ = limits;
    }
    
    int solvePositionIK(const Vector& target_pos, const JntArray& q_init, JntArray& q_result, 
                       double tolerance = 1e-4, int max_iterations = 1000) {
        q_result = q_init;
        
        for (int iter = 0; iter < max_iterations; ++iter) {
            Frame current_frame;
            if (fk_solver_.JntToCart(q_result, current_frame) < 0) {
                return -1;
            }
            
            Vector current_pos = current_frame.p;
            Vector pos_error = target_pos - current_pos;
            
            if (pos_error.Norm() < tolerance) {
                return iter;
            }
            
            Jacobian jac(chain_.getNrOfJoints());
            if (computePositionJacobian(q_result, jac) < 0) {
                return -2;
            }
            
            JntArray dq(chain_.getNrOfJoints());
            if (solvePseudoInverse(jac, pos_error, dq) < 0) {
                return -3;
            }
            
            double step_size = 0.1;
            for (unsigned int i = 0; i < q_result.rows(); ++i) {
                q_result(i) += step_size * dq(i);
                
                // Apply joint limits after updating the joint position
                if (!joint_limits_.empty() && i < joint_limits_.size()) {
                    q_result(i) = std::clamp(q_result(i), joint_limits_[i].first, joint_limits_[i].second);
                }
            }
        }
        
        return -4;
    }
    
private:
    int computePositionJacobian(const JntArray& q, Jacobian& jac) {
        double delta = 1e-6;
        Frame frame_plus, frame_minus;
        JntArray q_plus = q, q_minus = q;
        
        for (unsigned int i = 0; i < q.rows(); ++i) {
            q_plus(i) = q(i) + delta;
            q_minus(i) = q(i) - delta;
            
            if (fk_solver_.JntToCart(q_plus, frame_plus) < 0 || 
                fk_solver_.JntToCart(q_minus, frame_minus) < 0) {
                return -1;
            }
            
            Vector dp = (frame_plus.p - frame_minus.p) / (2.0 * delta);
            jac(0, i) = dp.x();
            jac(1, i) = dp.y();
            jac(2, i) = dp.z();
            
            q_plus(i) = q(i);
            q_minus(i) = q(i);
        }
        
        return 0;
    }
    
    int solvePseudoInverse(const KDL::Jacobian& jac, const KDL::Vector& error, KDL::JntArray& dq) {
        unsigned int nj = chain_.getNrOfJoints();

        // Convert KDL Jacobian to Eigen matrix
        Eigen::Matrix<double, 3, Eigen::Dynamic> J(3, nj);
        for (unsigned int i = 0; i < nj; ++i) {
            J(0, i) = jac(0, i);
            J(1, i) = jac(1, i);
            J(2, i) = jac(2, i);
        }

        // Error vector
        Eigen::Vector3d e(error.x(), error.y(), error.z());

        // Step-by-step evaluation to avoid temporary expression issues
        Eigen::Matrix3d JJT = J * J.transpose();                             // 3x3 concrete
        Eigen::Matrix3d damping = 1e-6 * Eigen::Matrix3d::Identity();        // 3x3 concrete
        Eigen::Matrix3d JJT_damped = JJT + damping;                          // 3x3 concrete
        Eigen::Matrix3d JJT_inv = JJT_damped.inverse();                      // inverse() now safe

        Eigen::MatrixXd J_pinv = J.transpose() * JJT_inv;                    // (n x 3)
        Eigen::VectorXd dq_e = J_pinv * e;                                   // (n x 1)

        // Convert result to KDL::JntArray
        dq.resize(nj);
        for (unsigned int i = 0; i < nj; ++i) {
            dq(i) = dq_e(i);
        }

        return 0;
    }
};

class IKVisualizerNode : public rclcpp::Node {
public:
    std::vector<std::pair<double, double>> joint_limits_;

    IKVisualizerNode() : Node("ik_visualizer"), 
                         tf_broadcaster_(this),
                         static_tf_broadcaster_(this),
                         first_solution_received_(false),
                         reached_target_(true),
                         suppress_initial_teleport_(true) {

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_markers", 10);

        if (!loadRobotModel()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model");
            return;
        }

        fk_solver_ = std::make_unique<ChainFkSolverPos_recursive>(kdl_chain_);
        pos_ik_solver_ = std::make_unique<PositionOnlyIKSolver>(kdl_chain_);
        
        // Set joint limits for the IK solver
        pos_ik_solver_->setJointLimits(joint_limits_);

        current_joint_positions_ = JntArray(kdl_chain_.getNrOfJoints());
        target_joint_positions_ = JntArray(kdl_chain_.getNrOfJoints());

        generateWorkspaceSamples();
        timer_ = this->create_wall_timer(50ms, std::bind(&IKVisualizerNode::timerCallback, this));
        initializeTestTargets();

        RCLCPP_INFO(this->get_logger(), "IK Visualizer Node initialized successfully");
    }

private:
    bool loadRobotModel() {
        std::string urdf_path = "/home/vansh/intern-ardee/src/fcl-ros2/PXA-100_description/urdf/px.urdf";
        
        std::ifstream urdf_file(urdf_path);
        if (!urdf_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file: %s", urdf_path.c_str());
            return false;
        }
        
        std::stringstream buffer;
        buffer << urdf_file.rdbuf();
        std::string urdf_str = buffer.str();
        urdf_file.close();
        
        urdf::Model robot_model;
        if (!robot_model.initString(urdf_str)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
            return false;
        }
        
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert URDF to KDL Tree");
            return false;
        }
        
        std::string base_link = "base_link";
        std::string tip_link = "end";
        
        if (!kdl_tree.getChain(base_link, tip_link, kdl_chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract chain from %s to %s", 
                        base_link.c_str(), tip_link.c_str());
            return false;
        }
        
        // Initialize joint names
        joint_names_ = {"Revolute 18", "Revolute 19", "Revolute 20", "Revolute 15" };

        // Extract joint limits from URDF
        for (const auto& joint_name : joint_names_) {
            auto joint = robot_model.getJoint(joint_name);
            if (joint && joint->type != urdf::Joint::FIXED) {
                if (joint->limits) {
                    joint_limits_.emplace_back(joint->limits->lower, joint->limits->upper);
                    RCLCPP_INFO(this->get_logger(), "Joint %s limits: [%.3f, %.3f]", 
                               joint_name.c_str(), joint->limits->lower, joint->limits->upper);
                } else {
                    // If no limits defined, assume full rotation (like continuous)
                    joint_limits_.emplace_back(-M_PI, M_PI);
                    RCLCPP_INFO(this->get_logger(), "Joint %s: no limits found, using [-π, π]", 
                               joint_name.c_str());
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded robot model with %d joints", 
                   kdl_chain_.getNrOfJoints());
        return true;
    }
    
    void generateWorkspaceSamples() {
        RCLCPP_INFO(this->get_logger(), "Generating workspace samples...");
        
        srand(42);
        int num_samples = 2000;
        
        for (int i = 0; i < num_samples; ++i) {
            JntArray q(kdl_chain_.getNrOfJoints());
            
            // Generate random joint angles within joint limits
            for (unsigned int j = 0; j < q.rows(); ++j) {
                if (j < joint_limits_.size()) {
                    double lower = joint_limits_[j].first;
                    double upper = joint_limits_[j].second;
                    q(j) = lower + ((double)rand() / RAND_MAX) * (upper - lower);
                } else {
                    q(j) = ((double)rand() / RAND_MAX - 0.5) * 2 * M_PI;
                }
            }
            
            Frame result;
            if (fk_solver_->JntToCart(q, result) >= 0) {
                workspace_points_.push_back(result.p);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Generated %zu workspace points", workspace_points_.size());
    }
    
    void initializeTestTargets() {
        Frame current_frame;
        // Original target and closest reachable point
        Vector original_target(0.1, 0.1, 0.05);
        Vector base_pos = current_frame.p;

        // test_targets_ = {
        //     base_pos,
        //     Vector(0.1, -0.13, 0.19),
        //     Vector(0.1, -0.12, 0.20),
         
        //     findClosestReachablePoint(Vector(0.1, 0.1, 0.05))
        // };
        test_targets_ = {
                    Vector(0.2, 0.0, 0.3),
                    Vector(0.1, 0.1, 0.2),
                    Vector(0.15, -0.05, 0.25),
                  
                    Vector(0.1, -0.1, 0.3)
                };
        
        current_target_index_ = 0;
        target_update_counter_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Initialized %zu test targets", test_targets_.size());
    }
    
    Vector findClosestReachablePoint(const Vector& target) {
        if (workspace_points_.empty()) return Vector::Zero();
        
        Vector closest = workspace_points_[0];
        double min_distance = (target - closest).Norm();
        
        for (const auto& point : workspace_points_) {
            double distance = (target - point).Norm();
            if (distance < min_distance) {
                min_distance = distance;
                closest = point;
            }
        }
        
        return closest;
    }
    
    // Method to print position error
    void printPositionError(const Vector& target, const JntArray& joint_positions) {
        Frame end_effector_frame;
        if (fk_solver_->JntToCart(joint_positions, end_effector_frame) >= 0) {
            Vector current_pos = end_effector_frame.p;
            Vector error = target - current_pos;
            double error_magnitude = error.Norm();
            
            RCLCPP_INFO(this->get_logger(), 
                       "Target: [%.4f, %.4f, %.4f] | Current: [%.4f, %.4f, %.4f] | Error: %.6f m",
                       target.x(), target.y(), target.z(),
                       current_pos.x(), current_pos.y(), current_pos.z(),
                       error_magnitude);
        }
    }

    // Throttled version - prints error every 500ms
    void printPositionErrorThrottled(const Vector& target, const JntArray& joint_positions) {
        Frame end_effector_frame;
        if (fk_solver_->JntToCart(joint_positions, end_effector_frame) >= 0) {
            Vector current_pos = end_effector_frame.p;
            Vector error = target - current_pos;
            double error_magnitude = error.Norm();
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                       "Target: [%.4f, %.4f, %.4f] | Current: [%.4f, %.4f, %.4f] | Error: %.6f m",
                       target.x(), target.y(), target.z(),
                       current_pos.x(), current_pos.y(), current_pos.z(),
                       error_magnitude);
        }
    }
    
    void timerCallback() {
        bool updated = false;
        for (unsigned int i = 0; i < current_joint_positions_.rows(); ++i) {
            double delta = target_joint_positions_(i) - current_joint_positions_(i);
            double step = M_PI / 180.0; // 1 degree in radians
            if (std::abs(delta) > step) {
                current_joint_positions_(i) += (delta > 0 ? 1 : -1) * step;
                updated = true;
            } else if(delta<=0.001) {
                current_joint_positions_(i) = target_joint_positions_(i);
            }
            else{
                current_joint_positions_(i) += (delta > 0 ? 0.01 : -0.01) * step;
                updated = true;
            }
        }

        if (!(suppress_initial_teleport_ && !first_solution_received_)) {
            publishJointStates(current_joint_positions_);
        }

        Vector current_target = test_targets_[current_target_index_];
        publishVisualizationMarkers(current_target, current_joint_positions_);
        
        // Print position error (throttled version to avoid spam)
        printPositionErrorThrottled(current_target, current_joint_positions_);

        if (!updated && reached_target_) {
            reached_target_ = false;
            current_target_index_ = (current_target_index_ + 1) % test_targets_.size();
            RCLCPP_INFO(this->get_logger(), "Switching to target %d", current_target_index_ + 1);

            Vector next_target = test_targets_[current_target_index_];
            JntArray q_init = current_joint_positions_;
            JntArray q_result(kdl_chain_.getNrOfJoints());

            int result = pos_ik_solver_->solvePositionIK(next_target, q_init, q_result);

            if (result >= 0) {
                target_joint_positions_ = q_result;
                if (!first_solution_received_) {
                    current_joint_positions_ = target_joint_positions_;
                    first_solution_received_ = true;
                    suppress_initial_teleport_ = false;
                }
                RCLCPP_INFO(this->get_logger(), "IK converged in %d iterations for target %d", 
                           result, current_target_index_ + 1);
            } else {
                // Retry with zero initialization as fallback
                JntArray fallback_q(kdl_chain_.getNrOfJoints());
                for (unsigned int i = 0; i < fallback_q.rows(); ++i) {
                    // Initialize within joint limits
                    if (i < joint_limits_.size()) {
                        fallback_q(i) = (joint_limits_[i].first + joint_limits_[i].second) / 2.0;
                    } else {
                        fallback_q(i) = 0.0;
                    }
                }

                if (pos_ik_solver_->solvePositionIK(next_target, fallback_q, q_result) >= 0) {
                    RCLCPP_WARN(this->get_logger(), "Fallback IK succeeded for target %d", current_target_index_ + 1);
                    target_joint_positions_ = q_result;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                        "IK failed for target %d with code: %d", current_target_index_ + 1, result);
                }
            }
        } else if (!updated) {
            reached_target_ = true;
        }
    }
    
    void publishJointStates(const JntArray& joint_positions) {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.header.frame_id = "base_link";
        
        joint_state_msg.name = joint_names_;
        joint_state_msg.position.resize(joint_positions.rows());
        joint_state_msg.velocity.resize(joint_positions.rows());
        joint_state_msg.effort.resize(joint_positions.rows());
        
        for (unsigned int i = 0; i < joint_positions.rows(); ++i) {
            joint_state_msg.position[i] = joint_positions(i);
            joint_state_msg.velocity[i] = 0.0;
            joint_state_msg.effort[i] = 0.0;
        }
        
        joint_state_pub_->publish(joint_state_msg);
    }
    
    void publishVisualizationMarkers(const Vector& target, const JntArray& joint_positions) {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Target position marker
        auto target_marker = visualization_msgs::msg::Marker();
        target_marker.header.frame_id = "base_link";
        target_marker.header.stamp = this->get_clock()->now();
        target_marker.ns = "targets";
        target_marker.id = 0;
        target_marker.type = visualization_msgs::msg::Marker::SPHERE;
        target_marker.action = visualization_msgs::msg::Marker::ADD;
        
        target_marker.pose.position.x = target.x();
        target_marker.pose.position.y = target.y();
        target_marker.pose.position.z = target.z();
        target_marker.pose.orientation.w = 1.0;
        
        target_marker.scale.x = 0.02;
        target_marker.scale.y = 0.02;
        target_marker.scale.z = 0.02;
        
        target_marker.color.r = 1.0;
        target_marker.color.g = 0.0;
        target_marker.color.b = 0.0;
        target_marker.color.a = 1.0;
        
        marker_array.markers.push_back(target_marker);
        
        // End-effector position marker
        Frame end_effector_frame;
        fk_solver_->JntToCart(joint_positions, end_effector_frame);
        
        auto ee_marker = visualization_msgs::msg::Marker();
        ee_marker.header.frame_id = "base_link";
        ee_marker.header.stamp = this->get_clock()->now();
        ee_marker.ns = "end_effector";
        ee_marker.id = 0;
        ee_marker.type = visualization_msgs::msg::Marker::SPHERE;
        ee_marker.action = visualization_msgs::msg::Marker::ADD;
        
        ee_marker.pose.position.x = end_effector_frame.p.x();
        ee_marker.pose.position.y = end_effector_frame.p.y();
        ee_marker.pose.position.z = end_effector_frame.p.z();
        ee_marker.pose.orientation.w = 1.0;
        
        ee_marker.scale.x = 0.015;
        ee_marker.scale.y = 0.015;
        ee_marker.scale.z = 0.015;
        
        ee_marker.color.r = 0.0;
        ee_marker.color.g = 1.0;
        ee_marker.color.b = 0.0;
        ee_marker.color.a = 1.0;
        
        marker_array.markers.push_back(ee_marker);
        
        // Line connecting target and end-effector
        auto line_marker = visualization_msgs::msg::Marker();
        line_marker.header.frame_id = "base_link";
        line_marker.header.stamp = this->get_clock()->now();
        line_marker.ns = "error_line";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        
        geometry_msgs::msg::Point p1, p2;
        p1.x = target.x();
        p1.y = target.y();
        p1.z = target.z();
        
        p2.x = end_effector_frame.p.x();
        p2.y = end_effector_frame.p.y();
        p2.z = end_effector_frame.p.z();
        
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
        
        line_marker.scale.x = 0.005;
        line_marker.color.r = 1.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 0.8;
        
        marker_array.markers.push_back(line_marker);
        
        // Workspace points (sample)
        auto workspace_marker = visualization_msgs::msg::Marker();
        workspace_marker.header.frame_id = "base_link";
        workspace_marker.header.stamp = this->get_clock()->now();
        workspace_marker.ns = "workspace";
        workspace_marker.id = 0;
        workspace_marker.type = visualization_msgs::msg::Marker::POINTS;
        workspace_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Sample every 10th point to avoid overwhelming RViz
        for (size_t i = 0; i < workspace_points_.size(); i += 10) {
            geometry_msgs::msg::Point p;
            p.x = workspace_points_[i].x();
            p.y = workspace_points_[i].y();
            p.z = workspace_points_[i].z();
            workspace_marker.points.push_back(p);
        }
        
        workspace_marker.scale.x = 0.002;
        workspace_marker.scale.y = 0.002;
        workspace_marker.color.r = 0.5;
        workspace_marker.color.g = 0.5;
        workspace_marker.color.b = 1.0;
        workspace_marker.color.a = 0.3;
        
        marker_array.markers.push_back(workspace_marker);
        
        marker_pub_->publish(marker_array);
    }
    
    // Member variables
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    KDL::Chain kdl_chain_;
    std::unique_ptr<ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<PositionOnlyIKSolver> pos_ik_solver_;

    std::vector<std::string> joint_names_;
    std::vector<Vector> workspace_points_;
    std::vector<Vector> test_targets_;

    int current_target_index_;
    int target_update_counter_;

    JntArray current_joint_positions_;
    JntArray target_joint_positions_;
    bool first_solution_received_;
    bool reached_target_;
    bool suppress_initial_teleport_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<IKVisualizerNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting IK Visualizer Node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}