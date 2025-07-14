#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
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

#include <trac_ik/trac_ik.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <memory>

using namespace KDL;
using namespace std::chrono_literals;

class ImprovedTrackIKNode : public rclcpp::Node {
private:
    // Core components
    std::string urdf_string_;
    std::unique_ptr<TRAC_IK::TRAC_IK> tracik_solver_;
    std::unique_ptr<ChainFkSolverPos_recursive> fk_solver_;
    
    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_point_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_point_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Transform broadcasting
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    // Robot model
    KDL::Chain kdl_chain_;
    std::vector<std::string> joint_names_;
    std::vector<std::pair<double, double>> joint_limits_;
    
    // State variables
    JntArray current_joint_positions_;
    JntArray target_joint_positions_;
    JntArray joint_velocities_;
    Vector current_target_position_;
    
    // Control parameters
    bool first_solution_received_;
    bool target_reached_;
    double position_tolerance_;
    double joint_velocity_limit_;
    double convergence_threshold_;
    
    // Workspace analysis
    std::vector<Vector> workspace_points_;
    std::vector<Vector> test_targets_;
    size_t current_target_index_;
    
    // Performance tracking
    int successful_solutions_;
    int failed_solutions_;
    double average_solve_time_;
    
public:
    ImprovedTrackIKNode() : Node("improved_trackik_node"), 
                           tf_broadcaster_(this),
                           static_tf_broadcaster_(this),
                           first_solution_received_(false),
                           target_reached_(true),
                           position_tolerance_(0.01),    
                           joint_velocity_limit_(1.0),   
                           convergence_threshold_(0.001),
                           current_target_index_(0),
                           successful_solutions_(0),
                           failed_solutions_(0),
                           average_solve_time_(0.0) {

        // Publishers
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_markers", 10);
        target_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("target_point", 10);
        
        // Subscribers - Added point subscriber for position-only commands
        target_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "target_point_cmd", 10, 
            std::bind(&ImprovedTrackIKNode::targetPointCallback, this, std::placeholders::_1));
            
        // Keep pose subscriber for backward compatibility
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose_cmd", 10, 
            std::bind(&ImprovedTrackIKNode::targetPoseCallback, this, std::placeholders::_1));

        // Initialize robot model
        if (!loadRobotModel()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model");
            return;
        }

        // Initialize solvers
        if (!initializeSolvers()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize solvers");
            return;
        }

        // Initialize test targets
        initializeTestTargets();
        
        // Generate workspace samples
        generateWorkspaceSamples();
        
        // Test basic functionality
        testSolverCapabilities();

        // Start main timer
        timer_ = this->create_wall_timer(100ms, std::bind(&ImprovedTrackIKNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Improved Track-IK Node initialized successfully (Position-only mode)");
    }

private:
    bool loadRobotModel() {
        // Load URDF file
        std::string urdf_path = "/home/vansh/intern-ardee/src/fcl-ros2/PXA-100_description/urdf/px.urdf";
        
        std::ifstream urdf_file(urdf_path);
        if (!urdf_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file: %s", urdf_path.c_str());
            return false;
        }
        
        std::stringstream buffer;
        buffer << urdf_file.rdbuf();
        urdf_string_ = buffer.str();
        urdf_file.close();
        
        // Parse URDF
        urdf::Model robot_model;
        if (!robot_model.initString(urdf_string_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
            return false;
        }
        
        // Build KDL tree
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert URDF to KDL Tree");
            return false;
        }
        
        // Extract chain
        std::string base_link = "base_link";
        std::string tip_link = "end";
        
        if (!kdl_tree.getChain(base_link, tip_link, kdl_chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract chain from %s to %s", 
                        base_link.c_str(), tip_link.c_str());
            return false;
        }
        
        // Extract joint names
        joint_names_.clear();
        for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); ++i) {
            KDL::Segment segment = kdl_chain_.getSegment(i);
            if (segment.getJoint().getType() != KDL::Joint::None) {
                joint_names_.push_back(segment.getJoint().getName());
            }
        }
        
        // Fallback joint names if extraction fails
        if (joint_names_.empty()) {
            joint_names_ = {"Revolute 18", "Revolute 19", "Revolute 20", "Revolute 15"};
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded robot model with %d joints", 
                   kdl_chain_.getNrOfJoints());
        
        return true;
    }
    
    bool initializeSolvers() {
        // Initialize Forward Kinematics solver
        fk_solver_ = std::make_unique<ChainFkSolverPos_recursive>(kdl_chain_);
        
        // Initialize Track-IK solver with more relaxed settings
        std::string base_link = "base_link";
        std::string tip_link = "end";
        
        double timeout = 0.01;   // Increased timeout to 10ms
        double eps = 5e-3;       // Relaxed tolerance to 5mm
        
        RCLCPP_INFO(this->get_logger(), "Initializing Track-IK with timeout=%.3f, eps=%.6f", timeout, eps);
        
        tracik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
            base_link, tip_link, urdf_string_, timeout, eps, TRAC_IK::Speed);
        
        // Verify Track-IK initialization
        KDL::Chain trac_ik_chain;
        if (!tracik_solver_->getKDLChain(trac_ik_chain)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from Track-IK");
            return false;
        }
    
        
        // Initialize joint arrays
        unsigned int nj = kdl_chain_.getNrOfJoints();
        current_joint_positions_ = JntArray(nj);
        target_joint_positions_ = JntArray(nj);
        joint_velocities_ = JntArray(nj);
        
        // Initialize to safe mid-range positions
        for (unsigned int i = 0; i < nj; ++i) {
            if (i < joint_limits_.size()) {
                current_joint_positions_(i) = (joint_limits_[i].first + joint_limits_[i].second) / 2.0;
            } else {
                current_joint_positions_(i) = 0.0;
            }
            joint_velocities_(i) = 0.0;
        }
        target_joint_positions_ = current_joint_positions_;
        
        // Set initial target position and verify it
        Frame initial_frame;
        if (fk_solver_->JntToCart(current_joint_positions_, initial_frame) >= 0) {
            current_target_position_ = initial_frame.p;
            RCLCPP_INFO(this->get_logger(), "Initial EE position: [%.3f, %.3f, %.3f]", 
                       current_target_position_.x(), current_target_position_.y(), current_target_position_.z());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute initial forward kinematics");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "All solvers initialized successfully");
        return true;
    }
    
    void initializeTestTargets() {
        // Get current end-effector position
        Frame current_frame;
        if (fk_solver_->JntToCart(current_joint_positions_, current_frame) >= 0) {
            Vector base_pos = current_frame.p;
            
            RCLCPP_INFO(this->get_logger(), "Current EE position: [%.3f, %.3f, %.3f]", 
                       base_pos.x(), base_pos.y(), base_pos.z());

                       
            
            // Create conservative test targets around current position
            test_targets_ = {
                base_pos,                                    // Current position
                base_pos + Vector(0.0, 0.1, 0.0),         // Small +Y movement
                base_pos + Vector(0.0, -0.05, 0.0),        // Small -Y movement
                base_pos + Vector(0.0, 0.0, 0.05),         // Small +Z movement
                base_pos + Vector(0.0, 0.0, -0.1),
                      // Small -Z movement
               
            };
            
            // test_targets_ = {
            //         Vector(0.2, 0.0, 0.3),
            //         Vector(0.1, 0.1, 0.2),
            //         Vector(0.15, -0.05, 0.25),
            //         Vector(0.2, 0.05, 0.35),
            //         Vector(0.1, -0.1, 0.3)
            //     };

            
            // Test all targets and keep only reachable ones
            std::vector<Vector> reachable_targets;
            for (size_t i = 0; i < test_targets_.size(); ++i) {
                const auto& target = test_targets_[i];
                RCLCPP_INFO(this->get_logger(), "Testing target %zu: [%.3f, %.3f, %.3f]", 
                           i, target.x(), target.y(), target.z());
                
                if (isTargetReachable(target)) {
                    // Do a quick IK test
                    JntArray test_result(kdl_chain_.getNrOfJoints());
                    
                    if (solveIKPositionOnly(target, current_joint_positions_, test_result)) {
                        reachable_targets.push_back(target);
                        RCLCPP_INFO(this->get_logger(), "✓ Target %zu is reachable", i);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "✗ Target %zu failed IK test", i);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "✗ Target %zu failed reachability test", i);
                }
            }
            
            test_targets_ = reachable_targets;
            
            RCLCPP_INFO(this->get_logger(), "Initialized %zu reachable test targets", test_targets_.size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current EE position for test target initialization");
        }
    }
    
    bool isTargetReachable(const Vector& target) {
        // More comprehensive reachability check
        double distance = target.Norm();
        // double dist = target_frame.p.Norm();  // Euclidean distance
        RCLCPP_INFO(get_logger(), "Target distance from base: %.3f", distance);

        // Check distance bounds
        if (distance < 0.01) {
            RCLCPP_WARN(this->get_logger(), "Target too close to origin: %.3f m", distance);
            return false;
        }
        
        if (distance > 0.8) {  // Conservative upper bound
            RCLCPP_WARN(this->get_logger(), "Target too far from origin: %.3f m", distance);
            return false;
        }
        
        // Check if target is not too close to base
        if (std::abs(target.z()) < 0.001) {
            RCLCPP_WARN(this->get_logger(), "Target too close to base plane: z=%.3f", target.z());
            return false;
        }
        
        return true;
    }
    
    void generateWorkspaceSamples() {
        RCLCPP_INFO(this->get_logger(), "Generating workspace samples...");
        
        workspace_points_.clear();
        srand(42);
        int num_samples = 2000;
        
        for (int i = 0; i < num_samples; ++i) {
            JntArray q(kdl_chain_.getNrOfJoints());
            
            // Generate random joint angles within limits
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
    
    void testSolverCapabilities() {
        RCLCPP_INFO(this->get_logger(), "Testing solver capabilities...");
        
        // Test current position
        Frame current_frame;
        if (fk_solver_->JntToCart(current_joint_positions_, current_frame) >= 0) {
            JntArray test_result(kdl_chain_.getNrOfJoints());
            
            auto start_time = std::chrono::high_resolution_clock::now();
            bool success = solveIKPositionOnly(current_frame.p, current_joint_positions_, test_result);
            auto end_time = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            
            if (success) {
                RCLCPP_INFO(this->get_logger(), "✓ Self-test passed (%.3f ms)", duration.count() / 1000.0);
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ Self-test failed");
            }
        }
    }
    
    // New method for position-only IK solving
    bool solveIKPositionOnly(const Vector& target_pos, const JntArray& q_init, JntArray& q_result) {
        // First check if target is within reasonable bounds
        double distance_from_origin = target_pos.Norm();
        RCLCPP_INFO(this->get_logger(), "Target position: [%.3f, %.3f, %.3f], distance: %.3f", 
                   target_pos.x(), target_pos.y(), target_pos.z(), distance_from_origin);
        
        // Check basic reachability
        if (distance_from_origin < 0.01) {
            RCLCPP_ERROR(this->get_logger(), "Target too close to origin: %.3f m", distance_from_origin);
            return false;
        }
        
        if (distance_from_origin > 1.0) {
            RCLCPP_ERROR(this->get_logger(), "Target too far from origin: %.3f m", distance_from_origin);
            return false;
        }
        
        // Create target frame with identity orientation (no orientation constraint)
        Frame target_frame;
        target_frame.p = target_pos;
        target_frame.M = Rotation::Identity();  // Don't constrain orientation
        
        q_result = JntArray(kdl_chain_.getNrOfJoints());
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Try Track-IK with relaxed settings
        int result = tracik_solver_->CartToJnt(q_init, target_frame, q_result);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        // Update performance statistics
        if (result >= 0) {
            successful_solutions_++;
            average_solve_time_ = (average_solve_time_ * (successful_solutions_ - 1) + 
                                  duration.count() / 1000.0) / successful_solutions_;
            
            // Verify solution (only check position, ignore orientation)
            Frame verify_frame;
            if (fk_solver_->JntToCart(q_result, verify_frame) >= 0) {
                Vector pos_error = target_pos - verify_frame.p;
                double error_norm = pos_error.Norm();
                
                RCLCPP_INFO(this->get_logger(), "Solution found! Position error: %.6f m", error_norm);
                
                if (error_norm > position_tolerance_) {
                    RCLCPP_WARN(this->get_logger(), "Large position error: %.6f m", error_norm);
                }
            }
            
            return true;
        } else {
            failed_solutions_++;
            
            // More detailed error reporting
            RCLCPP_ERROR(this->get_logger(), "Track-IK failed for target [%.3f, %.3f, %.3f]", 
                        target_pos.x(), target_pos.y(), target_pos.z());
            
            switch (result) {
                case -1:
                    RCLCPP_ERROR(this->get_logger(), "Track-IK failed: Timeout occurred (increase timeout or reduce precision)");
                    break;
                case -2:
                    RCLCPP_ERROR(this->get_logger(), "Track-IK failed: No solution within tolerance (target may be unreachable)");
                    break;
                case -3:
                    RCLCPP_ERROR(this->get_logger(), "Track-IK failed: Invalid inputs (check URDF or joint limits)");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Track-IK failed: Unknown error code %d", result);
                    break;
            }
            
            // Try to find current end-effector position for debugging
            Frame current_frame;
            if (fk_solver_->JntToCart(q_init, current_frame) >= 0) {
                RCLCPP_INFO(this->get_logger(), "Current EE position: [%.3f, %.3f, %.3f]", 
                           current_frame.p.x(), current_frame.p.y(), current_frame.p.z());
            }
            
            return false;
        }
    }
    
    // New callback for point-only commands
    void targetPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        // Update target from external point command
        current_target_position_ = Vector(msg->point.x, msg->point.y, msg->point.z);
        
        target_reached_ = false;
        RCLCPP_INFO(this->get_logger(), "New target point received: [%.3f, %.3f, %.3f]", 
                   current_target_position_.x(), current_target_position_.y(), current_target_position_.z());
    }
    
    // Modified to only use position from pose messages
    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Update target from external command (ignore orientation)
        current_target_position_ = Vector(msg->pose.position.x, 
                                         msg->pose.position.y, 
                                         msg->pose.position.z);
        
        target_reached_ = false;
        RCLCPP_INFO(this->get_logger(), "New target position received (orientation ignored): [%.3f, %.3f, %.3f]", 
                   current_target_position_.x(), current_target_position_.y(), current_target_position_.z());
    }
    
    void timerCallback() {
        // Smooth joint interpolation
        bool joints_moving = false;
        for (unsigned int i = 0; i < current_joint_positions_.rows(); ++i) {
            double delta = target_joint_positions_(i) - current_joint_positions_(i);
            
            if (std::abs(delta) > convergence_threshold_) {
                double max_step = joint_velocity_limit_ * 0.05; // 50ms timestep
                double step = std::min(std::abs(delta), max_step);
                
                current_joint_positions_(i) += (delta > 0 ? 1 : -1) * step;
                joint_velocities_(i) = (delta > 0 ? 0.01 : -0.01) * step / 0.05;
                joints_moving = true;
            } 
            else if(delta<=0.0001){
                current_joint_positions_(i) = target_joint_positions_(i);
                joint_velocities_(i) = 0.0;
            }
            else {
                double max_step = joint_velocity_limit_ * 0.05; // 50ms timestep
                double step = std::min(std::abs(delta), max_step);
                
                current_joint_positions_(i) += (delta > 0 ? 0.01 : -0.01) * step;
                joint_velocities_(i) = (delta > 0 ? 0.001 : -0.001) * step / 0.05;
                joints_moving = true;
            }
        }
        
        // Publish joint states
        publishJointStates();
        
        // Check if target is reached
        if (!joints_moving && !target_reached_) {
            target_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "Target reached!");
        }
        
        // Auto-cycle through test targets
        if (target_reached_ && !test_targets_.empty()) {
            static auto last_target_switch = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_target_switch).count() > 3) {
                // Switch to next target
                current_target_index_ = (current_target_index_ + 1) % test_targets_.size();
                current_target_position_ = test_targets_[current_target_index_];
                
                // Solve IK for new target (position only)
                JntArray q_result(kdl_chain_.getNrOfJoints());
                if (solveIKPositionOnly(current_target_position_, current_joint_positions_, q_result)) {
                    target_joint_positions_ = q_result;
                    target_reached_ = false;
                    RCLCPP_INFO(this->get_logger(), "Switching to target %zu", current_target_index_);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to solve IK for target %zu", current_target_index_);
                }
                
                last_target_switch = now;
            }
        }
        
        // Publish visualization
        publishVisualization();
        
        // Print performance statistics occasionally
        static int counter = 0;
        if (++counter % 200 == 0) { // Every 10 seconds
            RCLCPP_INFO(this->get_logger(), 
                       "Performance: Success=%d, Failed=%d, Avg solve time=%.3f ms",
                       successful_solutions_, failed_solutions_, average_solve_time_);
        }
    }
    
    void publishJointStates() {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.header.frame_id = "base_link";
        
        joint_state_msg.name = joint_names_;
        joint_state_msg.position.resize(current_joint_positions_.rows());
        joint_state_msg.velocity.resize(current_joint_positions_.rows());
        joint_state_msg.effort.resize(current_joint_positions_.rows());
        
        for (unsigned int i = 0; i < current_joint_positions_.rows(); ++i) {
            joint_state_msg.position[i] = current_joint_positions_(i);
            joint_state_msg.velocity[i] = joint_velocities_(i);
            joint_state_msg.effort[i] = 0.0;
        }
        
        joint_state_pub_->publish(joint_state_msg);
    }
    
    void publishVisualization() {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Current end-effector position
        Frame current_frame;
        if (fk_solver_->JntToCart(current_joint_positions_, current_frame) >= 0) {
            // End-effector marker
            auto ee_marker = visualization_msgs::msg::Marker();
            ee_marker.header.frame_id = "base_link";
            ee_marker.header.stamp = this->get_clock()->now();
            ee_marker.ns = "end_effector";
            ee_marker.id = 0;
            ee_marker.type = visualization_msgs::msg::Marker::SPHERE;
            ee_marker.action = visualization_msgs::msg::Marker::ADD;
            
            ee_marker.pose.position.x = current_frame.p.x();
            ee_marker.pose.position.y = current_frame.p.y();
            ee_marker.pose.position.z = current_frame.p.z();
            ee_marker.pose.orientation.w = 1.0;
            
            ee_marker.scale.x = 0.02;
            ee_marker.scale.y = 0.02;
            ee_marker.scale.z = 0.02;
            
            ee_marker.color.r = 0.0;
            ee_marker.color.g = 1.0;
            ee_marker.color.b = 0.0;
            ee_marker.color.a = 1.0;
            
            marker_array.markers.push_back(ee_marker);
        }
        
        // Target marker
        auto target_marker = visualization_msgs::msg::Marker();
        target_marker.header.frame_id = "base_link";
        target_marker.header.stamp = this->get_clock()->now();
        target_marker.ns = "target";
        target_marker.id = 0;
        target_marker.type = visualization_msgs::msg::Marker::SPHERE;
        target_marker.action = visualization_msgs::msg::Marker::ADD;
        
        target_marker.pose.position.x = current_target_position_.x();
        target_marker.pose.position.y = current_target_position_.y();
        target_marker.pose.position.z = current_target_position_.z();
        target_marker.pose.orientation.w = 1.0;
        
        target_marker.scale.x = 0.025;
        target_marker.scale.y = 0.025;
        target_marker.scale.z = 0.025;
        
        target_marker.color.r = 1.0;
        target_marker.color.g = 0.0;
        target_marker.color.b = 0.0;
        target_marker.color.a = 0.8;
        
        marker_array.markers.push_back(target_marker);
        
        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ImprovedTrackIKNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Improved Track-IK Node (Position-only mode)");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}