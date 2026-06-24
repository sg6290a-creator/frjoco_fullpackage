#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "qp_controller/qp_solver.hpp"

namespace qp_controllers
{

class QPController : public controller_interface::ControllerInterface
{
public:
    QPController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration()   const override;

    controller_interface::CallbackReturn on_init()      override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&)  override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // Parameters
    std::vector<std::string> joint_names_;
    double kp_pos_         = 0.5;
    double kp_rot_         = 0.5;
    double goal_tolerance_ = 0.01;
    double rot_tolerance_  = 0.01;
    double dq_max_         = 0.5;
    double alpha_          = 1e-4;
    double max_reach_      = 0.54;
    double min_reach_      = 0.10;
    double z_min_          = 0.0;
    double z_max_          = 1.0;
    Eigen::VectorXd q_min_;
    Eigen::VectorXd q_max_;

    // State
    Eigen::VectorXd q_current_;
    Eigen::VectorXd q_target_;
    Eigen::Vector3d    target_pos_;
    Eigen::Quaterniond target_rot_;
    bool has_target_      = false;
    bool reset_q_target_  = false;
    mutable std::mutex target_mutex_;

    QPSolver qp_solver_;

    // ROS
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     current_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    joint_cmd_pub_;
};

}  // namespace qp_controllers
