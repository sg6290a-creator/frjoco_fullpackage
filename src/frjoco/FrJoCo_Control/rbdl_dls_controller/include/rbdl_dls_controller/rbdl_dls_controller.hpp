#ifndef RBDL_DLS_CONTROLLERS__JOINT_GROUP_DLS_CONTROLLER_HPP_
#define RBDL_DLS_CONTROLLERS__JOINT_GROUP_DLS_CONTROLLER_HPP_

#include <memory>
#include <string>

// ROS2의 Forward Kinematics Controller를 기반으로 쏴줄거임
#include "forward_command_controller/forward_command_controller.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rbdl_dls_controller/rbdl_dls_solver.hpp"

#include <mutex>

namespace rbdl_dls_controllers
{

/**
 * \brief DLS inverse kinematics controller driven by a Cartesian pose goal.
 *
 * Subscribes to ~/target_pose (std_msgs/Float64MultiArray[6] = [x,y,z,roll,pitch,yaw]).
 * Each update step: FK → pose error → twist → DLS → joint position commands.
 *
 * \param joints             List of joint names to control.
 * \param kp_pos             Position error gain       (default 1.0).
 * \param kp_rot             Rotation error gain       (default 1.0).
 * \param goal_tolerance     Position stop threshold [m]   (default 0.005).
 * \param rot_tolerance      Rotation stop threshold [rad] (default 0.01).
 * \param dq_max             Max joint velocity [rad/s]    (default 1.0).
 */
class JointGroupDLSController : public forward_command_controller::ForwardCommandController
{
public:
    JointGroupDLSController();

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    DLSSolver dls_solver_;
    Eigen::VectorXd q_current_;
    Eigen::VectorXd q_target_;    // 적분된 목표 관절 위치 (실제 이동과 무관하게 누적)

    // Target pose 어디서
    bool has_target_ = false;
    bool reset_q_target_ = true;
    Eigen::Vector3d    target_pos_;
    Eigen::Quaterniond target_rot_;

    double kp_pos_        = 1.0;
    double kp_rot_        = 1.0;
    double goal_tolerance_ = 0.005;   // m
    double rot_tolerance_  = 0.01;    // rad
    double dq_max_         = 1.0;     // rad/s
    double max_reach_      = 0.54;    // m (90% of theoretical 0.60)
    double min_reach_      = 0.10;    // m (base 근처 회피)
    double z_min_          = 0.0;     // m (바닥 관통 방지)
    double z_max_          = 1.0;     // m (천장 제한)

    Eigen::VectorXd q_min_;
    Eigen::VectorXd q_max_;

    mutable std::mutex target_mutex_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr        current_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr        error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr        joint_cmd_pub_;
};

}  // namespace rbdl_dls_controllers

#endif  // RBDL_DLS_CONTROLLERS__JOINT_GROUP_DLS_CONTROLLER_HPP_

