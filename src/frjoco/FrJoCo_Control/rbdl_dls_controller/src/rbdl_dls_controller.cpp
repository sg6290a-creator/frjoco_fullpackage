#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rbdl_dls_controller/rbdl_dls_controller.hpp"
#include "rclcpp/parameter.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


namespace rbdl_dls_controllers
{

JointGroupDLSController::JointGroupDLSController()
: forward_command_controller::ForwardCommandController()
{
    interface_name_ = hardware_interface::HW_IF_POSITION;
}


controller_interface::CallbackReturn JointGroupDLSController::on_init()
{
    auto ret = forward_command_controller::ForwardCommandController::on_init();
    if (ret != CallbackReturn::SUCCESS) {
        return ret;
    }

    try {
        auto_declare<double>("kp_pos",          0.005);
        auto_declare<double>("kp_rot",          0.005);
        auto_declare<double>("goal_tolerance",  0.005);
        auto_declare<double>("rot_tolerance",   0.01);
        auto_declare<double>("dq_max",          1.0);
        auto_declare<double>("max_reach",       0.54);
        auto_declare<double>("min_reach",       0.10);
        auto_declare<double>("z_min",           0.0);
        auto_declare<double>("z_max",           1.0);
        auto_declare<std::vector<double>>("q_min",
            {-6.283, -6.283, -3.142, -6.283, -6.283, -6.283});
        auto_declare<std::vector<double>>("q_max",
            { 6.283,  6.283,  3.142,  6.283,  6.283,  6.283});
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn JointGroupDLSController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    auto ret = forward_command_controller::ForwardCommandController::on_configure(previous_state);
    if (ret != CallbackReturn::SUCCESS) {
        return ret;
    }

    try {
        kp_pos_        = get_node()->get_parameter("kp_pos").as_double();
        kp_rot_        = get_node()->get_parameter("kp_rot").as_double();
        goal_tolerance_= get_node()->get_parameter("goal_tolerance").as_double();
        rot_tolerance_ = get_node()->get_parameter("rot_tolerance").as_double();
        dq_max_        = get_node()->get_parameter("dq_max").as_double();
        max_reach_     = get_node()->get_parameter("max_reach").as_double();
        min_reach_     = get_node()->get_parameter("min_reach").as_double();
        z_min_         = get_node()->get_parameter("z_min").as_double();
        z_max_         = get_node()->get_parameter("z_max").as_double();

        auto qmin_vec = get_node()->get_parameter("q_min").as_double_array();
        auto qmax_vec = get_node()->get_parameter("q_max").as_double_array();
        if (qmin_vec.size() != params_.joints.size() ||
            qmax_vec.size() != params_.joints.size()) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "q_min/q_max size (%zu/%zu) != joint count (%zu)",
                qmin_vec.size(), qmax_vec.size(), params_.joints.size());
            return CallbackReturn::ERROR;
        }
        q_min_ = Eigen::Map<Eigen::VectorXd>(qmin_vec.data(), qmin_vec.size());
        q_max_ = Eigen::Map<Eigen::VectorXd>(qmax_vec.data(), qmax_vec.size());
    }
    catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get parameters: %s", e.what());
        return CallbackReturn::ERROR;
    }

    dls_solver_.setDqMax(dq_max_);

    if (!dls_solver_.init()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize DLS solver");
        return CallbackReturn::ERROR;
    }

    q_current_.resize(static_cast<Eigen::Index>(params_.joints.size()));
    q_current_.setZero();
    q_target_.resize(static_cast<Eigen::Index>(params_.joints.size()));
    q_target_.setZero();

    pose_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/target_pose", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (msg->data.size() != 6) {
                RCLCPP_WARN(get_node()->get_logger(),
                    "target_pose expects [x,y,z,roll,pitch,yaw] (6 values), got %zu",
                    msg->data.size());
                return;
            }

            Eigen::Vector3d p(msg->data[0], msg->data[1], msg->data[2]);
            const double roll  = msg->data[3];
            const double pitch = msg->data[4];
            const double yaw   = msg->data[5];

            // 사전 workspace 체크
            double d = p.norm();
            if (d > max_reach_ || d < min_reach_) {
                RCLCPP_WARN(get_node()->get_logger(),
                    "Target REJECTED: distance from base = %.3f m (limit: %.3f ~ %.3f)",
                    d, min_reach_, max_reach_);
                return;
            }
            if (p.z() < z_min_ || p.z() > z_max_) {
                RCLCPP_WARN(get_node()->get_logger(),
                    "Target REJECTED: z = %.3f m (limit: %.3f ~ %.3f)",
                    p.z(), z_min_, z_max_);
                return;
            }

            // ZYX 오일러 → quaternion
            Eigen::Quaterniond q =
                Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());

            std::lock_guard<std::mutex> lock(target_mutex_);
            target_pos_ = p;
            target_rot_ = q.normalized();
            has_target_ = true;
            reset_q_target_ = true;  // 새 타겟 시 q_target_ → q_current_ 리셋
            RCLCPP_INFO(get_node()->get_logger(),
                "New target ACCEPTED: xyz=[%.3f, %.3f, %.3f] rpy=[%.3f, %.3f, %.3f] (dist=%.3f m)",
                p.x(), p.y(), p.z(), roll, pitch, yaw, d);
        });

    current_pose_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/current_pose", 10);
    error_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/error", 10);
    joint_cmd_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/joint_commands", 10);

    RCLCPP_INFO(get_node()->get_logger(),
        "DLS Controller configured — kp_pos: %.2f, kp_rot: %.2f",
        kp_pos_, kp_rot_);
    return CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration
JointGroupDLSController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : params_.joints) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
}


controller_interface::return_type JointGroupDLSController::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    // 1. 현재 관절 위치 읽기
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
        q_current_(static_cast<Eigen::Index>(i)) = state_interfaces_[i].get_value();
    }

    // 새 타겟 수신 시 q_target_ 을 현재 위치로 리셋
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (reset_q_target_) {
            q_target_ = q_current_;
            reset_q_target_ = false;
        }
    }

    // ~/commands 토픽으로 관절 직접 명령 (IK 우선순위 낮음, joint override)
    auto * joint_cmd = rt_command_ptr_.readFromRT();
    if (joint_cmd && *joint_cmd &&
        (*joint_cmd)->data.size() == command_interfaces_.size())
    {
        const auto & data = (*joint_cmd)->data;
        for (size_t i = 0; i < command_interfaces_.size(); ++i) {
            (void)command_interfaces_[i].set_value(data[i]);
        }
        q_target_ = Eigen::Map<const Eigen::VectorXd>(data.data(),
            static_cast<Eigen::Index>(data.size()));
        return controller_interface::return_type::OK;
    }

    // 2. FK로 현재 EE 포즈 계산 (타겟 유무와 무관하게 항상 수행)
    Eigen::Isometry3d current_pose = dls_solver_.eePose(q_current_);

    // 3. current_pose 항상 퍼블리시 (타겟 없어도 위치 확인 가능)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = time;
        pose_msg.header.frame_id = "base_link";
        pose_msg.pose.position.x = current_pose.translation().x();
        pose_msg.pose.position.y = current_pose.translation().y();
        pose_msg.pose.position.z = current_pose.translation().z();
        Eigen::Quaterniond q_cur(current_pose.linear());
        pose_msg.pose.orientation.x = q_cur.x();
        pose_msg.pose.orientation.y = q_cur.y();
        pose_msg.pose.orientation.z = q_cur.z();
        pose_msg.pose.orientation.w = q_cur.w();
        current_pose_pub_->publish(pose_msg);
    }

    if (!has_target_) {
        return controller_interface::return_type::OK;
    }

    // 4. 포즈 에러 계산 (world frame)
    Eigen::Vector3d pos_err;
    Eigen::Quaterniond target_rot_local;
    Eigen::Vector3d target_pos_local;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_pos_local = target_pos_;
        target_rot_local = target_rot_;
    }
    pos_err = target_pos_local - current_pose.translation();

    // 회전 에러: R_err = R_target * R_current^T → axis-angle
    Eigen::Matrix3d R_err = target_rot_local.toRotationMatrix() * current_pose.linear().transpose();
    Eigen::AngleAxisd aa(R_err);
    Eigen::Vector3d rot_err = aa.angle() * aa.axis();

    // 5. 에러 퍼블리시
    {
        std_msgs::msg::Float64MultiArray err_msg;
        err_msg.data = {pos_err.norm(), aa.angle()};
        error_pub_->publish(err_msg);
    }

    // 6. 목표 도달 확인 (위치 + 회전)
    if (pos_err.norm() < goal_tolerance_ && aa.angle() < rot_tolerance_) {
        return controller_interface::return_type::OK;
    }

    // 7. 에러 → twist
    Eigen::VectorXd v_t(6);
    v_t << kp_pos_ * pos_err, kp_rot_ * rot_err;

    // 8. DLS IK → 속도 스텝 계산 (q_current_ 기반)
    Eigen::VectorXd dq_step = dls_solver_.calculate(q_current_, v_t, period.seconds()) - q_current_;

    // 9. q_target_ 적분: 실제 팔 이동 여부와 무관하게 매 사이클 누적
    //    단, q_current_ 로부터 최대 0.5 rad 이상 앞서지 않도록 제한 (안전)
    static constexpr double kMaxLead = 0.5;
    q_target_ = (q_target_ - q_current_).cwiseMax(-kMaxLead).cwiseMin(kMaxLead) + q_current_;
    q_target_ += dq_step;

    // 10. joint limit clamp
    for (Eigen::Index i = 0; i < q_target_.size(); ++i) {
        if (q_target_(i) < q_min_(i) || q_target_(i) > q_max_(i)) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                "Joint %ld clamped: %.3f → [%.3f, %.3f]",
                i, q_target_(i), q_min_(i), q_max_(i));
        }
        q_target_(i) = std::clamp(q_target_(i), q_min_(i), q_max_(i));
    }

    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
        "[DBG] period=%.6f  dq_step=%.4f  q_cur[0]=%.4f  q_tgt[0]=%.4f  pos_err=%.4f",
        period.seconds(), dq_step.norm(),
        q_current_(0), q_target_(0), pos_err.norm());

    // 11. 커맨드 출력 (누적된 q_target_ 전송)
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
        (void)command_interfaces_[i].set_value(q_target_(static_cast<Eigen::Index>(i)));
    }

    // q_target_ 퍼블리시 (추종 확인용)
    {
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data.resize(q_target_.size());
        for (Eigen::Index i = 0; i < q_target_.size(); ++i)
            cmd_msg.data[i] = q_target_(i);
        joint_cmd_pub_->publish(cmd_msg);
    }

    return controller_interface::return_type::OK;
}

}  // namespace rbdl_dls_controllers


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    rbdl_dls_controllers::JointGroupDLSController,
    controller_interface::ControllerInterface)
