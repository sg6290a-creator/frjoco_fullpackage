#include "qp_controller/qp_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qp_controllers
{

QPController::QPController() = default;

controller_interface::InterfaceConfiguration
QPController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& j : joint_names_)
        cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    return cfg;
}

controller_interface::InterfaceConfiguration
QPController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& j : joint_names_)
        cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    return cfg;
}

controller_interface::CallbackReturn QPController::on_init()
{
    try {
        auto_declare<std::vector<std::string>>("joints", {});
        auto_declare<double>("kp_pos",          0.5);
        auto_declare<double>("kp_rot",          0.5);
        auto_declare<double>("goal_tolerance",  0.01);
        auto_declare<double>("rot_tolerance",   0.01);
        auto_declare<double>("dq_max",          0.5);
        auto_declare<double>("alpha",           1e-4);
        auto_declare<double>("max_reach",       0.54);
        auto_declare<double>("min_reach",       0.10);
        auto_declare<double>("z_min",           0.0);
        auto_declare<double>("z_max",           1.0);
        auto_declare<std::vector<double>>("q_min",
            {-6.283, -6.283, -3.142, -6.283, -6.283, -6.283});
        auto_declare<std::vector<double>>("q_max",
            { 6.283,  6.283,  3.142,  6.283,  6.283,  6.283});
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QPController::on_configure(
    const rclcpp_lifecycle::State&)
{
    joint_names_    = get_node()->get_parameter("joints").as_string_array();
    kp_pos_         = get_node()->get_parameter("kp_pos").as_double();
    kp_rot_         = get_node()->get_parameter("kp_rot").as_double();
    goal_tolerance_ = get_node()->get_parameter("goal_tolerance").as_double();
    rot_tolerance_  = get_node()->get_parameter("rot_tolerance").as_double();
    dq_max_         = get_node()->get_parameter("dq_max").as_double();
    alpha_          = get_node()->get_parameter("alpha").as_double();
    max_reach_      = get_node()->get_parameter("max_reach").as_double();
    min_reach_      = get_node()->get_parameter("min_reach").as_double();
    z_min_          = get_node()->get_parameter("z_min").as_double();
    z_max_          = get_node()->get_parameter("z_max").as_double();

    auto qmin_v = get_node()->get_parameter("q_min").as_double_array();
    auto qmax_v = get_node()->get_parameter("q_max").as_double_array();
    if (qmin_v.size() != joint_names_.size() || qmax_v.size() != joint_names_.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "q_min/q_max size mismatch");
        return CallbackReturn::ERROR;
    }
    q_min_ = Eigen::Map<Eigen::VectorXd>(qmin_v.data(), (int)qmin_v.size());
    q_max_ = Eigen::Map<Eigen::VectorXd>(qmax_v.data(), (int)qmax_v.size());

    // Init QP solver
    QPSolver::Limits lim;
    lim.q_min  = q_min_;
    lim.q_max  = q_max_;
    lim.dq_max = dq_max_;
    lim.alpha  = alpha_;
    if (!qp_solver_.init(lim)) {
        RCLCPP_ERROR(get_node()->get_logger(), "QPSolver init failed");
        return CallbackReturn::ERROR;
    }

    const int n = (int)joint_names_.size();
    q_current_.resize(n); q_current_.setZero();
    q_target_.resize(n);  q_target_.setZero();

    // Target pose subscription
    pose_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/target_pose", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (msg->data.size() != 6) {
                RCLCPP_WARN(get_node()->get_logger(), "target_pose expects 6 values");
                return;
            }
            Eigen::Vector3d p(msg->data[0], msg->data[1], msg->data[2]);
            double d = p.norm();
            if (d > max_reach_ || d < min_reach_) {
                RCLCPP_WARN(get_node()->get_logger(),
                    "Target REJECTED: reach=%.3f (%.3f~%.3f)", d, min_reach_, max_reach_);
                return;
            }
            if (p.z() < z_min_ || p.z() > z_max_) {
                RCLCPP_WARN(get_node()->get_logger(),
                    "Target REJECTED: z=%.3f (%.3f~%.3f)", p.z(), z_min_, z_max_);
                return;
            }
            Eigen::Quaterniond q =
                Eigen::AngleAxisd(msg->data[5], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(msg->data[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(msg->data[3], Eigen::Vector3d::UnitX());
            std::lock_guard<std::mutex> lk(target_mutex_);
            target_pos_     = p;
            target_rot_     = q.normalized();
            has_target_     = true;
            reset_q_target_ = true;
            RCLCPP_INFO(get_node()->get_logger(),
                "New target ACCEPTED: xyz=[%.3f,%.3f,%.3f] rpy=[%.3f,%.3f,%.3f] (dist=%.3fm)",
                p.x(), p.y(), p.z(), msg->data[3], msg->data[4], msg->data[5], d);
        });

    current_pose_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/current_pose", 10);
    error_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/error", 10);
    joint_cmd_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/joint_commands", 10);

    RCLCPP_INFO(get_node()->get_logger(),
        "QP Controller configured — kp_pos:%.2f kp_rot:%.2f dq_max:%.2f alpha:%.2e",
        kp_pos_, kp_rot_, dq_max_, alpha_);
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QPController::on_activate(const rclcpp_lifecycle::State&)
{
    for (size_t i = 0; i < state_interfaces_.size(); ++i)
        q_current_(i) = state_interfaces_[i].get_value();
    q_target_ = q_current_;
    has_target_ = false;
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn QPController::on_deactivate(const rclcpp_lifecycle::State&)
{
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type QPController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
    const double dt = period.seconds();
    if (dt <= 0.0) return controller_interface::return_type::OK;

    // 1. 현재 관절 위치 읽기
    for (size_t i = 0; i < state_interfaces_.size(); ++i)
        q_current_(i) = state_interfaces_[i].get_value();

    // 새 타겟 시 q_target_ → q_current_ 리셋
    {
        std::lock_guard<std::mutex> lk(target_mutex_);
        if (reset_q_target_) {
            q_target_ = q_current_;
            reset_q_target_ = false;
        }
    }

    // 2. FK
    Eigen::Isometry3d current_pose = qp_solver_.eePose(q_current_);

    // 3. current_pose publish
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp    = time;
        msg.header.frame_id = "base_link";
        msg.pose.position.x = current_pose.translation().x();
        msg.pose.position.y = current_pose.translation().y();
        msg.pose.position.z = current_pose.translation().z();
        Eigen::Quaterniond qr(current_pose.linear());
        msg.pose.orientation.x = qr.x();
        msg.pose.orientation.y = qr.y();
        msg.pose.orientation.z = qr.z();
        msg.pose.orientation.w = qr.w();
        current_pose_pub_->publish(msg);
    }

    if (!has_target_) return controller_interface::return_type::OK;

    // 4. 포즈 에러
    Eigen::Vector3d    tgt_pos;
    Eigen::Quaterniond tgt_rot;
    { std::lock_guard<std::mutex> lk(target_mutex_); tgt_pos = target_pos_; tgt_rot = target_rot_; }

    Eigen::Vector3d pos_err = tgt_pos - current_pose.translation();
    Eigen::Matrix3d R_err   = tgt_rot.toRotationMatrix() * current_pose.linear().transpose();
    Eigen::AngleAxisd aa(R_err);
    Eigen::Vector3d rot_err = aa.angle() * aa.axis();

    // 5. 에러 publish
    {
        std_msgs::msg::Float64MultiArray emsg;
        emsg.data = {pos_err.norm(), aa.angle()};
        error_pub_->publish(emsg);
    }

    // 6. 수렴 확인
    if (pos_err.norm() < goal_tolerance_ && aa.angle() < rot_tolerance_)
        return controller_interface::return_type::OK;

    // 7. desired twist
    Eigen::VectorXd v_des(6);
    v_des << kp_pos_ * pos_err, kp_rot_ * rot_err;

    // 8. Jacobian
    Eigen::MatrixXd J;
    qp_solver_.calcJacobian(q_current_, J);

    // 9. QP solve → dq (rad/s)
    Eigen::VectorXd dq = qp_solver_.solve(q_current_, J, v_des, dt);

    // 10. q_target 적분 (q_current 로부터 최대 0.5 rad 앞서지 않도록)
    static constexpr double kMaxLead = 0.5;
    q_target_ = (q_target_ - q_current_).cwiseMax(-kMaxLead).cwiseMin(kMaxLead) + q_current_;
    q_target_ += dq * dt;

    // 11. joint limit clamp (QP가 이미 보장하지만 안전망)
    for (int i = 0; i < q_target_.size(); ++i) {
        if (q_target_(i) < q_min_(i) || q_target_(i) > q_max_(i)) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                "Joint %d clamped: %.3f → [%.3f,%.3f]", i, q_target_(i), q_min_(i), q_max_(i));
        }
        q_target_(i) = std::clamp(q_target_(i), q_min_(i), q_max_(i));
    }

    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
        "[QP] pos_err=%.4f rot_err=%.4f dq_norm=%.4f q0=%.3f tgt0=%.3f",
        pos_err.norm(), aa.angle(), dq.norm(), q_current_(0), q_target_(0));

    // 12. 커맨드 출력
    for (size_t i = 0; i < command_interfaces_.size(); ++i)
        (void)command_interfaces_[i].set_value(q_target_(i));

    // q_target publish (추종 확인용)
    {
        std_msgs::msg::Float64MultiArray cmsg;
        cmsg.data.assign(q_target_.data(), q_target_.data() + q_target_.size());
        joint_cmd_pub_->publish(cmsg);
    }

    return controller_interface::return_type::OK;
}

}  // namespace qp_controllers

PLUGINLIB_EXPORT_CLASS(
    qp_controllers::QPController,
    controller_interface::ControllerInterface)
