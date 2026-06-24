#include "frlab_manipulator_hardware/frlab_manipulator_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

namespace frlab_manipulator_hardware
{
namespace
{
double wrap_to_pi(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

hardware_interface::CallbackReturn FrlabManipulatorHardware::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    auto logger = rclcpp::get_logger(LOGGER);

    if (info_.joints.size() != static_cast<size_t>(manipulator_sdk::MANIPULATOR_DOF)) {
        RCLCPP_ERROR(
            logger, "Expected %d joints, got %zu.", manipulator_sdk::MANIPULATOR_DOF, info_.joints.size());
        return CallbackReturn::ERROR;
    }

    for (const auto & joint : info_.joints) {
        bool has_pos_state = false, has_pos_cmd = false;
        for (const auto & s : joint.state_interfaces) {
            if (s.name == hardware_interface::HW_IF_POSITION) has_pos_state = true;
        }
        for (const auto & c : joint.command_interfaces) {
            if (c.name == hardware_interface::HW_IF_POSITION) has_pos_cmd = true;
        }
        if (!has_pos_state || !has_pos_cmd) {
            RCLCPP_ERROR(
                logger,
                "Joint '%s' missing required position interface.", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
    }

    hw_positions_.fill(0.0);
    hw_velocities_.fill(0.0);
    hw_efforts_.fill(0.0);
    hw_position_commands_.fill(0.0);
    motor_position_offsets_.fill(0.0);
    ros_position_offsets_.fill(0.0);

    can_interface_ = "can0";
    read_deadline_ms_ = 3;
    read_poll_timeout_ms_ = 1;
    perf_log_every_n_cycles_ = 0;

    if (info_.hardware_parameters.count("can_interface")) {
        can_interface_ = info_.hardware_parameters.at("can_interface");
    }
    if (info_.hardware_parameters.count("default_velocity")) {
        default_velocity_rads_ = std::stod(info_.hardware_parameters.at("default_velocity"));
    }
    if (info_.hardware_parameters.count("read_deadline_ms")) {
        read_deadline_ms_ = std::stoi(info_.hardware_parameters.at("read_deadline_ms"));
    }
    if (info_.hardware_parameters.count("read_poll_timeout_ms")) {
        read_poll_timeout_ms_ = std::stoi(info_.hardware_parameters.at("read_poll_timeout_ms"));
    }
    if (info_.hardware_parameters.count("perf_log_every_n_cycles")) {
        perf_log_every_n_cycles_ = static_cast<std::size_t>(
            std::stoul(info_.hardware_parameters.at("perf_log_every_n_cycles")));
    }
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto & joint_name = info_.joints[i].name;
        const std::string named_key = "ros_position_offset_" + joint_name;
        const std::string index_key = "ros_position_offset_joint_" + std::to_string(i + 1);
        if (info_.hardware_parameters.count(named_key)) {
            ros_position_offsets_[i] = std::stod(info_.hardware_parameters.at(named_key));
        } else if (info_.hardware_parameters.count(index_key)) {
            ros_position_offsets_[i] = std::stod(info_.hardware_parameters.at(index_key));
        }
    }

    if (read_deadline_ms_ < 1) {
        read_deadline_ms_ = 1;
    }
    if (read_poll_timeout_ms_ < 0) {
        read_poll_timeout_ms_ = 0;
    }

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        if (std::abs(ros_position_offsets_[i]) > 1e-9) {
            RCLCPP_INFO(
                logger,
                "Joint '%s': applying ROS position offset %.3f rad",
                info_.joints[i].name.c_str(),
                ros_position_offsets_[i]);
        }
    }

    return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn FrlabManipulatorHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto logger = rclcpp::get_logger(LOGGER);
    RCLCPP_INFO(logger, "Connecting to arm on '%s'...", can_interface_.c_str());

    // CAN 인터페이스 존재 여부 사전 확인
    {
        int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd >= 0) {
            struct ifreq ifr{};
            std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
            if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
                RCLCPP_ERROR(
                    logger,
                    "'%s' not found. Run: sudo slcand -o -c -s8 /dev/ttyACM0 %s && sudo ip link set up %s",
                    can_interface_.c_str(), can_interface_.c_str(), can_interface_.c_str());
                ::close(fd);
                return CallbackReturn::ERROR;
            }
            ::close(fd);
        }
    }

    if (!arm_.init(can_interface_, read_deadline_ms_, read_poll_timeout_ms_)) {
        RCLCPP_ERROR(logger, "FrlabManipulator::init() failed.");
        return CallbackReturn::ERROR;
    }

    manipulator_sdk::ManipulatorState state;
    if (!(arm_.read(state) && state.valid)) {
        arm_.readCached(state);
    }

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const double calibrated_position = state.position[i] - ros_position_offsets_[i];
        const double wrapped_position = wrap_to_pi(calibrated_position);
        motor_position_offsets_[i] = calibrated_position - wrapped_position;
        hw_positions_[i] = wrapped_position;
        hw_velocities_[i] = state.velocity[i];
        hw_efforts_[i] = state.effort[i];
        hw_position_commands_[i] = wrapped_position;
        RCLCPP_INFO(
            logger,
            "Joint '%s': motor=%.3f rad, ros=%.3f rad, offset=%.3f rad",
            info_.joints[i].name.c_str(),
            state.position[i],
            hw_positions_[i],
            motor_position_offsets_[i]);
    }

    RCLCPP_INFO(logger, "Arm connected.");
    return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn FrlabManipulatorHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto logger = rclcpp::get_logger(LOGGER);
    manipulator_sdk::ManipulatorState state;
    arm_.readCached(state);
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        hw_positions_[i] = state.position[i] - motor_position_offsets_[i] - ros_position_offsets_[i];
        hw_velocities_[i] = state.velocity[i];
        hw_efforts_[i] = state.effort[i];
        hw_position_commands_[i] = hw_positions_[i];
    }

    RCLCPP_INFO(logger, "Arm activated.");
    return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn FrlabManipulatorHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "Arm deactivated.");
    return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn FrlabManipulatorHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    arm_.shutdown();
    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "Arm shutdown.");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
FrlabManipulatorHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(info_.joints.size() * 3);

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto & joint = info_.joints[i];
        for (const auto & iface : joint.state_interfaces) {
            if (iface.name == hardware_interface::HW_IF_POSITION) {
                interfaces.emplace_back(joint.name, iface.name, &hw_positions_[i]);
            } else if (iface.name == hardware_interface::HW_IF_VELOCITY) {
                interfaces.emplace_back(joint.name, iface.name, &hw_velocities_[i]);
            } else if (iface.name == hardware_interface::HW_IF_EFFORT) {
                interfaces.emplace_back(joint.name, iface.name, &hw_efforts_[i]);
            }
        }
    }

    return interfaces;
}

std::vector<hardware_interface::CommandInterface>
FrlabManipulatorHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(info_.joints.size());

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto & joint = info_.joints[i];
        for (const auto & iface : joint.command_interfaces) {
            if (iface.name == hardware_interface::HW_IF_POSITION) {
                interfaces.emplace_back(joint.name, iface.name, &hw_position_commands_[i]);
            }
        }
    }

    return interfaces;
}

hardware_interface::return_type FrlabManipulatorHardware::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // SyncDriver: write()에서 writeRead()로 통신 완료, 여기선 캐시만 반환.
    auto start = std::chrono::steady_clock::now();
    manipulator_sdk::ManipulatorState state;
    arm_.readCached(state);
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        hw_positions_[i] = state.position[i] - motor_position_offsets_[i] - ros_position_offsets_[i];
        hw_velocities_[i] = state.velocity[i];
        hw_efforts_[i] = state.effort[i];
    }
    last_read_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    perf_read_ms_accum_ += last_read_ms_;
    return hardware_interface::return_type::OK;
}


hardware_interface::return_type FrlabManipulatorHardware::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    auto logger = rclcpp::get_logger(LOGGER);
    auto start = std::chrono::steady_clock::now();
    std::array<double, manipulator_sdk::MANIPULATOR_DOF> pos_cmd, vel_cmd;
    vel_cmd.fill(default_velocity_rads_);

    pos_cmd.fill(0.0);
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        pos_cmd[i] = hw_position_commands_[i] + motor_position_offsets_[i] + ros_position_offsets_[i];
    }

    RCLCPP_DEBUG_THROTTLE(logger, throttle_clock_, 500,
        "[DBG] hw_cmd: %.3f %.3f %.3f %.3f %.3f %.3f",
        hw_position_commands_[0], hw_position_commands_[1], hw_position_commands_[2],
        hw_position_commands_[3], hw_position_commands_[4], hw_position_commands_[5]);

    if (!arm_.write(pos_cmd, vel_cmd)) {
        perf_read_failures_in_window_++;
        if (++consecutive_read_failures_ >= kMaxReadFailures) {
            RCLCPP_ERROR_THROTTLE(
                logger, throttle_clock_, 2000,
                "writeRead() failed %d times consecutively.", consecutive_read_failures_);
        } else {
            RCLCPP_WARN_THROTTLE(logger, throttle_clock_, 2000, "writeRead() failed, using cached state.");
        }
    } else {
        consecutive_read_failures_ = 0;
    }

    const double write_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    perf_write_ms_accum_ += write_ms;
    perf_cycle_ms_accum_ += last_read_ms_ + write_ms;
    perf_cycles_in_window_++;

    if (perf_log_every_n_cycles_ > 0 &&
        perf_cycles_in_window_ >= perf_log_every_n_cycles_) {
        const double cycles = static_cast<double>(perf_cycles_in_window_);
        RCLCPP_INFO(
            logger,
            "perf avg over %zu cycles: read=%.3fms write=%.3fms cycle=%.3fms read_failures=%zu",
            perf_cycles_in_window_,
            perf_read_ms_accum_ / cycles,
            perf_write_ms_accum_ / cycles,
            perf_cycle_ms_accum_ / cycles,
            perf_read_failures_in_window_);
        perf_cycles_in_window_ = 0;
        perf_read_failures_in_window_ = 0;
        perf_read_ms_accum_ = 0.0;
        perf_write_ms_accum_ = 0.0;
        perf_cycle_ms_accum_ = 0.0;
    }
    return hardware_interface::return_type::OK;
}

}  // namespace frlab_manipulator_hardware

PLUGINLIB_EXPORT_CLASS(
    frlab_manipulator_hardware::FrlabManipulatorHardware,
    hardware_interface::SystemInterface)
