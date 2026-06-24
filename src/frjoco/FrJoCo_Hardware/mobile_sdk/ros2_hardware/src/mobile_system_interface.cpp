/**
 * ============================================================================
 * MobileSystemInterface — Implementation
 * ============================================================================
 *
 * ros2_control SystemInterface ↔ MDDriver(pure C++) 브리지.
 * 모든 시리얼 통신/프로토콜/엔코더 로직은 MDDriver 에 위임.
 * 이 파일은 HardwareInfo 파싱 + lifecycle → MDDriver API 매핑만 담당.
 *
 * ============================================================================
 */

#include "mobile_sdk_ros2/mobile_system_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mobile_sdk_ros2
{

// ============================================================================
// Lifecycle
// ============================================================================

hardware_interface::CallbackReturn MobileSystemInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER),
        "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER),
        "║      Mobile SDK System Interface Initializing                ║");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER),
        "╚══════════════════════════════════════════════════════════════╝");

    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // ---- Parse HardwareInfo → MDDriverConfig ----
    mobile_sdk::MDDriverConfig cfg;

    auto getParam = [&](const std::string& key, const std::string& def) -> std::string {
        auto it = info_.hardware_parameters.find(key);
        return (it != info_.hardware_parameters.end()) ? it->second : def;
    };

    cfg.port_front      = getParam("port_front", "/dev/ttyUSB0");
    cfg.port_rear       = getParam("port_rear",  "/dev/ttyUSB1");
    cfg.baudrate        = std::stoi(getParam("baudrate",        "19200"));
    cfg.front_motor_id  = std::stoi(getParam("front_driver_id", "1"));
    cfg.rear_motor_id   = std::stoi(getParam("rear_driver_id",  "2"));
    cfg.id_mdui         = std::stoi(getParam("id_mdui",         "184"));
    cfg.id_mdt          = std::stoi(getParam("id_mdt",          "183"));
    cfg.wheel_radius    = std::stod(getParam("wheel_radius",    "0.098"));
    cfg.wheel_separation = std::stod(getParam("wheel_separation", "0.32"));
    cfg.gear_ratio      = std::stoi(getParam("gear_ratio",      "15"));
    cfg.poles            = std::stoi(getParam("poles",            "10"));

    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[PARAM] port_front = %s", cfg.port_front.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[PARAM] port_rear  = %s", cfg.port_rear.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[PARAM] baudrate   = %d", cfg.baudrate);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[PARAM] gear_ratio = %d, poles = %d",
                cfg.gear_ratio, cfg.poles);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[PARAM] wheel_radius = %.4f m", cfg.wheel_radius);

    // Joint validation (must be exactly 4)
    if (info_.joints.size() != mobile_sdk::NUM_WHEELS)
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER),
                     "[ERROR] Expected 4 joints but got %zu!", info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        joint_names_.push_back(info_.joints[i].name);
        RCLCPP_INFO(rclcpp::get_logger(LOGGER), "  [%zu] Joint: %s",
                     i, info_.joints[i].name.c_str());
    }

    // Create native driver
    driver_ = std::make_unique<mobile_sdk::MDDriver>(cfg);

    // Zero buffers
    hw_commands_.fill(0.0);
    hw_positions_.fill(0.0);
    hw_velocities_.fill(0.0);

    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[INIT] ✓ Initialization Complete!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileSystemInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[CONFIG] Opening serial ports via MDDriver...");

    if (!driver_->open())
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "[CONFIG] ✗ Failed to open serial ports!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[CONFIG] ✓ Serial ports opened.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileSystemInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[ACTIVATE] Activating 4WD motors...");

    if (!driver_->isOpen())
    {
        RCLCPP_ERROR(rclcpp::get_logger(LOGGER), "[ACTIVATE] Serial ports not open!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Reset all states
    hw_commands_.fill(0.0);
    hw_positions_.fill(0.0);
    hw_velocities_.fill(0.0);
    driver_->resetState();

    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[ACTIVATE] ✓ All 4 motors activated with encoder reset!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[DEACTIVATE] Emergency stop...");
    driver_->emergencyStop();
    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[DEACTIVATE] ✓ Motors stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileSystemInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[CLEANUP] Closing serial ports...");
    driver_->close();
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Interface Export
// ============================================================================

std::vector<hardware_interface::StateInterface>
MobileSystemInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < mobile_sdk::NUM_WHEELS; ++i)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MobileSystemInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < mobile_sdk::NUM_WHEELS; ++i)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    return command_interfaces;
}

// ============================================================================
// Control Loop
// ============================================================================

hardware_interface::return_type MobileSystemInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    driver_->readState();

    for (size_t i = 0; i < mobile_sdk::NUM_WHEELS; ++i)
    {
        hw_positions_[i]  = driver_->getPosition(i);
        hw_velocities_[i] = driver_->getVelocity(i);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileSystemInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    driver_->writeVelocity(
        hw_commands_[mobile_sdk::FRONT_LEFT],
        hw_commands_[mobile_sdk::FRONT_RIGHT],
        hw_commands_[mobile_sdk::REAR_LEFT],
        hw_commands_[mobile_sdk::REAR_RIGHT]);

    return hardware_interface::return_type::OK;
}

}  // namespace mobile_sdk_ros2

// ============================================================================
// Plugin registration
// ============================================================================
PLUGINLIB_EXPORT_CLASS(
    mobile_sdk_ros2::MobileSystemInterface,
    hardware_interface::SystemInterface)
