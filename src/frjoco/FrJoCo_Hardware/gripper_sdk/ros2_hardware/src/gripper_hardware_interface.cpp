#include "gripper_hardware/gripper_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace gripper_hardware
{

namespace
{
std::string getParam(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key,
  const std::string & default_value)
{
  const auto it = params.find(key);
  return it == params.end() ? default_value : it->second;
}

bool getBoolParam(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key,
  bool default_value)
{
  const std::string value = getParam(params, key, default_value ? "true" : "false");
  return value == "true" || value == "1" || value == "yes" || value == "on";
}
}  // namespace

GripperHardwareInterface::~GripperHardwareInterface()
{
  closeDynamixel();
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & params = info_.hardware_parameters;

  // Keep old_ws parameter aliases while defaulting to U2D2/Dynamixel direct control.
  port_ = getParam(
    params, "port_name", getParam(params, "gripper_port", getParam(params, "port", "/dev/ttyUSB0")));
  baudrate_ = std::stoi(
    getParam(params, "baud_rate", getParam(params, "gripper_baudrate", getParam(params, "baudrate", "1000000"))));
  protocol_version_ = std::stod(getParam(params, "protocol_version", "2.0"));
  dxl_id_ = static_cast<uint8_t>(
    std::stoi(getParam(params, "dxl_id", getParam(params, "gripper_id", getParam(params, "id", "15")))));
  min_position_ = std::stod(getParam(params, "min_position", "-0.010"));
  max_position_ = std::stod(getParam(params, "max_position", "0.019"));
  min_dxl_position_ = std::stoi(getParam(params, "min_dxl_position", "3000"));
  max_dxl_position_ = std::stoi(getParam(params, "max_dxl_position", "3600"));
  calibrate_open_on_activate_ = getBoolParam(params, "calibrate_open_on_activate", false);
  calibrated_open_is_high_ = getBoolParam(params, "calibrated_open_is_high", true);
  calibrated_close_delta_ticks_ = std::stoi(getParam(params, "calibrated_close_delta_ticks", "500"));
  profile_acceleration_address_ = static_cast<uint16_t>(
    std::stoi(getParam(params, "profile_acceleration_address", "108")));
  profile_velocity_address_ = static_cast<uint16_t>(
    std::stoi(getParam(params, "profile_velocity_address", "112")));
  profile_acceleration_ = static_cast<uint32_t>(
    std::stoul(getParam(params, "profile_acceleration", "10")));
  profile_velocity_ = static_cast<uint32_t>(
    std::stoul(getParam(params, "profile_velocity", "40")));
  command_deadband_ticks_ = std::stoi(getParam(params, "command_deadband_ticks", "10"));
  torque_enable_address_ = static_cast<uint16_t>(
    std::stoi(getParam(params, "torque_enable_address", "64")));
  operating_mode_address_ = static_cast<uint16_t>(
    std::stoi(getParam(params, "operating_mode_address", "11")));
  goal_position_address_ = static_cast<uint16_t>(
    std::stoi(getParam(params, "goal_position_address", "116")));
  present_position_address_ = static_cast<uint16_t>(
    std::stoi(getParam(params, "present_position_address", "132")));
  position_operating_mode_ = static_cast<uint8_t>(
    std::stoi(getParam(params, "position_operating_mode", "3")));
  set_operating_mode_on_activate_ = getBoolParam(params, "set_operating_mode_on_activate", true);
  torque_on_activate_ = getBoolParam(params, "torque_on_activate", true);
  torque_off_on_deactivate_ = getBoolParam(params, "torque_off_on_deactivate", false);
  ping_on_activate_ = getBoolParam(params, "ping_on_activate", false);

  if (info_.joints.empty() || info_.joints.size() > 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GripperHardwareInterface"),
      "Expected 1 or 2 gripper joints, got %zu",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  has_right_mimic_joint_ = info_.joints.size() == 2;
  left_position_command_ = max_position_;
  right_position_command_ = max_position_;
  left_position_state_ = max_position_;
  right_position_state_ = max_position_;

  RCLCPP_INFO(
    rclcpp::get_logger("GripperHardwareInterface"),
    "U2D2 gripper hardware: port=%s baud=%d id=%u protocol=%.1f pos=[%.3f, %.3f] dxl=[%d, %d] calibrate_open=%s delta=%d",
    port_.c_str(),
    baudrate_,
    dxl_id_,
    protocol_version_,
    min_position_,
    max_position_,
    min_dxl_position_,
    max_dxl_position_,
    calibrate_open_on_activate_ ? "true" : "false",
    calibrated_close_delta_ticks_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
GripperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &left_position_state_);
  state_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_VELOCITY,
    &left_velocity_state_);

  if (has_right_mimic_joint_) {
    state_interfaces.emplace_back(
      info_.joints[1].name,
      hardware_interface::HW_IF_POSITION,
      &right_position_state_);
    state_interfaces.emplace_back(
      info_.joints[1].name,
      hardware_interface::HW_IF_VELOCITY,
      &right_velocity_state_);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
GripperHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    info_.joints[0].name,
    hardware_interface::HW_IF_POSITION,
    &left_position_command_);

  if (has_right_mimic_joint_) {
    command_interfaces.emplace_back(
      info_.joints[1].name,
      hardware_interface::HW_IF_POSITION,
      &right_position_command_);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!openDynamixel()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GripperHardwareInterface"),
      "Failed to open U2D2 gripper port: %s",
      port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!configureDynamixel()) {
    closeDynamixel();
    return hardware_interface::CallbackReturn::ERROR;
  }

  int ticks = positionToTicks(max_position_);
  if (readPositionTicks(ticks)) {
    if (calibrate_open_on_activate_) {
      calibrateOpenRangeFromTicks(ticks);
    }
    left_position_state_ = ticksToPosition(ticks);
    right_position_state_ = left_position_state_;
  } else if (calibrate_open_on_activate_) {
    RCLCPP_WARN(
      rclcpp::get_logger("GripperHardwareInterface"),
      "Open calibration requested, but present position read failed. Using configured dxl range [%d, %d]",
      min_dxl_position_,
      max_dxl_position_);
  }

  left_position_state_ = max_position_;
  right_position_state_ = max_position_;
  left_position_command_ = max_position_;
  right_position_command_ = max_position_;
  last_ticks_sent_ = ticks;

  const int open_ticks = positionToTicks(max_position_);
  if (sendPositionTicks(open_ticks)) {
    last_ticks_sent_ = open_ticks;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("GripperHardwareInterface"),
      "Failed to send initial gripper open command");
  }

  RCLCPP_INFO(
    rclcpp::get_logger("GripperHardwareInterface"),
    "U2D2 gripper connected: %s @ %d baud, DXL ID %u, initial open=%.3f dxl=[%d, %d]",
    port_.c_str(),
    baudrate_,
    dxl_id_,
    max_position_,
    min_dxl_position_,
    max_dxl_position_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GripperHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (connected_ && torque_off_on_deactivate_) {
    setTorque(false);
  }
  closeDynamixel();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GripperHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!connected_) {
    return hardware_interface::return_type::OK;
  }

  int ticks = 0;
  if (readPositionTicks(ticks)) {
    const double previous_position = left_position_state_;
    left_position_state_ = ticksToPosition(ticks);
    const double seconds = period.seconds();
    left_velocity_state_ = seconds > 1e-9 ? (left_position_state_ - previous_position) / seconds : 0.0;
    read_failure_count_ = 0;
  } else {
    read_failure_count_++;
    if (read_failure_count_ == 1 || read_failure_count_ % 100 == 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("GripperHardwareInterface"),
        "Failed to read gripper present position (%d consecutive failures)",
        read_failure_count_);
    }
  }

  right_position_state_ = left_position_state_;
  right_velocity_state_ = left_velocity_state_;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GripperHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!connected_) {
    return hardware_interface::return_type::OK;
  }

  const int ticks = positionToTicks(left_position_command_);
  if (std::abs(ticks - last_ticks_sent_) <= command_deadband_ticks_) {
    return hardware_interface::return_type::OK;
  }

  if (sendPositionTicks(ticks)) {
    last_ticks_sent_ = ticks;
  } else {
    RCLCPP_WARN(rclcpp::get_logger("GripperHardwareInterface"), "Gripper Dynamixel write failed");
  }

  return hardware_interface::return_type::OK;
}

bool GripperHardwareInterface::openDynamixel()
{
  if (port_.empty() || port_ == "none") {
    return false;
  }

  port_handler_ = dynamixel::PortHandler::getPortHandler(port_.c_str());
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(
    static_cast<float>(protocol_version_));

  if (port_handler_ == nullptr || packet_handler_ == nullptr) {
    return false;
  }

  if (!port_handler_->openPort()) {
    closeDynamixel();
    return false;
  }

  if (!port_handler_->setBaudRate(baudrate_)) {
    closeDynamixel();
    return false;
  }

  connected_ = true;
  return true;
}

void GripperHardwareInterface::closeDynamixel()
{
  if (port_handler_ != nullptr) {
    port_handler_->closePort();
    delete port_handler_;
    port_handler_ = nullptr;
  }
  packet_handler_ = nullptr;
  connected_ = false;
}

bool GripperHardwareInterface::configureDynamixel()
{
  if (ping_on_activate_) {
    uint16_t model_number = 0;
    uint8_t error = 0;
    const int result = packet_handler_->ping(port_handler_, dxl_id_, &model_number, &error);
    if (!isPacketOk(result, error, "ping")) {
      return false;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("GripperHardwareInterface"),
      "Detected gripper Dynamixel model number: %u",
      model_number);
  }

  if (set_operating_mode_on_activate_) {
    if (!setTorque(false)) {
      return false;
    }
    if (!writeRegister1Byte(
        operating_mode_address_, position_operating_mode_, "set operating mode"))
    {
      return false;
    }
  }

  if (profile_acceleration_ > 0 &&
    !writeRegister4Byte(profile_acceleration_address_, profile_acceleration_, "set profile acceleration"))
  {
    return false;
  }
  if (profile_velocity_ > 0 &&
    !writeRegister4Byte(profile_velocity_address_, profile_velocity_, "set profile velocity"))
  {
    return false;
  }

  if (torque_on_activate_ && !setTorque(true)) {
    return false;
  }

  return true;
}

bool GripperHardwareInterface::setTorque(bool enabled)
{
  return writeRegister1Byte(
    torque_enable_address_,
    static_cast<uint8_t>(enabled ? 1 : 0),
    enabled ? "enable torque" : "disable torque");
}

bool GripperHardwareInterface::writeRegister1Byte(
  uint16_t address, uint8_t data, const char * label)
{
  uint8_t error = 0;
  const int result = packet_handler_->write1ByteTxRx(
    port_handler_, dxl_id_, address, data, &error);
  return isPacketOk(result, error, label);
}

bool GripperHardwareInterface::writeRegister4Byte(
  uint16_t address, uint32_t data, const char * label)
{
  uint8_t error = 0;
  const int result = packet_handler_->write4ByteTxRx(
    port_handler_, dxl_id_, address, data, &error);
  return isPacketOk(result, error, label);
}

bool GripperHardwareInterface::readPositionTicks(int & ticks)
{
  uint8_t error = 0;
  uint32_t value = 0;
  const int result = packet_handler_->read4ByteTxRx(
    port_handler_, dxl_id_, present_position_address_, &value, &error);
  if (result != COMM_SUCCESS || error != 0) {
    return false;
  }

  ticks = static_cast<int>(value);
  return true;
}

bool GripperHardwareInterface::sendPositionTicks(int ticks)
{
  ticks = std::clamp(ticks, 0, 4095);
  return writeRegister4Byte(
    goal_position_address_,
    static_cast<uint32_t>(ticks),
    "write goal position");
}

void GripperHardwareInterface::calibrateOpenRangeFromTicks(int open_ticks)
{
  open_ticks = std::clamp(open_ticks, 0, 4095);
  const int close_delta = std::max(1, std::abs(calibrated_close_delta_ticks_));

  if (calibrated_open_is_high_) {
    max_dxl_position_ = open_ticks;
    min_dxl_position_ = std::clamp(open_ticks - close_delta, 0, 4095);
  } else {
    min_dxl_position_ = open_ticks;
    max_dxl_position_ = std::clamp(open_ticks + close_delta, 0, 4095);
  }

  if (min_dxl_position_ == max_dxl_position_) {
    RCLCPP_WARN(
      rclcpp::get_logger("GripperHardwareInterface"),
      "Calibrated gripper dxl range collapsed at %d. Falling back to a 1 tick span.",
      open_ticks);
    if (max_dxl_position_ < 4095) {
      max_dxl_position_ += 1;
    } else {
      min_dxl_position_ -= 1;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("GripperHardwareInterface"),
    "Calibrated current gripper tick as open: open_tick=%d dxl=[%d, %d] close_delta=%d",
    open_ticks,
    min_dxl_position_,
    max_dxl_position_,
    close_delta);
}

int GripperHardwareInterface::positionToTicks(double position_m) const
{
  const double denom = max_position_ - min_position_;
  if (std::abs(denom) < 1e-9) {
    return min_dxl_position_;
  }

  double ratio = (position_m - min_position_) / denom;
  ratio = std::clamp(ratio, 0.0, 1.0);

  return min_dxl_position_ +
    static_cast<int>(ratio * (max_dxl_position_ - min_dxl_position_));
}

double GripperHardwareInterface::ticksToPosition(int ticks) const
{
  const double denom = static_cast<double>(max_dxl_position_ - min_dxl_position_);
  if (std::abs(denom) < 1e-9) {
    return min_position_;
  }

  double ratio = (static_cast<double>(ticks) - min_dxl_position_) / denom;
  ratio = std::clamp(ratio, 0.0, 1.0);

  return min_position_ + ratio * (max_position_ - min_position_);
}

bool GripperHardwareInterface::isPacketOk(int result, uint8_t error, const char * label) const
{
  if (result != COMM_SUCCESS) {
    RCLCPP_WARN(
      rclcpp::get_logger("GripperHardwareInterface"),
      "%s failed: %s",
      label,
      packet_handler_->getTxRxResult(result));
    return false;
  }

  if (error != 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("GripperHardwareInterface"),
      "%s returned DXL error: %s",
      label,
      packet_handler_->getRxPacketError(error));
    return false;
  }

  return true;
}

}  // namespace gripper_hardware

PLUGINLIB_EXPORT_CLASS(
  gripper_hardware::GripperHardwareInterface,
  hardware_interface::SystemInterface)
