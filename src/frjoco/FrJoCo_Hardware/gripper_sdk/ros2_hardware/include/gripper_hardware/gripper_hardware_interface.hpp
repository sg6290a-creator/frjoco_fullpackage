#ifndef GRIPPER_HARDWARE__GRIPPER_HARDWARE_INTERFACE_HPP_
#define GRIPPER_HARDWARE__GRIPPER_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace dynamixel
{
class PacketHandler;
class PortHandler;
}  // namespace dynamixel

namespace gripper_hardware
{

class GripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GripperHardwareInterface)

  GripperHardwareInterface() = default;
  ~GripperHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool openDynamixel();
  void closeDynamixel();
  bool configureDynamixel();
  bool setTorque(bool enabled);
  bool writeRegister1Byte(uint16_t address, uint8_t data, const char * label);
  bool writeRegister4Byte(uint16_t address, uint32_t data, const char * label);
  bool readPositionTicks(int & ticks);
  bool sendPositionTicks(int ticks);
  void calibrateOpenRangeFromTicks(int open_ticks);
  int positionToTicks(double position_m) const;
  double ticksToPosition(int ticks) const;
  bool isPacketOk(int result, uint8_t error, const char * label) const;

  std::string port_{"/dev/ttyUSB0"};
  int baudrate_{1000000};
  double protocol_version_{2.0};
  uint8_t dxl_id_{15};

  uint16_t torque_enable_address_{64};
  uint16_t operating_mode_address_{11};
  uint16_t profile_acceleration_address_{108};
  uint16_t profile_velocity_address_{112};
  uint16_t goal_position_address_{116};
  uint16_t present_position_address_{132};
  uint8_t position_operating_mode_{3};
  bool set_operating_mode_on_activate_{true};
  bool torque_on_activate_{true};
  bool torque_off_on_deactivate_{false};
  bool ping_on_activate_{true};

  double min_position_{-0.010};
  double max_position_{0.019};
  int min_dxl_position_{3000};
  int max_dxl_position_{3600};
  bool calibrate_open_on_activate_{false};
  bool calibrated_open_is_high_{true};
  int calibrated_close_delta_ticks_{500};
  uint32_t profile_acceleration_{10};
  uint32_t profile_velocity_{40};
  int command_deadband_ticks_{10};

  dynamixel::PortHandler * port_handler_{nullptr};
  dynamixel::PacketHandler * packet_handler_{nullptr};
  bool connected_{false};
  int last_ticks_sent_{-1};
  int read_failure_count_{0};

  double left_position_state_{0.0};
  double left_velocity_state_{0.0};
  double left_position_command_{0.0};

  double right_position_state_{0.0};
  double right_velocity_state_{0.0};
  double right_position_command_{0.0};

  bool has_right_mimic_joint_{false};
};

}  // namespace gripper_hardware

#endif  // GRIPPER_HARDWARE__GRIPPER_HARDWARE_INTERFACE_HPP_
