/**
 * ============================================================================
 * MobileSystemInterface — ros2_control SystemInterface for MD 4WD
 * ============================================================================
 *
 * MDDriver (pure C++) 를 has-a 멤버로 사용하는 얇은 ROS2 브리지.
 * Lifecycle 콜백에서 MDDriver 를 생성/열기/닫기 하고,
 * read()/write() 에서 MDDriver 의 readState()/writeVelocity() 를 호출.
 *
 * ============================================================================
 */

#ifndef MOBILE_SDK_ROS2__MOBILE_SYSTEM_INTERFACE_HPP_
#define MOBILE_SDK_ROS2__MOBILE_SYSTEM_INTERFACE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mobile_sdk/md_driver.hpp"

namespace mobile_sdk_ros2
{

class MobileSystemInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MobileSystemInterface)

    // Lifecycle callbacks
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    // Interface exports
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // Control loop
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    static constexpr const char* LOGGER = "MobileSystemInterface";

    // Native driver (pure C++)
    std::unique_ptr<mobile_sdk::MDDriver> driver_;

    // Hardware interface buffers
    std::array<double, mobile_sdk::NUM_WHEELS> hw_commands_{};
    std::array<double, mobile_sdk::NUM_WHEELS> hw_positions_{};
    std::array<double, mobile_sdk::NUM_WHEELS> hw_velocities_{};
    std::vector<std::string> joint_names_;
};

}  // namespace mobile_sdk_ros2

#endif  // MOBILE_SDK_ROS2__MOBILE_SYSTEM_INTERFACE_HPP_
