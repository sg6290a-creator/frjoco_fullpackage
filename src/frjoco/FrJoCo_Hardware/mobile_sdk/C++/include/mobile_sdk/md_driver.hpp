/**
 * ============================================================================
 * MDDriver — High-level MD200T 4WD motor controller (no ROS dependency)
 * ============================================================================
 *
 * 2개의 MD200T 듀얼 채널 드라이버 (front/rear) 를 통합 관리.
 * SerialPort + MDProtocol 을 조합하여 4륜 제어 API를 제공.
 *
 * 사용 예:
 *   mobile_sdk::MDDriverConfig cfg;
 *   cfg.port_front = "/dev/ttyUSB0";
 *   cfg.port_rear  = "/dev/ttyUSB1";
 *
 *   mobile_sdk::MDDriver driver(cfg);
 *   driver.open();
 *   driver.writeVelocity(1.0, 1.0, 1.0, 1.0);  // rad/s
 *   driver.readState();
 *   double pos = driver.getPosition(mobile_sdk::FRONT_LEFT);
 *   driver.close();
 *
 * ============================================================================
 */

#ifndef MOBILE_SDK__MD_DRIVER_HPP_
#define MOBILE_SDK__MD_DRIVER_HPP_

#include <array>
#include <mutex>

#include "mobile_sdk/serial_port.hpp"
#include "mobile_sdk/md_protocol.hpp"

namespace mobile_sdk
{

class MDDriver
{
public:
    explicit MDDriver(const MDDriverConfig& config);
    ~MDDriver();

    // Non-copyable
    MDDriver(const MDDriver&) = delete;
    MDDriver& operator=(const MDDriver&) = delete;

    // ----------------------------------------------------------------
    // Lifecycle
    // ----------------------------------------------------------------

    /// Open both serial ports. Returns true if both succeed.
    bool open();

    /// Close both serial ports.
    void close();

    /// Check if both serial ports are open.
    bool isOpen() const;

    // ----------------------------------------------------------------
    // Motor control
    // ----------------------------------------------------------------

    /**
     * Send velocity commands to all 4 wheels.
     * @param fl_rad_s  Front-left  wheel velocity (rad/s)
     * @param fr_rad_s  Front-right wheel velocity (rad/s)
     * @param rl_rad_s  Rear-left   wheel velocity (rad/s)
     * @param rr_rad_s  Rear-right  wheel velocity (rad/s)
     * @return true on success.
     */
    bool writeVelocity(double fl_rad_s, double fr_rad_s,
                       double rl_rad_s, double rr_rad_s);

    /// Emergency stop — send zero velocity 3 times.
    void emergencyStop();

    // ----------------------------------------------------------------
    // State reading
    // ----------------------------------------------------------------

    /**
     * Read from both serial ports and parse incoming packets.
     * Call this periodically (e.g. every control cycle).
     * @return true on success.
     */
    bool readState();

    /// Reset all motor states and encoder initialization.
    void resetState();

    // ----------------------------------------------------------------
    // State accessors
    // ----------------------------------------------------------------

    /// Get wheel position in radians (accumulated).
    double getPosition(size_t wheel_idx) const;

    /// Get wheel velocity in rad/s.
    double getVelocity(size_t wheel_idx) const;

    /// Get configuration (read-only reference).
    const MDDriverConfig& config() const { return config_; }

    /// Get tick_to_rad conversion factor.
    double tickToRad() const { return tick_to_rad_; }

private:
    // Internal helpers
    bool sendVelocity(SerialPort& serial, int16_t left_rpm, int16_t right_rpm, uint8_t motor_id);
    bool receiveData(SerialPort& serial, CommState& comm, MotorState& motor);
    void updateWheelState(size_t wheel_idx, int16_t rpm, int32_t position,
                          int32_t& last_tick, double& last_rad);

    // Configuration
    MDDriverConfig config_;
    double ppr_;
    double tick_to_rad_;

    // Serial ports
    SerialPort serial_front_;
    SerialPort serial_rear_;

    // Motor / comm state
    MotorState front_motor_;
    MotorState rear_motor_;
    CommState  front_comm_;
    CommState  rear_comm_;

    // Output state (4 wheels)
    std::array<double, NUM_WHEELS> positions_;
    std::array<double, NUM_WHEELS> velocities_;

    // Encoder initialization
    std::array<bool, NUM_WHEELS> encoder_initialized_;
    std::array<int,  NUM_WHEELS> encoder_init_count_;

    mutable std::mutex mutex_;
};

}  // namespace mobile_sdk

#endif  // MOBILE_SDK__MD_DRIVER_HPP_
