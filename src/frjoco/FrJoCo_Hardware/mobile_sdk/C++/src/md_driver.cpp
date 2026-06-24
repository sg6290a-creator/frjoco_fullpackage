/**
 * ============================================================================
 * MDDriver — Implementation
 * ============================================================================
 */

#include "mobile_sdk/md_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mobile_sdk
{

// ============================================================================
// Constructor / Destructor
// ============================================================================

MDDriver::MDDriver(const MDDriverConfig& config)
    : config_(config)
{
    ppr_        = computePPR(config_.poles, config_.gear_ratio);
    tick_to_rad_ = computeTickToRad(config_.poles, config_.gear_ratio);

    initMotorState(front_motor_);
    initMotorState(rear_motor_);
    initCommState(front_comm_);
    initCommState(rear_comm_);

    positions_.fill(0.0);
    velocities_.fill(0.0);
    encoder_initialized_.fill(false);
    encoder_init_count_.fill(0);

    printf("[MDDriver] Config: front=%s rear=%s baud=%d gear=%d poles=%d\n",
           config_.port_front.c_str(), config_.port_rear.c_str(),
           config_.baudrate, config_.gear_ratio, config_.poles);
    printf("[MDDriver] PPR=%.0f  tick_to_rad=%.8f\n", ppr_, tick_to_rad_);
}

MDDriver::~MDDriver()
{
    close();
}

// ============================================================================
// Lifecycle
// ============================================================================

bool MDDriver::open()
{
    bool ok = true;

    if (!serial_front_.open(config_.port_front, config_.baudrate))
    {
        fprintf(stderr, "[MDDriver] Failed to open FRONT port: %s\n",
                config_.port_front.c_str());
        ok = false;
    }

    if (!serial_rear_.open(config_.port_rear, config_.baudrate))
    {
        fprintf(stderr, "[MDDriver] Failed to open REAR port: %s\n",
                config_.port_rear.c_str());
        ok = false;
    }

    if (ok)
    {
        printf("[MDDriver] Both serial ports opened successfully.\n");
    }

    return ok;
}

void MDDriver::close()
{
    serial_front_.close();
    serial_rear_.close();
}

bool MDDriver::isOpen() const
{
    return serial_front_.isOpen() && serial_rear_.isOpen();
}

// ============================================================================
// Motor control
// ============================================================================

bool MDDriver::sendVelocity(SerialPort& serial, int16_t left_rpm, int16_t right_rpm, uint8_t motor_id)
{
    if (!serial.isOpen())
        return false;

    uint8_t packet[MAX_PACKET_SIZE];
    size_t len = buildVelocityPacket(
        packet,
        static_cast<uint8_t>(config_.id_mdt),
        static_cast<uint8_t>(config_.id_mdui),
        motor_id,
        left_rpm, right_rpm);

    ssize_t written = serial.write(packet, len);
    if (written < 0)
    {
        fprintf(stderr, "[MDDriver] Serial write error on %s\n", serial.port().c_str());
        return false;
    }

    serial.flush();
    return true;
}

bool MDDriver::writeVelocity(double fl_rad_s, double fr_rad_s,
                              double rl_rad_s, double rr_rad_s)
{
    std::lock_guard<std::mutex> lock(mutex_);

    int16_t fl_rpm = radPerSecToRpm(fl_rad_s, config_.gear_ratio);
    int16_t fr_rpm = radPerSecToRpm(fr_rad_s, config_.gear_ratio);  
    int16_t rl_rpm = radPerSecToRpm(rl_rad_s, config_.gear_ratio);
    int16_t rr_rpm = radPerSecToRpm(rr_rad_s, config_.gear_ratio);

    // Front driver: CH1=-FR, CH2=-FL (채널 반전 보정 — 원본과 동일)
    bool ok1 = sendVelocity(serial_front_, fl_rpm, -fr_rpm,
                            static_cast<uint8_t>(config_.front_motor_id));

    // Rear driver: CH1=RL, CH2=RR
    bool ok2 = sendVelocity(serial_rear_, rl_rpm, -rr_rpm,
                            static_cast<uint8_t>(config_.rear_motor_id));

    return ok1 && ok2;
}

void MDDriver::emergencyStop()
{
    for (int i = 0; i < 3; ++i)
    {
        if (serial_front_.isOpen())
            sendVelocity(serial_front_, 0, 0, static_cast<uint8_t>(config_.front_motor_id));
        if (serial_rear_.isOpen())
            sendVelocity(serial_rear_, 0, 0, static_cast<uint8_t>(config_.rear_motor_id));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("[MDDriver] Emergency stop sent.\n");
}

// ============================================================================
// State reading
// ============================================================================

bool MDDriver::receiveData(SerialPort& serial, CommState& comm, MotorState& motor)
{
    if (!serial.isOpen())
        return false;

    size_t avail = serial.available();
    if (avail == 0)
        return true;  // no data yet — not an error

    uint8_t buffer[MAX_DATA_SIZE];
    size_t read_size = (avail < MAX_DATA_SIZE) ? avail : MAX_DATA_SIZE;

    ssize_t n = serial.read(buffer, read_size);
    if (n < 0)
    {
        fprintf(stderr, "[MDDriver] Serial read error on %s\n", serial.port().c_str());
        return false;
    }

    if (motor.init_motor)
    {
        motor.init_motor = false;
    }
    else
    {
        analyzeReceivedData(buffer, static_cast<size_t>(n), comm, motor,
                            static_cast<uint8_t>(config_.id_mdt),
                            static_cast<uint8_t>(config_.id_mdui));
    }

    return true;
}

bool MDDriver::readState()
{
    std::lock_guard<std::mutex> lock(mutex_);

    receiveData(serial_front_, front_comm_, front_motor_);
    receiveData(serial_rear_,  rear_comm_,  rear_motor_);

    // Front Driver: FL(left), FR(right)
    updateWheelState(FRONT_LEFT,  front_motor_.left_rpm,  front_motor_.left_position,
                     front_motor_.left_last_tick,  front_motor_.left_last_rad);
    updateWheelState(FRONT_RIGHT, front_motor_.right_rpm, front_motor_.right_position,
                     front_motor_.right_last_tick, front_motor_.right_last_rad);

    // Rear Driver: RL(left), RR(right)
    updateWheelState(REAR_LEFT,  rear_motor_.left_rpm,  rear_motor_.left_position,
                     rear_motor_.left_last_tick,  rear_motor_.left_last_rad);
    updateWheelState(REAR_RIGHT, rear_motor_.right_rpm, rear_motor_.right_position,
                     rear_motor_.right_last_tick, rear_motor_.right_last_rad);

    return true;
}

void MDDriver::resetState()
{
    std::lock_guard<std::mutex> lock(mutex_);

    positions_.fill(0.0);
    velocities_.fill(0.0);
    initMotorState(front_motor_);
    initMotorState(rear_motor_);
    initCommState(front_comm_);
    initCommState(rear_comm_);
    encoder_initialized_.fill(false);
    encoder_init_count_.fill(0);
}

// ============================================================================
// State accessors
// ============================================================================

double MDDriver::getPosition(size_t wheel_idx) const
{
    if (wheel_idx >= NUM_WHEELS) return 0.0;
    std::lock_guard<std::mutex> lock(mutex_);
    return positions_[wheel_idx];
}

double MDDriver::getVelocity(size_t wheel_idx) const
{
    if (wheel_idx >= NUM_WHEELS) return 0.0;
    std::lock_guard<std::mutex> lock(mutex_);
    return velocities_[wheel_idx];
}

// ============================================================================
// Encoder → wheel state
// ============================================================================

void MDDriver::updateWheelState(size_t wheel_idx, int16_t rpm, int32_t position,
                                 int32_t& last_tick, double& last_rad)
{
    if (wheel_idx >= NUM_WHEELS) return;

    // Encoder initialization (skip first 5 readings)
    if (!encoder_initialized_[wheel_idx])
    {
        if (encoder_init_count_[wheel_idx]++ < 5)
        {
            last_tick = position;
            last_rad  = 0.0;
            positions_[wheel_idx]  = 0.0;
            velocities_[wheel_idx] = rpmToRadPerSec(rpm, config_.gear_ratio);
            return;
        }

        last_tick = position;
        last_rad  = 0.0;
        positions_[wheel_idx] = 0.0;
        encoder_initialized_[wheel_idx] = true;
        printf("[MDDriver] Wheel %zu encoder initialized at tick=%d\n",
               wheel_idx, position);
    }

    // Tick difference
    int32_t tick_diff = position - last_tick;

    // Overflow detection
    const int32_t OVERFLOW_THRESHOLD = 1000000;
    if (std::abs(tick_diff) > OVERFLOW_THRESHOLD)
    {
        last_tick = position;
        velocities_[wheel_idx] = rpmToRadPerSec(rpm, config_.gear_ratio);
        return;
    }

    last_tick = position;

    // Accumulate radians
    double rad_diff = tick_diff * tick_to_rad_;
    last_rad += rad_diff;

    positions_[wheel_idx]  = last_rad;
    velocities_[wheel_idx] = rpmToRadPerSec(rpm, config_.gear_ratio);
}

}  // namespace mobile_sdk
