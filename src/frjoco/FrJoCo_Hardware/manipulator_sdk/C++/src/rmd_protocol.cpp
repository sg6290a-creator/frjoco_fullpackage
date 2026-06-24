/**
 * ============================================================================
 * RMDProtocol — Implementation
 * ============================================================================
 */

#include "manipulator_sdk/rmd_protocol.hpp"

#include <cmath>
#include <cstring>
#include <cstdlib>

namespace manipulator_sdk
{

// ════════════════════════════════════════════════════════════════
//  Command Builders
// ════════════════════════════════════════════════════════════════

void RMDProtocol::buildMotorOff(uint8_t data[8])
{
    std::memset(data, 0, 8);
    data[0] = RMD_CMD_MOTOR_OFF;  // 0x80
}

void RMDProtocol::buildMotorOn(uint8_t data[8])
{
    std::memset(data, 0, 8);
    data[0] = RMD_CMD_MOTOR_ON;  // 0x88
}

void RMDProtocol::buildReadStatus2(uint8_t data[8])
{
    std::memset(data, 0, 8);
    data[0] = RMD_CMD_READ_STATUS2;  // 0x9C
}

void RMDProtocol::buildReadMultiTurn(uint8_t data[8])
{
    std::memset(data, 0, 8);
    data[0] = RMD_CMD_READ_MULTI_TURN;  // 0x60
}

void RMDProtocol::buildPositionCtrl2(uint8_t data[8], const MotorCommand& cmd)
{
    double position_deg = radToDeg(cmd.position_rad);
    double velocity_dps = std::abs(radSToDps(cmd.velocity_rads));

    // Apply velocity limits
    if (velocity_dps < 5.0) velocity_dps = cmd.default_vel_dps;
    if (velocity_dps > cmd.max_vel_dps) velocity_dps = cmd.max_vel_dps;

    int32_t  angle_ctrl = static_cast<int32_t>(position_deg * 100);  // 0.01 deg units
    uint16_t speed_ctrl = static_cast<uint16_t>(velocity_dps);

    data[0] = RMD_CMD_POSITION_CTRL2;  // 0xA4
    data[1] = 0x00;                     // spin direction (auto)
    data[2] = speed_ctrl & 0xFF;
    data[3] = (speed_ctrl >> 8) & 0xFF;
    data[4] = angle_ctrl & 0xFF;
    data[5] = (angle_ctrl >> 8) & 0xFF;
    data[6] = (angle_ctrl >> 16) & 0xFF;
    data[7] = (angle_ctrl >> 24) & 0xFF;
}

void RMDProtocol::buildSetAcceleration(uint8_t data[8], uint32_t accel_dps2)
{
    data[0] = RMD_CMD_SET_ACCEL;  // 0x43
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = accel_dps2 & 0xFF;
    data[5] = (accel_dps2 >> 8) & 0xFF;
    data[6] = (accel_dps2 >> 16) & 0xFF;
    data[7] = (accel_dps2 >> 24) & 0xFF;
}

// ════════════════════════════════════════════════════════════════
//  Response Parser
// ════════════════════════════════════════════════════════════════

MotorState RMDProtocol::parseStatus2(const uint8_t rx_data[8], uint8_t rx_len,
                                      double torque_constant)
{
    MotorState state;

    // 0x9C (READ_STATUS2) 와 0xA4 (POSITION_CTRL2) 응답이 같은 포맷.
    if (rx_len < 8 ||
        (rx_data[0] != RMD_CMD_READ_STATUS2 &&
         rx_data[0] != RMD_CMD_POSITION_CTRL2)) {
        return state;  // valid = false
    }

    // Byte 1: temperature (°C)
    state.temperature = static_cast<int8_t>(rx_data[1]);

    // Byte 2-3: torque current (int16, 0.01A)
    int16_t current_raw = static_cast<int16_t>((rx_data[3] << 8) | rx_data[2]);

    // Byte 4-5: speed (int16, 1 dps)
    int16_t speed_raw = static_cast<int16_t>((rx_data[5] << 8) | rx_data[4]);

    // Byte 6-7: shaft angle (int16, 1°/LSB, multi-turn capable).
    //   src2 (검증된 동작 버전) 와 동일하게 정규화 없이 그대로 multi-turn 각도 사용.
    int16_t shaft_angle = static_cast<int16_t>((rx_data[7] << 8) | rx_data[6]);
    state.position_rad  = degToRad(static_cast<double>(shaft_angle));
    state.velocity_rads = dpsToRadS(static_cast<double>(speed_raw));
    state.effort_nm     = current_raw * 0.01 * torque_constant;
    state.valid         = true;

    return state;
}

MotorState RMDProtocol::parseMultiTurn(const uint8_t rx_data[8], uint8_t rx_len)
{
    MotorState state;
    if (rx_len < 8 || rx_data[0] != RMD_CMD_READ_MULTI_TURN) {
        return state;
    }

    // RMD V3 0x60 응답 레이아웃:
    //   data[0]   = 0x60
    //   data[1..3]= NULL padding
    //   data[4..7]= int32 little-endian, 0.01°/LSB (출력 축 다회전)
    int32_t raw = static_cast<int32_t>(
        static_cast<uint32_t>(rx_data[4]) |
        (static_cast<uint32_t>(rx_data[5]) << 8)  |
        (static_cast<uint32_t>(rx_data[6]) << 16) |
        (static_cast<uint32_t>(rx_data[7]) << 24));

    const double angle_deg = static_cast<double>(raw) * 0.01;
    state.position_rad = degToRad(angle_deg);
    state.valid = true;
    return state;
}

}  // namespace manipulator_sdk
