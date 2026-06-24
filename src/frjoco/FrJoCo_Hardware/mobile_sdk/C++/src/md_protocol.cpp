/**
 * ============================================================================
 * MD Motor Protocol — Implementation
 * ============================================================================
 */

#include "mobile_sdk/md_protocol.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mobile_sdk
{

// ============================================================================
// Struct initializers
// ============================================================================

void initMotorState(MotorState& motor)
{
    motor.left_rpm       = 0;
    motor.right_rpm      = 0;
    motor.left_position  = 0;
    motor.right_position = 0;
    motor.left_last_rad  = 0.0;
    motor.right_last_rad = 0.0;
    motor.left_last_tick = 0;
    motor.right_last_tick = 0;
    motor.init_motor     = true;
}

void initCommState(CommState& comm)
{
    memset(comm.recv_buf, 0, sizeof(comm.recv_buf));
    comm.step         = 0;
    comm.packet_num   = 0;
    comm.checksum     = 0;
    comm.max_data_num = 0;
    comm.data_num     = 0;
    comm.packet_ok    = false;
    comm.error_count  = 0;
    comm.header_count = 0;
}

// ============================================================================
// Byte conversion helpers
// ============================================================================

IByte short2Byte(int16_t value)
{
    IByte result;
    result.low  = static_cast<uint8_t>(value & 0xFF);
    result.high = static_cast<uint8_t>((value >> 8) & 0xFF);
    return result;
}

int16_t byte2Short(uint8_t low, uint8_t high)
{
    return static_cast<int16_t>((high << 8) | low);
}

int32_t byte2Long(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
    return static_cast<int32_t>((b4 << 24) | (b3 << 16) | (b2 << 8) | b1);
}

// ============================================================================
// Packet builder
// ============================================================================

size_t buildVelocityPacket(uint8_t* packet,
                           uint8_t id_mdt, uint8_t id_mdui, uint8_t motor_id,
                           int16_t left_rpm, int16_t right_rpm)
{
    uint8_t checksum = 0;

    // Header
    packet[0] = id_mdt;              // RMID: receiver (183 = MDT)
    packet[1] = id_mdui;             // TMID: sender   (184 = PC)
    packet[2] = motor_id;            // Motor ID (1 or 2)
    packet[3] = PID_PNT_VEL_CMD;    // PID: 207

    checksum = packet[0] + packet[1] + packet[2] + packet[3];

    // Data
    packet[4] = 7;  // DataSize = 7 bytes

    IByte left_bytes  = short2Byte(left_rpm);
    IByte right_bytes = short2Byte(right_rpm);

    packet[5]  = 1;                   // D1 ENABLE (CH1)
    packet[6]  = left_bytes.low;      // D1 RPM low
    packet[7]  = left_bytes.high;     // D1 RPM high
    packet[8]  = 1;                   // D2 ENABLE (CH2)
    packet[9]  = right_bytes.low;     // D2 RPM low
    packet[10] = right_bytes.high;    // D2 RPM high
    packet[11] = REQUEST_PNT_MAIN_DATA;  // Return data request

    for (int i = 4; i <= 11; ++i)
    {
        checksum += packet[i];
    }

    packet[12] = static_cast<uint8_t>(~checksum + 1);  // 2's complement checksum

    return 13;
}

// ============================================================================
// processReceivedData — extract RPM / position from validated packet
// ============================================================================

void processReceivedData(const CommState& comm, MotorState& motor)
{
    uint8_t pid      = comm.recv_buf[3];
    uint8_t dataSize = comm.recv_buf[4];
    const uint8_t* data = &comm.recv_buf[5];

    (void)dataSize;  // used only for debug

    // PID 210 (0xD2): dual-channel response — 18 data bytes
    if (pid == PID_MAIN_DATA_ALT)
    {
        // CH1 (left):  D0-D1=RPM, D5-D8=Position
        motor.left_rpm       = byte2Short(data[0], data[1]);
        motor.left_position  = byte2Long(data[5], data[6], data[7], data[8]);

        // CH2 (right): D9-D10=RPM, D14-D17=Position
        motor.right_rpm      = byte2Short(data[9], data[10]);
        motor.right_position = byte2Long(data[14], data[15], data[16], data[17]);
    }
    // PID 193 (0xC1): single-channel response — 17 data bytes
    else if (pid == PID_MAIN_DATA)
    {
        motor.left_rpm       = byte2Short(data[0], data[1]);
        motor.left_position  = byte2Long(data[10], data[11], data[12], data[13]);

        // Single-channel: same values for right
        motor.right_rpm      = motor.left_rpm;
        motor.right_position = motor.left_position;
    }
}

// ============================================================================
// analyzeReceivedData — packet state machine
// ============================================================================

int analyzeReceivedData(const uint8_t* buffer, size_t size,
                        CommState& comm, MotorState& motor,
                        uint8_t id_mdt, uint8_t id_mdui)
{
    for (size_t j = 0; j < size; ++j)
    {
        // Overflow guard
        if (comm.packet_num >= MAX_PACKET_SIZE)
        {
            comm.step         = 0;
            comm.packet_num   = 0;
            comm.checksum     = 0;
            comm.header_count = 0;
            continue;
        }

        switch (comm.step)
        {
            case 0:  // RMID / TMID — need 2 matching headers
                if (buffer[j] == id_mdt || buffer[j] == id_mdui)
                {
                    comm.checksum += buffer[j];
                    comm.recv_buf[comm.packet_num++] = buffer[j];
                    comm.error_count = 0;
                    comm.header_count++;
                    if (comm.header_count >= 2)
                    {
                        comm.step++;
                        comm.header_count = 0;
                    }
                }
                else
                {
                    comm.header_count = 0;
                    comm.step         = 0;
                    comm.packet_num   = 0;
                    comm.checksum     = 0;
                    comm.error_count++;
                }
                break;

            case 1:  // Motor ID (1 or 2)
                if (buffer[j] == 1 || buffer[j] == 2)
                {
                    comm.checksum += buffer[j];
                    comm.recv_buf[comm.packet_num++] = buffer[j];
                    comm.step++;
                    comm.error_count = 0;
                }
                else
                {
                    comm.step         = 0;
                    comm.packet_num   = 0;
                    comm.checksum     = 0;
                    comm.header_count = 0;
                    comm.error_count++;
                }
                break;

            case 2:  // PID
                comm.checksum += buffer[j];
                comm.recv_buf[comm.packet_num++] = buffer[j];
                comm.step++;
                break;

            case 3:  // DataSize
                comm.max_data_num = buffer[j];
                comm.data_num     = 0;
                comm.checksum += buffer[j];
                comm.recv_buf[comm.packet_num++] = buffer[j];
                comm.step++;
                break;

            case 4:  // Data bytes
                comm.recv_buf[comm.packet_num++] = buffer[j];
                comm.checksum += buffer[j];
                comm.data_num++;

                if (comm.data_num >= MAX_DATA_SIZE)
                {
                    comm.step         = 0;
                    comm.packet_num   = 0;
                    comm.checksum     = 0;
                    comm.header_count = 0;
                    break;
                }

                if (comm.data_num >= comm.max_data_num)
                {
                    comm.step++;
                }
                break;

            case 5:  // Checksum verification
                comm.checksum += buffer[j];
                comm.recv_buf[comm.packet_num++] = buffer[j];

                if (comm.checksum == 0)
                {
                    comm.packet_ok = true;
                }

                comm.step         = 0;
                comm.packet_num   = 0;
                comm.checksum     = 0;
                comm.header_count = 0;
                break;

            default:
                comm.step         = 0;
                comm.packet_num   = 0;
                comm.checksum     = 0;
                comm.header_count = 0;
                break;
        }

        // Process complete packet
        if (comm.packet_ok)
        {
            comm.packet_ok = false;
            processReceivedData(comm, motor);
        }

        // Error recovery
        if (comm.error_count >= 10)
        {
            comm.error_count  = 0;
            comm.step         = 0;
            comm.packet_num   = 0;
            comm.checksum     = 0;
            comm.header_count = 0;
        }
    }

    return MD_SUCCESS;
}

// ============================================================================
// Unit conversions
// ============================================================================

int16_t radPerSecToRpm(double rad_per_sec, int gear_ratio)
{
    // wheel rad/s → wheel RPM → motor RPM (× gear_ratio)
    return static_cast<int16_t>(rad_per_sec * 60.0 / (2.0 * M_PI) * gear_ratio);
}

double rpmToRadPerSec(int16_t rpm, int gear_ratio)
{
    // motor RPM → wheel RPM (÷ gear_ratio) → wheel rad/s
    return rpm * 2.0 * M_PI / 60.0 / gear_ratio;
}

double computePPR(int poles, int gear_ratio)
{
    return poles * 3.0 * gear_ratio;
}

double computeTickToRad(int poles, int gear_ratio)
{
    return (2.0 * M_PI) / computePPR(poles, gear_ratio);
}

}  // namespace mobile_sdk
