/**
 * ============================================================================
 * MD Motor Protocol — Constants, structs, packet build/parse (no ROS dependency)
 * ============================================================================
 *
 * MD200T 듀얼 채널 모터 드라이버 RS-485 프로토콜.
 * 패킷 구조: [RMID][TMID][ID][PID][DataSize][D0..DN][CHK]
 *
 * ============================================================================
 */

#ifndef MOBILE_SDK__MD_PROTOCOL_HPP_
#define MOBILE_SDK__MD_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace mobile_sdk
{

// ============================================================================
// Constants
// ============================================================================

constexpr int MD_SUCCESS = 1;
constexpr int MD_FAIL    = 0;

constexpr size_t NUM_WHEELS = 4;

// Wheel indices
constexpr size_t FRONT_LEFT  = 0;
constexpr size_t FRONT_RIGHT = 1;
constexpr size_t REAR_LEFT   = 2;
constexpr size_t REAR_RIGHT  = 3;

// Protocol IDs
constexpr uint8_t PID_MAIN_DATA     = 193;   // 0xC1 — single-channel feedback
constexpr uint8_t PID_MAIN_DATA_ALT = 210;   // 0xD2 — dual-channel feedback
constexpr uint8_t PID_PNT_VEL_CMD   = 207;   // 0xCF — velocity command

// Packet limits
constexpr size_t MAX_PACKET_SIZE = 26;
constexpr size_t MAX_DATA_SIZE   = 23;

// Default IDs: MDUI=184 (PC), MDT=183 (driver)
constexpr uint8_t DEFAULT_ID_MDUI = 184;
constexpr uint8_t DEFAULT_ID_MDT  = 183;

// Data request type embedded in velocity command
constexpr uint8_t REQUEST_PNT_MAIN_DATA = 2;

// ============================================================================
// Structs
// ============================================================================

/// 2-byte little-endian helper
struct IByte {
    uint8_t low;
    uint8_t high;
};

/// Per-driver motor state (dual channel: left = CH1, right = CH2)
struct MotorState {
    int16_t  left_rpm;
    int16_t  right_rpm;
    int32_t  left_position;
    int32_t  right_position;
    double   left_last_rad;
    double   right_last_rad;
    int32_t  left_last_tick;
    int32_t  right_last_tick;
    bool     init_motor;
};

/// Per-driver serial receive state machine
struct CommState {
    uint8_t recv_buf[MAX_PACKET_SIZE];
    uint8_t step;
    uint8_t packet_num;
    uint8_t checksum;
    uint8_t max_data_num;
    uint8_t data_num;
    bool    packet_ok;
    uint8_t error_count;
    int     header_count;   // internal: RMID/TMID counter
};

/// Driver configuration (passed at construction time)
struct MDDriverConfig {
    std::string port_front  = "/dev/ttyUSB0";
    std::string port_rear   = "/dev/ttyUSB1";
    int    baudrate         = 57600;
    int    front_motor_id   = 1;
    int    rear_motor_id    = 2;
    int    id_mdui          = DEFAULT_ID_MDUI;   // 184
    int    id_mdt           = DEFAULT_ID_MDT;    // 183
    double wheel_radius     = 0.098;
    double wheel_separation = 0.32;
    int    gear_ratio       = 15;
    int    poles             = 10;
};

// ============================================================================
// Free functions — protocol helpers
// ============================================================================

/// Initialize MotorState to zero
void initMotorState(MotorState& motor);

/// Initialize CommState to zero
void initCommState(CommState& comm);

/// int16 → little-endian 2 bytes
IByte short2Byte(int16_t value);

/// little-endian 2 bytes → int16
int16_t byte2Short(uint8_t low, uint8_t high);

/// little-endian 4 bytes → int32
int32_t byte2Long(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4);

/**
 * Build a 13-byte velocity command packet.
 * @param[out] packet  Must point to at least 13 bytes.
 * @param id_mdt       Receiver ID (default 183).
 * @param id_mdui      Sender ID (default 184).
 * @param motor_id     Motor driver ID (1 or 2).
 * @param left_rpm     CH1 RPM command.
 * @param right_rpm    CH2 RPM command.
 * @return Packet length (always 13).
 */
size_t buildVelocityPacket(uint8_t* packet,
                           uint8_t id_mdt, uint8_t id_mdui, uint8_t motor_id,
                           int16_t left_rpm, int16_t right_rpm);

/**
 * Feed received bytes into the state machine.
 * When a valid packet is found, processReceivedData() is called internally.
 *
 * @param buffer   Raw bytes from serial port.
 * @param size     Number of bytes.
 * @param comm     Communication state (preserved across calls).
 * @param motor    Motor state — updated when a complete packet arrives.
 * @param id_mdt   Expected RMID (183).
 * @param id_mdui  Expected TMID (184).
 * @return MD_SUCCESS
 */
int analyzeReceivedData(const uint8_t* buffer, size_t size,
                        CommState& comm, MotorState& motor,
                        uint8_t id_mdt, uint8_t id_mdui);

/**
 * Extract RPM / position from a validated packet stored in comm.recv_buf.
 * Called automatically by analyzeReceivedData when checksum passes.
 */
void processReceivedData(const CommState& comm, MotorState& motor);

// ============================================================================
// Unit conversions
// ============================================================================

/// rad/s → motor RPM (with gear ratio)
int16_t radPerSecToRpm(double rad_per_sec, int gear_ratio);

/// motor RPM → rad/s (with gear ratio)
double rpmToRadPerSec(int16_t rpm, int gear_ratio);

/// Compute encoder PPR from poles and gear_ratio: poles * 3 * gear_ratio
double computePPR(int poles, int gear_ratio);

/// Compute tick_to_rad: 2π / PPR
double computeTickToRad(int poles, int gear_ratio);

}  // namespace mobile_sdk

#endif  // MOBILE_SDK__MD_PROTOCOL_HPP_
