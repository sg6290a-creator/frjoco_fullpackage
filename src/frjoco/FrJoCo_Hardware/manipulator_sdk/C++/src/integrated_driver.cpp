/**
 * ============================================================================
 * IntegratedDriver — Implementation (SocketCAN / CANable v2.0)
 * ============================================================================
 *
 * RMD + Robstride 모터를 1개의 SocketCAN 인터페이스에서 통합 제어.
 * CAN 프레임 구분: extended flag (false=RMD, true=Robstride)
 *
 * ============================================================================
 */

#include "manipulator_sdk/integrated_driver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace manipulator_sdk
{

namespace
{

constexpr int kAsyncReaderIdleSleepUs = 250;

MotorState parseRmdFeedbackFrame(const uint8_t rx_data[8], uint8_t rx_len,
                                 double torque_constant)
{
    if (rx_len < 8) {
        return {};
    }

    // 0x9C 와 0xA4 응답 둘 다 동일한 motor status 2 포맷을 사용.
    //   byte 6-7 = int16 multi-turn shaft angle (1°/LSB).
    if (rx_data[0] == RMD_CMD_READ_STATUS2 ||
        rx_data[0] == RMD_CMD_POSITION_CTRL2) {
        return RMDProtocol::parseStatus2(rx_data, rx_len, torque_constant);
    }

    return {};
}

}  // namespace

IntegratedDriver::IntegratedDriver() = default;

IntegratedDriver::~IntegratedDriver()
{
    cleanup();
}

void IntegratedDriver::setLogCallback(IntegratedLogCallback cb)
{
    log_cb_ = std::move(cb);
}

void IntegratedDriver::log(int level, const std::string& msg)
{
    if (log_cb_) {
        log_cb_(level, msg);
    } else {
        const char* prefix = (level == LOG_ERROR) ? "[ERROR]" :
                             (level == LOG_WARN)  ? "[WARN]"  : "[INFO]";
        fprintf(stderr, "[IntegratedDriver] %s %s\n", prefix, msg.c_str());
    }
}

void IntegratedDriver::startAsyncFeedback()
{
    if (async_reader_.isRunning()) {
        return;
    }

    async_reader_.setFrameHandler(
        [this](uint32_t rx_id, const uint8_t* rx_data, uint8_t rx_len, bool ext) {
            handleAsyncFrame(rx_id, rx_data, rx_len, ext);
        });

    if (!async_reader_.start(&can_, &io_mutex_, kAsyncReaderIdleSleepUs)) {
        log(LOG_WARN, "Failed to start async CAN reader; falling back to stale cached reads");
    }
}

void IntegratedDriver::stopAsyncFeedback()
{
    async_reader_.stop();
}

void IntegratedDriver::markJointFeedback(UnifiedJoint& joint)
{
    joint.online = true;
    joint.feedback_valid = true;
    joint.last_feedback_time = std::chrono::steady_clock::now();
}

bool IntegratedDriver::isJointFeedbackFresh(
    const UnifiedJoint& joint,
    const std::chrono::steady_clock::time_point& now) const
{
    if (!joint.feedback_valid) {
        return false;
    }

    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - joint.last_feedback_time).count();
    const int max_age_ms = std::max(20, config_.read_deadline_ms * 4);
    return age_ms <= max_age_ms;
}

bool IntegratedDriver::shouldPollJoint(
    const UnifiedJoint& joint,
    const std::chrono::steady_clock::time_point& now) const
{
    if (!joint.feedback_valid) {
        return true;
    }

    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - joint.last_feedback_time).count();
    const int poll_age_ms = std::max(2, config_.read_deadline_ms);
    return age_ms >= poll_age_ms;
}

void IntegratedDriver::handleAsyncFrame(
    uint32_t rx_id, const uint8_t* rx_data, uint8_t rx_len, bool ext)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!ext) {
        for (auto& j : joints_) {
            if (j.motor_type != MotorType::RMD) continue;
            if (rx_id != RMDProtocol::rxId(j.motor_id)) continue;

            MotorState s = parseRmdFeedbackFrame(rx_data, rx_len, j.torque_constant);
            if (!s.valid) break;

            const double dir = static_cast<double>(j.direction);
            j.position_rad = s.position_rad * dir;
            j.velocity_rads = s.velocity_rads * dir;
            j.effort_nm = s.effort_nm * dir;
            j.temperature = s.temperature;
            markJointFeedback(j);
            break;
        }
        return;
    }

    if (RobstrideProtocol::getMsgType(rx_id) != RS_MSG_FEEDBACK) {
        return;
    }

    // Robstride feedback arb_id: [msg_type:5][mode:2][err:6][motor_id:8][host_id:8]
    // motor_id는 bit8~15
    const uint8_t fb_motor_id = static_cast<uint8_t>((rx_id >> 8) & 0xFF);
    for (auto& j : joints_) {
        if (j.motor_type != MotorType::ROBSTRIDE) continue;
        if (static_cast<uint8_t>(j.motor_id) != fb_motor_id) continue;

        RobstrideState s = RobstrideProtocol::parseFeedback(
            rx_id, rx_data,
            j.rs_torque_min, j.rs_torque_max,
            j.rs_vel_min,    j.rs_vel_max);
        if (!s.valid) break;

        const double dir = static_cast<double>(j.direction);
        j.position_rad = static_cast<double>(s.position_rad) * dir;
        j.velocity_rads = static_cast<double>(s.velocity_rads) * dir;
        j.effort_nm = static_cast<double>(s.torque_nm) * dir;
        j.temperature = static_cast<double>(s.temperature);
        j.rs_mode = s.mode;
        j.rs_error_bits = s.error_bits;

        // 모터 상태 변화 감지 로그 (mode: 0=Reset 1=Cal 2=Run, err=0 정상)
        if (s.error_bits != 0 || s.mode != 2) {
            char buf[128];
            snprintf(buf, sizeof(buf),
                "[WARN] RS '%s' (ID=%d): mode=%d err=0x%02X pos=%.3f",
                j.name.c_str(), j.motor_id, s.mode, s.error_bits, s.position_rad);
            log(LOG_WARN, buf);
        }

        markJointFeedback(j);
        break;
    }
}

// ════════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════════

bool IntegratedDriver::configure(const IntegratedDriverConfig& config)
{
    stopAsyncFeedback();
    config_ = config;

    // ── Step 1: Open SocketCAN ─────────────────────────────
    log(LOG_INFO, "[1/3] Opening SocketCAN interface: " + config.can_if + "...");
    if (!can_.open(config.can_if)) {
        log(LOG_ERROR, "Failed to open SocketCAN interface: " + config.can_if);
        return false;
    }
    // configure/activate 중 파라미터 읽기/쓰기 응답이 필요하므로 필터 해제.
    // activate() 완료 후 setFilterOperational()로 comm_type=2 only 필터 적용.
    can_.setFilterPassAll();
    log(LOG_INFO, "  [OK] " + config.can_if + " open");

    // ── Step 2: Initialize joints ──────────────────────────
    joints_.resize(config.joints.size());
    for (size_t i = 0; i < config.joints.size(); ++i) {
        const auto& def = config.joints[i];
        auto& j = joints_[i];
        j.name                = def.name;
        j.motor_type          = def.motor_type;
        j.motor_id            = def.motor_id;
        j.required            = def.required;
        j.online              = false;
        j.torque_constant     = def.torque_constant;
        j.robstride_max_speed = def.max_speed_rads;
        j.direction           = (def.direction >= 0) ? 1 : -1;
        j.feedback_valid      = false;
    }

    clearReceiveBuffer();

    int rmd_count = 0, rs_count = 0;
    for (const auto& j : joints_) {
        if (j.motor_type == MotorType::RMD) rmd_count++;
        else rs_count++;
    }

    // ── Step 3: Test communication ─────────────────────────
    log(LOG_INFO, "[2/3] Testing " + std::to_string(joints_.size()) +
        " motors (" + std::to_string(rmd_count) + " RMD + " +
        std::to_string(rs_count) + " Robstride)...");

    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        bool ok = false;

        if (j.motor_type == MotorType::RMD) {
            ok = readRmdMotor(i);
        } else {
            ok = enableRobstride(i);
            if (ok) {
                j.enabled = true;
                disableRobstride(i);
                j.enabled = false;
            }
        }

        if (ok) {
            j.online = true;
            char buf[128];
            snprintf(buf, sizeof(buf),
                     "  [OK] '%s' (%s, ID=%d): pos=%.2f°",
                     j.name.c_str(), motorTypeString(j.motor_type),
                     j.motor_id, j.position_rad * 180.0 / M_PI);
            log(LOG_INFO, buf);
        } else if (j.required) {
            log(LOG_ERROR, "  [FAIL] '" + j.name + "' (" +
                motorTypeString(j.motor_type) + ", ID=" +
                std::to_string(j.motor_id) + ") — no response");
            can_.close();
            return false;
        } else {
            log(LOG_WARN, "  [SKIP] optional '" + j.name + "' (" +
                motorTypeString(j.motor_type) + ", ID=" +
                std::to_string(j.motor_id) + ") — no response");
            j.online = false;
            j.enabled = false;
            j.feedback_valid = false;
        }
    }

    log(LOG_INFO, "[3/3] Configuration complete: " +
        std::to_string(joints_.size()) + " motors on " + config.can_if);
    return true;
}

bool IntegratedDriver::activate()
{
    log(LOG_INFO, "Activating motors...");

    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        if (!j.online) {
            if (!j.required) {
                log(LOG_WARN, "  optional joint '" + j.name + "' remains offline; skipping activate");
                continue;
            }
            log(LOG_WARN, "  required joint '" + j.name + "' is offline during activate");
            continue;
        }

        if (j.motor_type == MotorType::RMD)
        {
            // src2 검증된 시퀀스: motor on/off 사이클 없음, 0x60 폴링 없음.
            // 0x9C 한 번 읽고 그 multi-turn 값을 target 으로 셋팅. 0xA4 가 첫 closed-loop
            // 명령이 되어 모터가 그 target 으로 hold.
            if (readRmdMotor(i)) {
                j.position_command = j.position_rad;
                writeRmdMotor(i);
                j.enabled = true;
                setRmdAcceleration(i, config_.rmd_acceleration);
                char buf[128];
                snprintf(buf, sizeof(buf), "  RMD '%s' (ID=%d): %.2f° — enabled, ready",
                         j.name.c_str(), j.motor_id, j.position_rad * 180.0 / M_PI);
                log(LOG_INFO, buf);
            } else {
                log(LOG_WARN, "  RMD '" + j.name + "' read failed during activate");
            }
        }
        else  // ROBSTRIDE
        {
            disableRobstride(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            writeRobstrideParam(i, RS_PARAM_RUN_MODE, RS_MODE_POSITION_CSP);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            writeRobstrideParam(i, RS_PARAM_LIMIT_SPD, j.robstride_max_speed);
            // 0이면 모터 기본값 유지
            if (j.rs_loc_kp > 0.0f) writeRobstrideParam(i, RS_PARAM_LOC_KP, j.rs_loc_kp);
            if (j.rs_spd_kp > 0.0f) writeRobstrideParam(i, RS_PARAM_SPD_KP, j.rs_spd_kp);
            if (j.rs_spd_ki > 0.0f) writeRobstrideParam(i, RS_PARAM_SPD_KI, j.rs_spd_ki);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            if (enableRobstride(i)) {
                j.enabled = true;
                j.position_command = j.position_rad;  // 현재 위치 유지 → 초반 점프 방지
                char buf[128];
                snprintf(buf, sizeof(buf),
                         "  Robstride '%s' (ID=%d): %.2f° — CSP mode, ready",
                         j.name.c_str(), j.motor_id, j.position_rad * 180.0 / M_PI);
                log(LOG_INFO, buf);
            } else {
                log(LOG_WARN, "  Robstride '" + j.name + "' enable failed");
            }
        }
    }

    log(LOG_INFO, "Activation complete");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    clearReceiveBuffer();
    // EDULITE_A3 방식: comm_type=2 (RS_MSG_FEEDBACK)만 수신하도록 커널 필터 적용.
    // LOC_REF write 응답(0x15) 등이 버퍼에 쌓이지 않아 async reader가 깨끗하게 동작.
    can_.setFilterOperational();
    startAsyncFeedback();
    return true;
}

void IntegratedDriver::deactivate()
{
    stopAsyncFeedback();
    log(LOG_INFO, "Deactivating motors...");

    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        if (!j.online) {
            j.enabled = false;
            continue;
        }
        if (j.motor_type == MotorType::ROBSTRIDE && j.enabled) {
            disableRobstride(i);
            j.enabled = false;
            log(LOG_INFO, "  Robstride '" + j.name + "' disabled");
        }
        j.enabled = false;
    }

    log(LOG_INFO, "Deactivation complete");
}

void IntegratedDriver::cleanup()
{
    stopAsyncFeedback();
    deactivate();
    can_.close();
}

// ════════════════════════════════════════════════════════════════
//  Communication — Read
// ════════════════════════════════════════════════════════════════

bool IntegratedDriver::readAll()
{
    struct PollRequest {
        uint32_t id = 0;
        uint8_t data[8] = {0};
        uint8_t len = 8;
        bool extended = false;
    };

    std::vector<PollRequest> polls;
    const auto now = std::chrono::steady_clock::now();
    bool all_fresh = true;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        polls.reserve(joints_.size());

        for (const auto& j : joints_) {
            if (!j.online && !j.required) continue;
            if (!isJointFeedbackFresh(j, now)) all_fresh = false;

            if (j.online && shouldPollJoint(j, now)) {
                if (j.motor_type == MotorType::RMD) {
                    PollRequest req;
                    RMDProtocol::buildReadStatus2(req.data);
                    req.id = RMDProtocol::txId(j.motor_id);
                    req.extended = false;
                    polls.push_back(req);
                }
                // Robstride: writeRobstrideMotor()에서 Enable을 매 사이클 전송하므로
                // 커널 필터(comm_type=2)를 통해 async reader가 피드백 수신.
                // readAll()에서 별도 Enable 불필요.
            }
        }
    }

    if (all_fresh && polls.empty()) return true;

    for (const auto& poll : polls) {
        std::lock_guard<std::mutex> io_lock(io_mutex_);
        if (!can_.sendFrame(poll.id, poll.data, poll.len, poll.extended)) return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& j : joints_) {
        if (!j.online && !j.required) continue;
        if (j.required && !j.feedback_valid) return false;
    }
    return true;
}

bool IntegratedDriver::readMotor(size_t index)
{
    if (index >= joints_.size()) return false;
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

    if (joints_[index].motor_type == MotorType::RMD)
        return readRmdMotor(index);
    else
        return readRobstrideMotor(index);
}

// ════════════════════════════════════════════════════════════════
//  Communication — Write
// ════════════════════════════════════════════════════════════════

void IntegratedDriver::writeAll()
{
    for (size_t i = 0; i < joints_.size(); ++i) {
        writeMotor(i);
    }

    // CSP mode: 10사이클(50ms)마다 Enable을 한 번 전송 → async reader가 피드백 수신.
    // writeRobstrideMotor()와 같은 사이클에 보내지 않아 진동 없음.
    static uint32_t enable_counter = 0;
    if (++enable_counter % 10 == 0) {
        for (size_t i = 0; i < joints_.size(); ++i) {
            auto& j = joints_[i];
            if (j.motor_type != MotorType::ROBSTRIDE || !j.online || !j.enabled) continue;
            uint32_t en_id;
            uint8_t  en_data[8];
            RobstrideProtocol::buildEnable(en_id, en_data, static_cast<uint8_t>(j.motor_id));
            std::lock_guard<std::mutex> io_lock(io_mutex_);
            can_.sendFrame(en_id, en_data, 8, true);
        }
    }
}

bool IntegratedDriver::writeMotor(size_t index)
{
    if (index >= joints_.size()) return false;
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

    if (joints_[index].motor_type == MotorType::RMD)
        return writeRmdMotor(index);
    else
        return writeRobstrideMotor(index);
}

// ════════════════════════════════════════════════════════════════
//  Internal — RMD
// ════════════════════════════════════════════════════════════════

bool IntegratedDriver::readRmdMotor(size_t index)
{
    auto& j = joints_[index];
    uint8_t cmd[8];
    RMDProtocol::buildReadStatus2(cmd);
    uint32_t tx_id = RMDProtocol::txId(j.motor_id);
    uint32_t expected_rx = RMDProtocol::rxId(j.motor_id);

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (!can_.sendFrame(tx_id, cmd, 8, false))  // standard CAN
        return false;

    for (int attempt = 0; attempt < 10; ++attempt) {
        bool ext = false;
        if (can_.receiveFrame(rx_id, rx_data, rx_len, 1, &ext)) {
            if (!ext && rx_id == expected_rx) {
                MotorState state = RMDProtocol::parseStatus2(rx_data, rx_len, j.torque_constant);
                if (state.valid) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    const double dir = static_cast<double>(j.direction);
                    j.position_rad  = state.position_rad * dir;
                    j.velocity_rads = state.velocity_rads * dir;
                    j.effort_nm     = state.effort_nm * dir;
                    j.temperature   = state.temperature;
                    markJointFeedback(j);
                    return true;
                }
            }
        }
    }
    return false;
}

bool IntegratedDriver::readRmdMultiTurn(size_t index)
{
    auto& j = joints_[index];
    uint8_t cmd[8];
    RMDProtocol::buildReadMultiTurn(cmd);
    uint32_t tx_id = RMDProtocol::txId(j.motor_id);
    uint32_t expected_rx = RMDProtocol::rxId(j.motor_id);

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (!can_.sendFrame(tx_id, cmd, 8, false))
        return false;

    for (int attempt = 0; attempt < 10; ++attempt) {
        bool ext = false;
        if (can_.receiveFrame(rx_id, rx_data, rx_len, 2, &ext)) {
            if (!ext && rx_id == expected_rx && rx_data[0] == RMD_CMD_READ_MULTI_TURN) {
                MotorState state = RMDProtocol::parseMultiTurn(rx_data, rx_len);
                if (state.valid) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    j.position_rad = state.position_rad;
                    markJointFeedback(j);
                    return true;
                }
            }
        }
    }
    return false;
}

bool IntegratedDriver::writeRmdMotor(size_t index)
{
    int motor_id = 0;
    uint8_t cmd[8];
    MotorCommand mc;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& j = joints_[index];
        motor_id = j.motor_id;
        const double dir = static_cast<double>(j.direction);
        mc.position_rad = j.position_command * dir;
        mc.velocity_rads = j.velocity_command * dir;
        mc.default_vel_dps = config_.rmd_default_vel_dps;
        mc.max_vel_dps = config_.rmd_max_vel_dps;
    }

    RMDProtocol::buildPositionCtrl2(cmd, mc);
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    return can_.sendFrame(RMDProtocol::txId(motor_id), cmd, 8, false);
}

bool IntegratedDriver::setRmdAcceleration(size_t index, uint32_t accel_dps2)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::RMD)
        return false;
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

    auto& j = joints_[index];
    uint8_t cmd[8];
    RMDProtocol::buildSetAcceleration(cmd, accel_dps2);

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (!can_.sendFrame(RMDProtocol::txId(j.motor_id), cmd, 8, false))
        return false;

    // Consume ACK to keep receive buffer clean for subsequent reads
    uint32_t rx_id; uint8_t rx_data[8], rx_len; bool ext = false;
    for (int attempt = 0; attempt < 10; ++attempt) {
        if (can_.receiveFrame(rx_id, rx_data, rx_len, 10, &ext)) {
            if (!ext && rx_id == RMDProtocol::rxId(j.motor_id)) break;
        } else {
            break;
        }
    }
    return true;
}

// ════════════════════════════════════════════════════════════════
//  Internal — Robstride
// ════════════════════════════════════════════════════════════════

bool IntegratedDriver::rsSendAndRecv(uint32_t tx_id, const uint8_t* tx_data,
                                     uint32_t* rx_id, uint8_t* rx_data,
                                     uint8_t* rx_len, int timeout_ms,
                                     uint8_t expected_msg_type)
{
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (!can_.sendFrame(tx_id, tx_data, 8, true))  // extended CAN
        return false;

    uint32_t id;
    uint8_t data[8], len;

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        bool ext = false;
        if (can_.receiveFrame(id, data, len, 2, &ext)) {
            if (!ext) continue;

            uint8_t resp_motor = static_cast<uint8_t>((id >> 8) & 0xFF);
            uint8_t sent_motor = static_cast<uint8_t>(tx_id & 0xFF);
            if (resp_motor != sent_motor) continue;

            // expected_msg_type=0xFF는 any 허용
            if (expected_msg_type != 0xFF) {
                uint8_t msg_type = RobstrideProtocol::getMsgType(id);
                if (msg_type != expected_msg_type) continue;
            }

            if (rx_id)   *rx_id = id;
            if (rx_data) std::memcpy(rx_data, data, 8);
            if (rx_len)  *rx_len = len;
            return true;
        }
    }
    return false;
}

bool IntegratedDriver::readRobstrideMotor(size_t index)
{
    auto& j = joints_[index];

    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(j.motor_id));

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    if (!rsSendAndRecv(arb_id, data, &rx_id, rx_data, &rx_len))
        return false;

    RobstrideState state = RobstrideProtocol::parseFeedback(
        rx_id, rx_data,
        j.rs_torque_min, j.rs_torque_max,
        j.rs_vel_min,    j.rs_vel_max);
    if (state.valid) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double dir = static_cast<double>(j.direction);
        j.position_rad  = static_cast<double>(state.position_rad) * dir;
        j.velocity_rads = static_cast<double>(state.velocity_rads) * dir;
        j.effort_nm     = static_cast<double>(state.torque_nm) * dir;
        j.temperature   = static_cast<double>(state.temperature);
        j.rs_mode       = state.mode;
        j.rs_error_bits = state.error_bits;
        markJointFeedback(j);
        return true;
    }
    return false;
}

bool IntegratedDriver::writeRobstrideMotor(size_t index)
{
    int motor_id = 0;
    float target = 0.0f;
    bool enabled = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto& j = joints_[index];
        motor_id = j.motor_id;
        const double dir = static_cast<double>(j.direction);
        target = static_cast<float>(j.position_command * dir);
        enabled = j.enabled;
    }

    if (!enabled) return false;

    // CSP mode: LOC_REF만 200Hz로 전송.
    // 커널 필터(comm_type=2 only)로 write 응답 차단,
    // async reader는 Enable 응답(RS_MSG_FEEDBACK)만 수신.
    uint32_t ref_id;
    uint8_t  ref_data[8];
    RobstrideProtocol::buildWriteParam(
        ref_id, ref_data, static_cast<uint8_t>(motor_id), RS_PARAM_LOC_REF, target);

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    return can_.sendFrame(ref_id, ref_data, 8, true);
}

bool IntegratedDriver::enableRobstride(size_t index)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;
    if (!joints_[index].online && !joints_[index].required) {
        return true;
    }

    auto& j = joints_[index];
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildEnable(arb_id, data, static_cast<uint8_t>(j.motor_id));

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;

    if (!rsSendAndRecv(arb_id, data, &rx_id, rx_data, &rx_len))
        return false;

    RobstrideState state = RobstrideProtocol::parseFeedback(
        rx_id, rx_data,
        j.rs_torque_min, j.rs_torque_max,
        j.rs_vel_min,    j.rs_vel_max);
    if (state.valid) {
        std::lock_guard<std::mutex> lock(mutex_);
        const double dir = static_cast<double>(j.direction);
        j.position_rad  = static_cast<double>(state.position_rad) * dir;
        j.velocity_rads = static_cast<double>(state.velocity_rads) * dir;
        j.effort_nm     = static_cast<double>(state.torque_nm) * dir;
        j.temperature   = static_cast<double>(state.temperature);
        j.rs_mode       = state.mode;
        j.rs_error_bits = state.error_bits;
        j.enabled       = true;
        markJointFeedback(j);
    }
    return state.valid;
}

bool IntegratedDriver::disableRobstride(size_t index)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

    int motor_id = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        motor_id = joints_[index].motor_id;
    }
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildDisable(arb_id, data, static_cast<uint8_t>(motor_id));

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    bool ok = can_.sendFrame(arb_id, data, 8, true);
    if (ok) {
        std::lock_guard<std::mutex> lock(mutex_);
        joints_[index].enabled = false;
    }
    return ok;
}

bool IntegratedDriver::setRobstrideMode(size_t index, uint8_t mode)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

    disableRobstride(index);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    writeRobstrideParam(index, RS_PARAM_RUN_MODE, static_cast<float>(mode));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    return enableRobstride(index);
}

bool IntegratedDriver::readRobstrideParam(size_t index, uint16_t param_id, float& value)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

    auto& j = joints_[index];
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildReadParam(arb_id, data, static_cast<uint8_t>(j.motor_id), param_id);

    std::lock_guard<std::mutex> io_lock(io_mutex_);
    if (!can_.sendFrame(arb_id, data, 8, true))
        return false;

    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);

    while (std::chrono::steady_clock::now() < deadline) {
        uint32_t rx_id;
        uint8_t rx_data[8], rx_len;
        bool ext = false;

        if (can_.receiveFrame(rx_id, rx_data, rx_len, 5, &ext)) {
            if (!ext) continue;

            uint8_t msg_type = RobstrideProtocol::getMsgType(rx_id);
            if (msg_type == RS_MSG_FEEDBACK) continue;

            if (RobstrideProtocol::parseParamResponse(rx_data, param_id, value))
                return true;
        }
    }
    return false;
}

bool IntegratedDriver::writeRobstrideParam(size_t index, uint16_t param_id, float value)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;
    if (!joints_[index].online) {
        return !joints_[index].required;
    }

    auto& j = joints_[index];
    uint32_t arb_id;
    uint8_t data[8];
    RobstrideProtocol::buildWriteParam(arb_id, data, static_cast<uint8_t>(j.motor_id),
                                       param_id, value);

    uint32_t rx_id;
    uint8_t rx_data[8], rx_len;
    return rsSendAndRecv(arb_id, data, &rx_id, rx_data, &rx_len);
}

// ════════════════════════════════════════════════════════════════
//  Utility
// ════════════════════════════════════════════════════════════════

void IntegratedDriver::clearReceiveBuffer()
{
    uint32_t rx_id;
    uint8_t data[8], len;
    int cleared = 0;
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    while (can_.receiveFrame(rx_id, data, len, 1) && cleared < 200) {
        cleared++;
    }
}

}  // namespace manipulator_sdk
