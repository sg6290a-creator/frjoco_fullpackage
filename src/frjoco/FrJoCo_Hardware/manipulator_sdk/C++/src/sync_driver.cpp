/**
 * ============================================================================
 * SyncDriver — Implementation
 * ============================================================================
 */

#include "manipulator_sdk/sync_driver.hpp"
#include "manipulator_sdk/frlab_manipulator.hpp"  // RS03_DEFAULT_SPEED_LIMIT

#include <cstring>
#include <cstdio>
#include <chrono>
#include <string>
#include <thread>

namespace manipulator_sdk
{
namespace
{
constexpr const char* RESET = "\033[0m";
constexpr const char* GREEN = "\033[92m";
constexpr const char* RED = "\033[91m";

std::string jointDisplayName(const std::string& name)
{
    if (name == "joint_1") return "J1 shoulder_pan_joint";
    if (name == "joint_2") return "J2 shoulder_lift_joint";
    if (name == "joint_3") return "J3 elbow_joint";
    if (name == "joint_4") return "J4 wrist_1_joint";
    if (name == "joint_5") return "J5 wrist_2_joint";
    if (name == "joint_6") return "J6 wrist_3_joint";
    return name;
}

std::string oxMark(bool ok)
{
    return std::string(ok ? GREEN : RED) + (ok ? "O" : "X") + RESET;
}

std::string motorCheckLine(const char* motor_model, const UnifiedJoint& joint, bool ok)
{
    char buf[256];
    snprintf(
        buf,
        sizeof(buf),
        "  %s %-22s | %s '%s' (ID=%d): %s",
        oxMark(ok).c_str(),
        jointDisplayName(joint.name).c_str(),
        motor_model,
        joint.name.c_str(),
        joint.motor_id,
        ok ? "OK" : "NO RESPONSE");
    return std::string(buf);
}
}  // namespace

SyncDriver::SyncDriver() = default;

SyncDriver::~SyncDriver()
{
    cleanup();
}

void SyncDriver::setLogCallback(SyncLogCallback cb)
{
    log_cb_ = std::move(cb);
}

void SyncDriver::log(int level, const std::string& msg)
{
    if (log_cb_) {
        log_cb_(level, msg);
    } else {
        const char* p = (level == LOG_ERROR) ? "[ERROR]" :
                        (level == LOG_WARN)  ? "[WARN]"  : "[INFO]";
        fprintf(stderr, "[SyncDriver] %s %s\n", p, msg.c_str());
    }
}

// ════════════════════════════════════════════════════════════════
//  Helpers
// ════════════════════════════════════════════════════════════════

void SyncDriver::clearBuffer()
{
    uint32_t id; uint8_t data[8], len; bool ext = false;
    while (can_.receiveFrame(id, data, len, 0, &ext)) {}
}

bool SyncDriver::rsSendAndRecv(uint32_t tx_id, const uint8_t* tx_data,
                                uint32_t* out_rx_id, uint8_t* out_rx_data,
                                uint8_t* out_rx_len, int timeout_ms,
                                uint8_t expected_msg_type)
{
    if (!can_.sendFrame(tx_id, tx_data, 8, true))
        return false;

    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        uint32_t id; uint8_t data[8], len; bool ext = false;
        if (!can_.receiveFrame(id, data, len, 2, &ext)) continue;
        if (!ext) continue;

        (void)expected_msg_type;

        if (out_rx_id)   *out_rx_id  = id;
        if (out_rx_data) std::memcpy(out_rx_data, data, 8);
        if (out_rx_len)  *out_rx_len = len;
        return true;
    }
    return false;
}

// ════════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════════

bool SyncDriver::configure(const IntegratedDriverConfig& config)
{
    config_ = config;
    joints_.clear();

    log(LOG_INFO, "[1/3] Opening SocketCAN: " + config.can_if);
    if (!can_.open(config.can_if)) {
        log(LOG_ERROR, "Failed to open: " + config.can_if);
        return false;
    }

    // Build joint list
    for (const auto& jd : config.joints) {
        UnifiedJoint j;
        j.name       = jd.name;
        j.motor_type = jd.motor_type;
        j.motor_id   = jd.motor_id;
        j.required   = jd.required;
        j.direction  = jd.direction;

        if (jd.motor_type == MotorType::RMD) {
            j.torque_constant = jd.torque_constant;
        } else {
            j.robstride_max_speed = jd.max_speed_rads;
            // RS03 decode ranges
            j.rs_torque_min = RS03_TORQUE_MIN;
            j.rs_torque_max = RS03_TORQUE_MAX;
            j.rs_vel_min    = RS03_VEL_MIN;
            j.rs_vel_max    = RS03_VEL_MAX;
        }
        joints_.push_back(j);
    }

    log(LOG_INFO, "[2/3] Testing motors (J1~J6 CAN O/X)...");
    bool required_motor_failed = false;

    // Test RMD motors (0x9C ping)
    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        if (j.motor_type != MotorType::RMD) continue;

        uint8_t cmd[8]; RMDProtocol::buildReadStatus2(cmd);
        uint32_t tx_id = RMDProtocol::txId(j.motor_id);
        uint32_t exp_rx = RMDProtocol::rxId(j.motor_id);

        bool ok = false;
        if (can_.sendFrame(tx_id, cmd, 8, false)) {
            for (int t = 0; t < 10; ++t) {
                uint32_t rx_id; uint8_t rx_data[8], rx_len; bool ext = false;
                if (can_.receiveFrame(rx_id, rx_data, rx_len, 10, &ext)) {
                    if (!ext && rx_id == exp_rx) {
                        MotorState s = RMDProtocol::parseStatus2(rx_data, rx_len, j.torque_constant);
                        if (s.valid) { j.online = true; ok = true; break; }
                    }
                }
            }
        }

        if (ok) {
            log(LOG_INFO, motorCheckLine("RMD", j, true));
        } else {
            if (j.required) {
                log(LOG_ERROR, motorCheckLine("RMD", j, false));
                required_motor_failed = true;
            } else {
                log(LOG_WARN, motorCheckLine("RMD", j, false));
            }
        }
    }

    // Test Robstride motors (Enable → feedback ping)
    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        if (j.motor_type != MotorType::ROBSTRIDE) continue;

        uint32_t en_id; uint8_t en_data[8];
        RobstrideProtocol::buildEnable(en_id, en_data, static_cast<uint8_t>(j.motor_id));

        uint32_t rx_id; uint8_t rx_data[8], rx_len;
        bool ok = rsSendAndRecv(en_id, en_data, &rx_id, rx_data, &rx_len, 500, RS_MSG_FEEDBACK);

        if (ok) {
            j.online = true;
            log(LOG_INFO, motorCheckLine("RS03", j, true));
        } else {
            if (j.required) {
                log(LOG_ERROR, motorCheckLine("RS03", j, false));
                required_motor_failed = true;
            } else {
                log(LOG_WARN, motorCheckLine("RS03", j, false));
            }
        }
    }

    if (required_motor_failed) {
        log(LOG_ERROR, "[3/3] Configure failed: one or more required motors did not respond");
        can_.close();
        return false;
    }

    log(LOG_INFO, "[3/3] Configure complete");
    return true;
}

bool SyncDriver::activate()
{
    log(LOG_INFO, "Activating motors...");

    // RMD: set acceleration, read initial position
    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        if (j.motor_type != MotorType::RMD || !j.online) continue;

        // Set acceleration
        uint8_t accel_cmd[8];
        RMDProtocol::buildSetAcceleration(accel_cmd, config_.rmd_acceleration);
        can_.sendFrame(RMDProtocol::txId(j.motor_id), accel_cmd, 8, false);
        // Drain ACK
        uint32_t id; uint8_t data[8], len; bool ext = false;
        for (int t = 0; t < 5; ++t) {
            if (can_.receiveFrame(id, data, len, 5, &ext)) break;
        }

        // Read initial position (0x9C)
        uint8_t cmd[8]; RMDProtocol::buildReadStatus2(cmd);
        uint32_t exp_rx = RMDProtocol::rxId(j.motor_id);
        if (can_.sendFrame(RMDProtocol::txId(j.motor_id), cmd, 8, false)) {
            for (int t = 0; t < 10; ++t) {
                bool e = false;
                if (can_.receiveFrame(id, data, len, 10, &e)) {
                    if (!e && id == exp_rx) {
                        MotorState s = RMDProtocol::parseStatus2(data, len, j.torque_constant);
                        if (s.valid) {
                            const double dir = static_cast<double>(j.direction);
                            j.position_rad  = s.position_rad * dir;
                            j.velocity_rads = s.velocity_rads * dir;
                            j.effort_nm     = s.effort_nm * dir;
                            j.temperature   = s.temperature;
                            j.position_command = j.position_rad;
                            j.feedback_valid   = true;
                            j.enabled = true;

                            char buf[128];
                            snprintf(buf, sizeof(buf), "  RMD '%s' init pos=%.2f deg",
                                     j.name.c_str(), j.position_rad * 180.0 / M_PI);
                            log(LOG_INFO, buf);
                            break;
                        }
                    }
                }
            }
        }
    }

    // Robstride: set CSP mode (RUN_MODE=5), set speed limit, read initial position
    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        if (j.motor_type != MotorType::ROBSTRIDE || !j.online) continue;

        // Disable → Reset 상태 진입 후 파라미터 쓰기
        {
            uint32_t dis_id; uint8_t dis_data[8];
            RobstrideProtocol::buildDisable(dis_id, dis_data, static_cast<uint8_t>(j.motor_id));
            can_.sendFrame(dis_id, dis_data, 8, true);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Reset 안정화 대기
            clearBuffer();
        }

        // Write RUN_MODE=5 (CSP) — 실패 시 재시도
        {
            bool ok = false;
            for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
                uint32_t wp_id; uint8_t wp_data[8];
                RobstrideProtocol::buildWriteParam(wp_id, wp_data,
                    static_cast<uint8_t>(j.motor_id), RS_PARAM_RUN_MODE,
                    static_cast<float>(RS_MODE_POSITION_CSP));
                uint32_t rx_id; uint8_t rx_data[8], rx_len;
                ok = rsSendAndRecv(wp_id, wp_data, &rx_id, rx_data, &rx_len, 200, RS_MSG_WRITE_PARAM);
                if (!ok) std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            if (!ok) log(LOG_WARN, "  RS03 '" + j.name + "': RUN_MODE write no ack (3 attempts)");
            else {
                // Readback 확인
                uint32_t rp_id; uint8_t rp_data[8];
                RobstrideProtocol::buildReadParam(rp_id, rp_data,
                    static_cast<uint8_t>(j.motor_id), RS_PARAM_RUN_MODE);
                uint32_t rx_id; uint8_t rx_data[8], rx_len;
                if (rsSendAndRecv(rp_id, rp_data, &rx_id, rx_data, &rx_len, 200, RS_MSG_READ_PARAM)) {
                    float val = 0;
                    RobstrideProtocol::parseParamResponse(rx_data, RS_PARAM_RUN_MODE, val);
                    char buf[128];
                    snprintf(buf, sizeof(buf), "  RS03 '%s': RUN_MODE readback=%.0f (expect 5)",
                             j.name.c_str(), val);
                    log(LOG_INFO, buf);
                }
            }
        }

        // Write LIMIT_SPD
        {
            uint32_t wp_id; uint8_t wp_data[8];
            RobstrideProtocol::buildWriteParam(wp_id, wp_data,
                static_cast<uint8_t>(j.motor_id), RS_PARAM_LIMIT_SPD,
                j.robstride_max_speed);
            rsSendAndRecv(wp_id, wp_data, nullptr, nullptr, nullptr, 200, RS_MSG_WRITE_PARAM);
        }

        // Enable → get initial feedback
        {
            uint32_t en_id; uint8_t en_data[8];
            RobstrideProtocol::buildEnable(en_id, en_data, static_cast<uint8_t>(j.motor_id));
            uint32_t rx_id; uint8_t rx_data[8], rx_len;
            bool ok = rsSendAndRecv(en_id, en_data, &rx_id, rx_data, &rx_len, 500, RS_MSG_FEEDBACK);

            if (ok) {
                RobstrideState s = RobstrideProtocol::parseFeedback(
                    rx_id, rx_data, j.rs_torque_min, j.rs_torque_max, j.rs_vel_min, j.rs_vel_max);
                if (s.valid) {
                    const double dir = static_cast<double>(j.direction);
                    j.position_rad     = static_cast<double>(s.position_rad) * dir;
                    j.velocity_rads    = static_cast<double>(s.velocity_rads) * dir;
                    j.effort_nm        = static_cast<double>(s.torque_nm) * dir;
                    j.temperature      = static_cast<double>(s.temperature);
                    j.position_command = j.position_rad;
                    j.feedback_valid   = true;
                    j.enabled          = true;

                    char buf[128];
                    snprintf(buf, sizeof(buf), "  RS03 '%s' CSP OK, init pos=%.3f rad",
                             j.name.c_str(), j.position_rad);
                    log(LOG_INFO, buf);
                }
            } else {
                log(LOG_WARN, "  RS03 '" + j.name + "': Enable no feedback");
            }
        }
    }

    log(LOG_INFO, "Activation complete");
    return true;
}

void SyncDriver::deactivate()
{
    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        if (!j.online || !j.enabled) continue;

        if (j.motor_type == MotorType::ROBSTRIDE) {
            uint32_t dis_id; uint8_t dis_data[8];
            RobstrideProtocol::buildDisable(dis_id, dis_data,
                static_cast<uint8_t>(j.motor_id), 0);
            can_.sendFrame(dis_id, dis_data, 8, true);
            log(LOG_INFO, "  RS03 '" + j.name + "' disabled");
        }
        j.enabled = false;
    }
}

void SyncDriver::cleanup()
{
    deactivate();
    can_.close();
}

// ════════════════════════════════════════════════════════════════
//  Per-cycle step
// ════════════════════════════════════════════════════════════════

bool SyncDriver::stepRmd(size_t index)
{
    auto& j = joints_[index];
    if (!j.online) return !j.required;

    // 0xA4 위치 명령 전송 — 응답에 현재 상태가 담겨 옴
    MotorCommand mc;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        const double dir = static_cast<double>(j.direction);
        mc.position_rad    = j.position_command * dir;
        mc.velocity_rads   = j.velocity_command;
        mc.default_vel_dps = config_.rmd_default_vel_dps;
        mc.max_vel_dps     = config_.rmd_max_vel_dps;
    }

    uint8_t cmd[8];
    RMDProtocol::buildPositionCtrl2(cmd, mc);
    uint32_t tx_id  = RMDProtocol::txId(j.motor_id);
    uint32_t exp_rx = RMDProtocol::rxId(j.motor_id);

    if (!can_.sendFrame(tx_id, cmd, 8, false)) return false;

    for (int t = 0; t < 10; ++t) {
        uint32_t rx_id; uint8_t rx_data[8], rx_len; bool ext = false;
        if (can_.receiveFrame(rx_id, rx_data, rx_len, 2, &ext)) {
            if (!ext && rx_id == exp_rx) {
                MotorState s = RMDProtocol::parseStatus2(rx_data, rx_len, j.torque_constant);
                if (s.valid) {
                    std::lock_guard<std::mutex> lk(mutex_);
                    const double dir = static_cast<double>(j.direction);
                    j.position_rad   = s.position_rad * dir;
                    j.velocity_rads  = s.velocity_rads * dir;
                    j.effort_nm      = s.effort_nm * dir;
                    j.temperature    = s.temperature;
                    j.feedback_valid = true;
                    j.last_feedback_time = std::chrono::steady_clock::now();
                    return true;
                }
            }
        }
    }
    return false;
}

bool SyncDriver::stepRobstride(size_t index)
{
    auto& j = joints_[index];
    if (!j.online) return !j.required;

    // Step 1: LOC_REF 파라미터 쓰기
    {
        double pos_cmd;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            const double dir = static_cast<double>(j.direction);
            pos_cmd = j.position_command * dir;
        }

        uint32_t wp_id; uint8_t wp_data[8];
        RobstrideProtocol::buildWriteParam(wp_id, wp_data,
            static_cast<uint8_t>(j.motor_id), RS_PARAM_LOC_REF,
            static_cast<float>(pos_cmd));

        // LOC_REF write 응답(0x15)이 오면 드레인, 안 와도 무방
        uint32_t rx_id; uint8_t rx_data[8], rx_len;
        rsSendAndRecv(wp_id, wp_data, &rx_id, rx_data, &rx_len, 5, RS_MSG_WRITE_PARAM);
    }

    // Step 2: Enable 전송 → 피드백 응답 수신
    {
        uint32_t en_id; uint8_t en_data[8];
        RobstrideProtocol::buildEnable(en_id, en_data, static_cast<uint8_t>(j.motor_id));
        uint32_t rx_id; uint8_t rx_data[8], rx_len;
        bool ok = rsSendAndRecv(en_id, en_data, &rx_id, rx_data, &rx_len,
                                config_.read_deadline_ms * 2, RS_MSG_FEEDBACK);

        if (!ok) return !j.required;

        RobstrideState s = RobstrideProtocol::parseFeedback(
            rx_id, rx_data, j.rs_torque_min, j.rs_torque_max, j.rs_vel_min, j.rs_vel_max);
        if (!s.valid) return !j.required;

        std::lock_guard<std::mutex> lk(mutex_);
        const double dir = static_cast<double>(j.direction);
        j.position_rad   = static_cast<double>(s.position_rad) * dir;
        j.velocity_rads  = static_cast<double>(s.velocity_rads) * dir;
        j.effort_nm      = static_cast<double>(s.torque_nm) * dir;
        j.temperature    = static_cast<double>(s.temperature);
        j.rs_mode        = s.mode;
        j.rs_error_bits  = s.error_bits;
        j.feedback_valid = true;
        j.last_feedback_time = std::chrono::steady_clock::now();

        if (s.error_bits != 0 || s.mode != 2) {
            char buf[128];
            snprintf(buf, sizeof(buf), "[WARN] RS '%s' mode=%d err=0x%02X",
                     j.name.c_str(), s.mode, s.error_bits);
            log(LOG_WARN, buf);
        }
    }
    return true;
}

bool SyncDriver::writeRead()
{
    struct RmdPending { size_t index; uint32_t exp_rx; bool done = false; };
    struct RsPending  { size_t index; bool done = false; };

    std::vector<RmdPending> rmd_pending;
    std::vector<RsPending>  rs_pending;

    // ── Phase 1: RMD 0xA4 burst 전송 ──────────────────────────
    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        if (j.motor_type != MotorType::RMD || !j.online) continue;

        MotorCommand mc;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            const double dir = static_cast<double>(j.direction);
            mc.position_rad    = j.position_command * dir;
            mc.velocity_rads   = j.velocity_command;
            mc.default_vel_dps = config_.rmd_default_vel_dps;
            mc.max_vel_dps     = config_.rmd_max_vel_dps;
        }
        uint8_t cmd[8];
        RMDProtocol::buildPositionCtrl2(cmd, mc);
        can_.sendFrame(RMDProtocol::txId(j.motor_id), cmd, 8, false);
        rmd_pending.push_back({i, RMDProtocol::rxId(j.motor_id), false});
    }

    // ── Phase 2: RMD 응답 수집 (deadline 5ms) ──────────────────
    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(8);
        int remaining = static_cast<int>(rmd_pending.size());
        while (remaining > 0 && std::chrono::steady_clock::now() < deadline) {
            uint32_t rx_id; uint8_t rx_data[8], rx_len; bool ext = false;
            if (!can_.receiveFrame(rx_id, rx_data, rx_len, 1, &ext)) continue;
            if (ext) continue;  // extended = Robstride, 무시

            for (auto& p : rmd_pending) {
                if (p.done || rx_id != p.exp_rx) continue;
                auto& j = joints_[p.index];
                MotorState s = RMDProtocol::parseStatus2(rx_data, rx_len, j.torque_constant);
                if (!s.valid) break;

                std::lock_guard<std::mutex> lk(mutex_);
                const double dir = static_cast<double>(j.direction);
                j.position_rad   = s.position_rad * dir;
                j.velocity_rads  = s.velocity_rads * dir;
                j.effort_nm      = s.effort_nm * dir;
                j.temperature    = s.temperature;
                j.feedback_valid = true;
                j.last_feedback_time = std::chrono::steady_clock::now();
                p.done = true;
                remaining--;
                break;
            }
        }
        for (const auto& p : rmd_pending)
            if (!p.done && joints_[p.index].required) return false;
    }

    // ── Phase 3: RS03 LOC_REF/Enable burst 전송 후 feedback 수집 ──
    for (size_t i = 0; i < joints_.size(); ++i) {
        auto& j = joints_[i];
        if (j.motor_type != MotorType::ROBSTRIDE || !j.online) continue;

        const uint8_t mid = static_cast<uint8_t>(j.motor_id);

        // LOC_REF write (fire, drain 응답)
        {
            double pos_cmd;
            {
                std::lock_guard<std::mutex> lk(mutex_);
                const double dir = static_cast<double>(j.direction);
                pos_cmd = j.position_command * dir;
            }
            uint32_t wp_id; uint8_t wp_data[8];
            RobstrideProtocol::buildWriteParam(wp_id, wp_data, mid, RS_PARAM_LOC_REF,
                                               static_cast<float>(pos_cmd));
            can_.sendFrame(wp_id, wp_data, 8, true);
        }
        rs_pending.push_back({i, false});
    }

    for (const auto& p : rs_pending) {
        auto& j = joints_[p.index];
        uint32_t en_id; uint8_t en_data[8];
        RobstrideProtocol::buildEnable(
            en_id, en_data, static_cast<uint8_t>(j.motor_id));
        can_.sendFrame(en_id, en_data, 8, true);
    }

    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(8);
        int remaining = static_cast<int>(rs_pending.size());
        while (remaining > 0 && std::chrono::steady_clock::now() < deadline) {
            uint32_t rx_id; uint8_t rx_data[8], rx_len; bool ext = false;
            if (!can_.receiveFrame(rx_id, rx_data, rx_len, 1, &ext)) continue;
            if (!ext || RobstrideProtocol::getMsgType(rx_id) != RS_MSG_FEEDBACK) continue;

            for (auto& p : rs_pending) {
                if (p.done) continue;
                auto& j = joints_[p.index];
                RobstrideState s = RobstrideProtocol::parseFeedback(
                    rx_id, rx_data, j.rs_torque_min, j.rs_torque_max, j.rs_vel_min, j.rs_vel_max);
                if (!s.valid || s.motor_id != static_cast<uint8_t>(j.motor_id)) continue;

                std::lock_guard<std::mutex> lk(mutex_);
                const double dir = static_cast<double>(j.direction);
                j.position_rad   = static_cast<double>(s.position_rad) * dir;
                j.velocity_rads  = static_cast<double>(s.velocity_rads) * dir;
                j.effort_nm      = static_cast<double>(s.torque_nm) * dir;
                j.temperature    = static_cast<double>(s.temperature);
                j.rs_mode        = s.mode;
                j.rs_error_bits  = s.error_bits;
                j.feedback_valid = true;
                j.last_feedback_time = std::chrono::steady_clock::now();
                p.done = true;
                remaining--;

                if (s.error_bits != 0 || s.mode != 2) {
                    char buf[128];
                    snprintf(buf, sizeof(buf), "[WARN] RS '%s' mode=%d err=0x%02X",
                             j.name.c_str(), s.mode, s.error_bits);
                    log(LOG_WARN, buf);
                }
                break;
            }
        }

        for (const auto& p : rs_pending)
            if (!p.done && joints_[p.index].required) return false;
    }

    return true;
}

// ════════════════════════════════════════════════════════════════
//  Robstride param write (for external callers)
// ════════════════════════════════════════════════════════════════

bool SyncDriver::writeRobstrideParam(size_t index, uint16_t param_id, float value)
{
    if (index >= joints_.size() || joints_[index].motor_type != MotorType::ROBSTRIDE)
        return false;

    uint32_t wp_id; uint8_t wp_data[8];
    RobstrideProtocol::buildWriteParam(wp_id, wp_data,
        static_cast<uint8_t>(joints_[index].motor_id), param_id, value);

    return rsSendAndRecv(wp_id, wp_data, nullptr, nullptr, nullptr, 200, RS_MSG_WRITE_PARAM);
}

}  // namespace manipulator_sdk
