/**
 * ============================================================================
 * FrlabManipulator — Implementation (SyncDriver 기반)
 * ============================================================================
 */

#include "manipulator_sdk/frlab_manipulator.hpp"

namespace manipulator_sdk
{

FrlabManipulator::FrlabManipulator() = default;

FrlabManipulator::~FrlabManipulator()
{
    if (initialized_) shutdown();
}

// ════════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════════

void FrlabManipulator::buildConfig(
    const std::string& can_if,
    IntegratedDriverConfig& cfg,
    int read_deadline_ms,
    int read_poll_timeout_ms)
{
    cfg.can_if = can_if;

    // RMD (X4-36) dynamics
    cfg.rmd_acceleration    = 500;
    cfg.rmd_deceleration    = 500;
    cfg.rmd_max_vel_dps     = 360.0;
    cfg.rmd_default_vel_dps = X4_36_DEFAULT_VEL_DPS;

    // Robstride03 defaults
    cfg.rs_limit_speed   = RS03_DEFAULT_SPEED_LIMIT;
    cfg.rs_limit_torque  = RS03_TORQUE_MAX;
    cfg.rs_limit_current = 23.0f;
    cfg.read_deadline_ms = read_deadline_ms;
    cfg.read_poll_timeout_ms = read_poll_timeout_ms;

    cfg.joints.clear();

    // ── Joint 1: Robstride03, ID=1 ────────────────────────────
    {
        IntegratedDriverConfig::JointDef jd;
        jd.name           = "joint_1";
        jd.motor_type     = MotorType::ROBSTRIDE;
        jd.motor_id       = 1;
        jd.max_speed_rads = RS03_DEFAULT_SPEED_LIMIT;
        jd.direction      = -1;
        cfg.joints.push_back(jd);
    }

    // ── Joint 2: Robstride03, ID=127 ───────────────────────────
    {
        IntegratedDriverConfig::JointDef jd;
        jd.name           = "joint_2";
        jd.motor_type     = MotorType::ROBSTRIDE;
        jd.motor_id       = 127;
        jd.max_speed_rads = RS03_DEFAULT_SPEED_LIMIT;
        cfg.joints.push_back(jd);
    }

    // ── Joint 3: X4-36, ID=1 ──────────────────────────────────
    {
        IntegratedDriverConfig::JointDef jd;
        jd.name            = "joint_3";
        jd.motor_type      = MotorType::RMD;
        jd.motor_id        = 1;
        jd.torque_constant = X4_36_TORQUE_CONSTANT;
        jd.required        = true;
        cfg.joints.push_back(jd);
    }

    // ── Joint 4: X4-36, ID=2 ──────────────────────────────────
    {
        IntegratedDriverConfig::JointDef jd;
        jd.name            = "joint_4";
        jd.motor_type      = MotorType::RMD;
        jd.motor_id        = 2;
        jd.torque_constant = X4_36_TORQUE_CONSTANT;
        jd.direction       = -1;
        cfg.joints.push_back(jd);
    }

    // ── Joint 5: X4-36, ID=3 ──────────────────────────────────
    {
        IntegratedDriverConfig::JointDef jd;
        jd.name            = "joint_5";
        jd.motor_type      = MotorType::RMD;
        jd.motor_id        = 3;
        jd.torque_constant = X4_36_TORQUE_CONSTANT;
        jd.direction       = -1;
        cfg.joints.push_back(jd);
    }

    // ── Joint 6: X4-36, ID=4 ──────────────────────────────────
    {
        IntegratedDriverConfig::JointDef jd;
        jd.name            = "joint_6";
        jd.motor_type      = MotorType::RMD;
        jd.motor_id        = 4;
        jd.torque_constant = X4_36_TORQUE_CONSTANT;
        cfg.joints.push_back(jd);
    }
}

bool FrlabManipulator::init(
    const std::string& can_if,
    int read_deadline_ms,
    int read_poll_timeout_ms)
{
    IntegratedDriverConfig cfg;
    buildConfig(can_if, cfg, read_deadline_ms, read_poll_timeout_ms);

    if (!driver_.configure(cfg))
        return false;

    if (!driver_.activate())
        return false;

    initialized_ = true;
    return true;
}

void FrlabManipulator::shutdown()
{
    if (!initialized_) return;
    driver_.deactivate();
    driver_.cleanup();
    initialized_ = false;
}

// ════════════════════════════════════════════════════════════════
//  Main interface
// ════════════════════════════════════════════════════════════════

void FrlabManipulator::readCached(ManipulatorState& state)
{
    std::lock_guard<std::mutex> lock(driver_.mutex());
    for (int i = 0; i < MANIPULATOR_DOF; ++i) {
        const auto& j        = driver_.joint(static_cast<size_t>(i));
        state.position[i]    = j.position_rad;
        state.velocity[i]    = j.velocity_rads;
        state.effort[i]      = j.effort_nm;
        state.temperature[i] = j.temperature;
    }
    state.valid = true;
}

bool FrlabManipulator::read(ManipulatorState& state)
{
    // SyncDriver는 writeRead()로 write+read 동시 처리.
    // 단독 read 호출 시엔 캐시된 값을 반환.
    readCached(state);
    return state.valid;
}

bool FrlabManipulator::write(const std::array<double, MANIPULATOR_DOF>& pos_cmd,
                             const std::array<double, MANIPULATOR_DOF>& vel_cmd)
{
    {
        std::lock_guard<std::mutex> lock(driver_.mutex());
        for (int i = 0; i < MANIPULATOR_DOF; ++i) {
            auto& j            = driver_.joint(static_cast<size_t>(i));
            j.position_command = pos_cmd[i];
            j.velocity_command = vel_cmd[i];
        }
    }
    return driver_.writeRead();
}

bool FrlabManipulator::step(const std::array<double, MANIPULATOR_DOF>& pos_cmd,
                            const std::array<double, MANIPULATOR_DOF>& vel_cmd,
                            ManipulatorState& fb)
{
    bool ok = write(pos_cmd, vel_cmd);
    readCached(fb);
    fb.valid = ok;
    return ok;
}

// ════════════════════════════════════════════════════════════════
//  Configuration helpers
// ════════════════════════════════════════════════════════════════

bool FrlabManipulator::setRobstrideSpeedLimit(size_t joint_idx, float speed_rads)
{
    if (joint_idx >= 2) return false;
    driver_.joint(joint_idx).robstride_max_speed = speed_rads;
    return driver_.writeRobstrideParam(joint_idx, RS_PARAM_LIMIT_SPD, speed_rads);
}

bool FrlabManipulator::setRmdAcceleration(size_t /*joint_idx*/, uint32_t /*accel_dps2*/)
{
    // activate() 시 acceleration 이미 설정됨. 런타임 변경 미지원.
    return true;
}

}  // namespace manipulator_sdk
