/**
 * ============================================================================
 * SyncDriver — 동기식 RMD + Robstride 통합 드라이버
 * ============================================================================
 *
 * async reader / CAN 필터 없이 단일 스레드에서 순차 처리.
 *
 *   read()  : RMD  → 0x9C 요청 → 응답 대기
 *             RS03 → Enable 전송 → 피드백 응답 대기
 *   write() : RMD  → 0xA4 전송 (응답에서 상태도 파싱)
 *             RS03 → LOC_REF 파라미터 쓰기
 *
 * IntegratedDriverConfig / UnifiedJoint 구조체를 그대로 재사용.
 * ============================================================================
 */

#ifndef ARM_SDK__SYNC_DRIVER_HPP_
#define ARM_SDK__SYNC_DRIVER_HPP_

#include "manipulator_sdk/integrated_driver.hpp"  // UnifiedJoint, IntegratedDriverConfig, MotorType
#include "manipulator_sdk/socketcan_device.hpp"
#include "manipulator_sdk/rmd_protocol.hpp"
#include "manipulator_sdk/robstride_protocol.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <mutex>
#include <functional>

namespace manipulator_sdk
{

using SyncLogCallback = std::function<void(int level, const std::string& msg)>;

class SyncDriver
{
public:
    SyncDriver();
    ~SyncDriver();

    SyncDriver(const SyncDriver&) = delete;
    SyncDriver& operator=(const SyncDriver&) = delete;

    void setLogCallback(SyncLogCallback cb);

    // ── Lifecycle ──────────────────────────────────────────────

    /** Open CAN, test all motors, set initial modes. */
    bool configure(const IntegratedDriverConfig& config);

    /** Enable all motors, read initial positions. */
    bool activate();

    /** Disable all motors. */
    void deactivate();

    /** Close CAN socket. */
    void cleanup();

    // ── Per-cycle API ──────────────────────────────────────────

    /**
     * Write commands to all joints, then read state back.
     * RMD  : 0xA4 위치 명령 전송 → 응답에서 상태 파싱
     * RS03 : LOC_REF 쓰기 → Enable 전송 → 피드백 응답 수신
     * @return true if all required joints responded
     */
    bool writeRead();

    // ── Joint access ───────────────────────────────────────────

    size_t jointCount() const { return joints_.size(); }
    UnifiedJoint& joint(size_t i) { return joints_[i]; }
    const UnifiedJoint& joint(size_t i) const { return joints_[i]; }
    std::mutex& mutex() { return mutex_; }

    // ── Robstride helpers ──────────────────────────────────────

    bool writeRobstrideParam(size_t index, uint16_t param_id, float value);

private:
    void log(int level, const std::string& msg);

    // Synchronous send+receive for Robstride (extended CAN)
    bool rsSendAndRecv(uint32_t tx_id, const uint8_t* tx_data,
                       uint32_t* rx_id = nullptr, uint8_t* rx_data = nullptr,
                       uint8_t* rx_len = nullptr, int timeout_ms = 500,
                       uint8_t expected_msg_type = 0xFF);

    // Drain stale frames from socket buffer
    void clearBuffer();

    // Per-joint step
    bool stepRmd(size_t index);        // write 0xA4 → parse response
    bool stepRobstride(size_t index);  // write LOC_REF → Enable → parse feedback

    SocketCANDevice can_;
    IntegratedDriverConfig config_;
    std::vector<UnifiedJoint> joints_;

    std::mutex mutex_;
    SyncLogCallback log_cb_;
};

}  // namespace manipulator_sdk

#endif  // ARM_SDK__SYNC_DRIVER_HPP_
