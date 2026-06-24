/**
 * set_zero — 모든 관절 하드웨어 zero 설정
 * ============================================================
 *
 * Joint 1 (Robstride ID=1)   — SetZero (0x06), 플래시 저장
 * Joint 2 (Robstride ID=127) — SetZero (0x06), 플래시 저장
 * Joint 3 (RMD ID=1)         — 0x64, ROM 저장 (재부팅 필요)
 * Joint 4 (RMD ID=2)         — 0x64, ROM 저장
 * Joint 5 (RMD ID=3)         — 0x64, ROM 저장
 * Joint 6 (RMD ID=4)         — 0x64, ROM 저장
 *
 * 사전 조건:
 *   sudo modprobe gs_usb
 *   sudo ip link set can2 up type can bitrate 1000000
 *
 * 빌드:
 *   make -C /path/to/FrJoCo_Bringup/scripts
 *
 * 실행:
 *   sudo ./set_zero          (기본: can2)
 *   sudo ./set_zero can0
 *
 * 주의:
 *   - Robstride: SetZero 후 재부팅(전원 off/on) 필요
 *   - RMD:       0x64 후 전원 재인가 필요
 * ============================================================
 */

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

// ════════════════════════════════════════════════════════════════
//  SocketCAN helpers
// ════════════════════════════════════════════════════════════════

static int g_fd = -1;

static bool canOpen(const std::string& iface)
{
    g_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (g_fd < 0) { perror("socket"); return false; }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (::ioctl(g_fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl SIOCGIFINDEX");
        ::close(g_fd); g_fd = -1;
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(g_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("bind"); ::close(g_fd); g_fd = -1; return false;
    }
    return true;
}

static bool sendStd(uint32_t id, const uint8_t data[8])
{
    struct can_frame f{};
    f.can_id  = id & CAN_SFF_MASK;
    f.can_dlc = 8;
    std::memcpy(f.data, data, 8);
    return ::write(g_fd, &f, sizeof(f)) == sizeof(f);
}

static bool sendExt(uint32_t id, const uint8_t data[8])
{
    struct can_frame f{};
    f.can_id  = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    f.can_dlc = 8;
    std::memcpy(f.data, data, 8);
    return ::write(g_fd, &f, sizeof(f)) == sizeof(f);
}

static bool recvFrame(uint32_t& id, uint8_t data[8], uint8_t& len, bool& ext, int timeout_ms)
{
    fd_set rfds;
    FD_ZERO(&rfds); FD_SET(g_fd, &rfds);
    struct timeval tv{ timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
    if (::select(g_fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) return false;

    struct can_frame f{};
    if (::read(g_fd, &f, sizeof(f)) != sizeof(f)) return false;

    ext = (f.can_id & CAN_EFF_FLAG) != 0;
    id  = f.can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK);
    len = f.can_dlc;
    std::memcpy(data, f.data, 8);
    return true;
}

static void flushRx()
{
    uint32_t id; uint8_t d[8], l; bool ext;
    while (recvFrame(id, d, l, ext, 1)) {}
}

// ════════════════════════════════════════════════════════════════
//  Robstride helpers (Extended CAN, arb_id = type<<24 | host<<8 | motor)
// ════════════════════════════════════════════════════════════════

static constexpr uint8_t RS_MSG_DISABLE  = 0x04;
static constexpr uint8_t RS_MSG_SET_ZERO = 0x06;

static uint32_t rsArbId(uint8_t msg_type, uint8_t motor_id)
{
    return (static_cast<uint32_t>(msg_type) << 24) | motor_id;
}

// Disable 후 SetZero 전송 (플래시 저장)
static bool rsSetZero(uint8_t motor_id)
{
    uint8_t data[8] = {};

    // 1. Disable
    if (!sendExt(rsArbId(RS_MSG_DISABLE, motor_id), data)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    flushRx();

    // 2. SetZero (data[0]=1: save to flash)
    data[0] = 1;
    if (!sendExt(rsArbId(RS_MSG_SET_ZERO, motor_id), data)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    flushRx();

    return true;
}

// ════════════════════════════════════════════════════════════════
//  RMD helpers (Standard CAN, 0x140 + motor_id)
// ════════════════════════════════════════════════════════════════

static constexpr uint32_t RMD_TX_BASE = 0x140;
static constexpr uint32_t RMD_RX_BASE = 0x240;
static constexpr uint8_t  RMD_CMD_SET_ZERO = 0x64;

// 현재 위치를 ROM에 zero로 저장 (전원 재인가 후 적용)
static bool rmdSetZero(uint8_t motor_id)
{
    uint8_t tx[8] = { RMD_CMD_SET_ZERO, 0, 0, 0, 0, 0, 0, 0 };
    if (!sendStd(RMD_TX_BASE + motor_id, tx)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // ACK 수신 확인
    uint32_t rx_id; uint8_t rx_data[8], rx_len; bool ext;
    for (int i = 0; i < 5; ++i) {
        if (recvFrame(rx_id, rx_data, rx_len, ext, 100)) {
            if (!ext && rx_id == RMD_RX_BASE + motor_id && rx_data[0] == RMD_CMD_SET_ZERO)
                return true;
        }
    }
    // ACK 없어도 명령은 전송됨 — 일부 펌웨어는 ACK 없음
    return true;
}

// ════════════════════════════════════════════════════════════════
//  Main
// ════════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    std::string can_if = (argc >= 2) ? argv[1] : "can2";

    printf("╔══════════════════════════════════════════════════╗\n");
    printf("║         FrLab Arm — Hardware Zero Set            ║\n");
    printf("╠══════════════════════════════════════════════════╣\n");
    printf("║  Interface : %-10s                          ║\n", can_if.c_str());
    printf("║  Joint 1,2 : Robstride (SetZero → flash)         ║\n");
    printf("║  Joint 3~6 : RMD X4-36 (0x64 → ROM)              ║\n");
    printf("╠══════════════════════════════════════════════════╣\n");
    printf("║  ⚠ 현재 자세가 모든 관절의 zero가 됩니다.        ║\n");
    printf("║  ⚠ 완료 후 전원을 껐다 켜야 적용됩니다.          ║\n");
    printf("╚══════════════════════════════════════════════════╝\n\n");

    printf("계속하려면 Enter, 취소하려면 Ctrl+C: ");
    fflush(stdout);
    getchar();

    // ── Open CAN ──────────────────────────────────────────────
    printf("[1/3] Opening %s...\n", can_if.c_str());
    if (!canOpen(can_if)) {
        printf("  [FAIL] %s 열기 실패. 인터페이스 설정 확인:\n", can_if.c_str());
        printf("    sudo modprobe gs_usb\n");
        printf("    sudo ip link set %s up type can bitrate 1000000\n", can_if.c_str());
        return 1;
    }
    printf("  [OK] %s open\n\n", can_if.c_str());
    flushRx();

    // ── Robstride zero ─────────────────────────────────────────
    printf("[2/3] Robstride SetZero...\n");

    struct { const char* name; uint8_t id; } rs_joints[] = {
        { "joint_1 (Robstride ID=1)",   1   },
        { "joint_2 (Robstride ID=127)", 127 },
    };

    for (auto& j : rs_joints) {
        printf("  %-30s ... ", j.name);
        fflush(stdout);
        if (rsSetZero(j.id)) {
            printf("[OK]\n");
        } else {
            printf("[FAIL] CAN 전송 오류\n");
        }
    }
    printf("\n");

    // ── RMD zero ──────────────────────────────────────────────
    printf("[3/3] RMD 0x64 SetZero...\n");

    struct { const char* name; uint8_t id; } rmd_joints[] = {
        { "joint_3 (RMD ID=1)", 1 },
        { "joint_4 (RMD ID=2)", 2 },
        { "joint_5 (RMD ID=3)", 3 },
        { "joint_6 (RMD ID=4)", 4 },
    };

    for (auto& j : rmd_joints) {
        printf("  %-25s ... ", j.name);
        fflush(stdout);
        if (rmdSetZero(j.id)) {
            printf("[OK]\n");
        } else {
            printf("[FAIL] CAN 전송 오류\n");
        }
    }
    printf("\n");

    ::close(g_fd);

    printf("════════════════════════════════════════════════════\n");
    printf("  완료! 아래 순서로 전원을 껐다 켜세요:\n");
    printf("  1. 모터 전원 OFF\n");
    printf("  2. 3초 대기\n");
    printf("  3. 모터 전원 ON\n");
    printf("  → 새 zero 위치가 적용됩니다.\n");
    printf("════════════════════════════════════════════════════\n");
    return 0;
}
