/**
 * ============================================================================
 * test_wheels=
 * ============================================================================
 *
 * 사용법:
 *   ./test_wheels /dev/ttyUSB0 /dev/ttyUSB1
 *   ./test_wheels                              ← 기본값: ttyUSB0 / ttyUSB1
 *
 * 빌드 (C++/ 폴더에서):
 *   mkdir -p build && cd build
 *   cmake .. && make test_wheels
 *   ./test_wheels
 *
 * ============================================================================
 */

#include "mobile_sdk/md_driver.hpp"
#include "mobile_sdk/md_protocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <chrono>
#include <thread>
#include <cstring>

static volatile bool g_running = true;

void signalHandler(int) { g_running = false; }

int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);

    const char* port_front = (argc >= 2) ? argv[1] : "/dev/ttyUSB0";
    const char* port_rear  = (argc >= 3) ? argv[2] : "/dev/ttyUSB1";

    printf("╔══════════════════════════════════════════╗\n");
    printf("║  Mobile SDK Test — MD200T 4WD            ║\n");
    printf("╠══════════════════════════════════════════╣\n");
    printf("║  Front: %-33s║\n", port_front);
    printf("║  Rear:  %-33s║\n", port_rear);
    printf("╚══════════════════════════════════════════╝\n\n");

    // ── Configure ──────────────────────────────────────────
    mobile_sdk::MDDriverConfig config;
    config.port_front = port_front;
    config.port_rear  = port_rear;
    config.baudrate   = 57600;

    mobile_sdk::MDDriver driver(config);

    // ── Open ───────────────────────────────────────────────
    printf("[TEST] Step 1: Opening serial ports...\n");
    if (!driver.open()) {
        printf("[FAIL] Failed to open serial ports.\n");
        printf("  Check:\n");
        printf("    - Ports exist? (ls -l %s %s)\n", port_front, port_rear);
        printf("    - Permission? (sudo chmod 666 /dev/ttyUSB*)\n");
        printf("    - Cables connected?\n");
        return 1;
    }
    printf("[OK] Both ports opened.\n\n");

    // ── Phase 1: Read initial state ────────────────────────
    printf("[TEST] Step 2: Reading initial state...\n");
    driver.readState();
    printf("  FL=%.1f°  FR=%.1f°  RL=%.1f°  RR=%.1f°\n\n",
           driver.getPosition(mobile_sdk::FRONT_LEFT)  * 180.0 / 3.14159265,
           driver.getPosition(mobile_sdk::FRONT_RIGHT) * 180.0 / 3.14159265,
           driver.getPosition(mobile_sdk::REAR_LEFT)   * 180.0 / 3.14159265,
           driver.getPosition(mobile_sdk::REAR_RIGHT)  * 180.0 / 3.14159265);

    // ── Phase 2: Drive forward ─────────────────────────────
    double speed_rads = 1.0;  // 1 rad/s ≈ 57 deg/s (느린 속도)
    printf("[TEST] Step 3: Driving all 4 wheels forward at %.1f rad/s for 3 seconds...\n", speed_rads);
    printf("  ⚠ 로봇이 움직입니다! 안전 확인하세요.\n");
    printf("  (Ctrl+C로 즉시 정지)\n\n");

    auto drive_start = std::chrono::steady_clock::now();
    int cycle = 0;

    while (g_running) {
        auto elapsed = std::chrono::steady_clock::now() - drive_start;
        double elapsed_sec = std::chrono::duration<double>(elapsed).count();

        if (elapsed_sec >= 3.0) break;  // 3초 후 정지

        // 4바퀴 전진 (양수 = 전진)
        driver.writeVelocity(speed_rads, speed_rads, speed_rads, speed_rads);
        driver.readState();

        if (cycle % 5 == 0) {
            printf("\r  [%.1fs] FL=%.1f°/s  FR=%.1f°/s  RL=%.1f°/s  RR=%.1f°/s  ",
                   elapsed_sec,
                   driver.getVelocity(mobile_sdk::FRONT_LEFT)  * 180.0 / 3.14159265,
                   driver.getVelocity(mobile_sdk::FRONT_RIGHT) * 180.0 / 3.14159265,
                   driver.getVelocity(mobile_sdk::REAR_LEFT)   * 180.0 / 3.14159265,
                   driver.getVelocity(mobile_sdk::REAR_RIGHT)  * 180.0 / 3.14159265);
            fflush(stdout);
        }

        cycle++;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // ── Phase 3: Stop ──────────────────────────────────────
    printf("\n\n[TEST] Step 4: Stopping...\n");
    driver.emergencyStop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ── Phase 4: Final state ───────────────────────────────
    driver.readState();
    printf("  Final positions:\n");
    printf("  FL=%.1f°  FR=%.1f°  RL=%.1f°  RR=%.1f°\n",
           driver.getPosition(mobile_sdk::FRONT_LEFT)  * 180.0 / 3.14159265,
           driver.getPosition(mobile_sdk::FRONT_RIGHT) * 180.0 / 3.14159265,
           driver.getPosition(mobile_sdk::REAR_LEFT)   * 180.0 / 3.14159265,
           driver.getPosition(mobile_sdk::REAR_RIGHT)  * 180.0 / 3.14159265);

    driver.close();
    printf("\n[DONE] Test complete.\n");

    return 0;
}
