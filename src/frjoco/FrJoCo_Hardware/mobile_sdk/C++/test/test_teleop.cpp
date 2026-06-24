/**
 * ============================================================================
 * test_teleop — 키보드로 4WD 모바일 로봇 조종 (ROS 없이)
 * ============================================================================
 *
 * 조작법:
 *   W / ↑  : 전진
 *   S / ↓  : 후진
 *   A / ←  : 좌회전 (제자리)
 *   D / →  : 우회전 (제자리)
 *   Q      : 좌전진 (좌측 곡선)
 *   E      : 우전진 (우측 곡선)
 *   Z      : 좌후진
 *   C      : 우후진
 *   SPACE  : 정지
 *   +/-    : 속도 증가/감소
 *   ESC    : 종료
 *
 * 사용법:
 *   ./test_teleop /dev/ttyUSB0 /dev/ttyUSB1
 *   ./test_teleop                              ← 기본값: ttyUSB0 / ttyUSB1
 *
 * 빌드:
 *   cd C++/build && cmake .. && make test_teleop
 *
 * ============================================================================
 */

#include "mobile_sdk/md_driver.hpp"
#include "mobile_sdk/md_protocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cmath>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

// ── Terminal raw mode ──────────────────────────────────────
static struct termios g_orig_termios;
static bool g_raw_mode = false;

static void disableRawMode()
{
    if (g_raw_mode)
    {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &g_orig_termios);
        g_raw_mode = false;
    }
}

static void enableRawMode()
{
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    struct termios raw = g_orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);   // No echo, no line buffering
    raw.c_cc[VMIN]  = 0;                // Non-blocking
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
    g_raw_mode = true;
    atexit(disableRawMode);
}

static int readKey()
{
    int avail = 0;
    ioctl(STDIN_FILENO, FIONREAD, &avail);
    if (avail <= 0) return -1;

    char c;
    if (::read(STDIN_FILENO, &c, 1) != 1) return -1;

    // Arrow key: ESC [ A/B/C/D
    if (c == 27)
    {
        char seq[2];
        if (::read(STDIN_FILENO, &seq[0], 1) != 1) return 27;
        if (::read(STDIN_FILENO, &seq[1], 1) != 1) return 27;
        if (seq[0] == '[')
        {
            switch (seq[1])
            {
                case 'A': return 'w';   // ↑ = 전진
                case 'B': return 's';   // ↓ = 후진
                case 'C': return 'd';   // → = 우회전
                case 'D': return 'a';   // ← = 좌회전
            }
        }
        return 27;  // ESC
    }

    // Uppercase → lowercase
    if (c >= 'A' && c <= 'Z') c += 32;
    return c;
}

// ── Signal handler ─────────────────────────────────────────
static volatile bool g_running = true;

void signalHandler(int)
{
    g_running = false;
}

// ── Main ───────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    signal(SIGINT, signalHandler);

    const char* port_front = (argc >= 2) ? argv[1] : "/dev/ttyUSB0";
    const char* port_rear  = (argc >= 3) ? argv[2] : "/dev/ttyUSB1";

    // ── Banner ─────────────────────────────────────────────
    printf("\033[2J\033[H");  // Clear screen
    printf("╔══════════════════════════════════════════════════╗\n");
    printf("║     Mobile SDK Teleop — 키보드 로봇 조종          ║\n");
    printf("╠══════════════════════════════════════════════════╣\n");
    printf("║                                                  ║\n");
    printf("║      Q (좌전진)    W/↑ (전진)    E (우전진)       ║\n");
    printf("║      A/← (좌회전)  SPACE (정지)  D/→ (우회전)     ║\n");
    printf("║      Z (좌후진)    S/↓ (후진)    C (우후진)       ║\n");
    printf("║                                                  ║\n");
    printf("║      +/- : 속도 조절    ESC : 종료               ║\n");
    printf("║                                                  ║\n");
    printf("╚══════════════════════════════════════════════════╝\n\n");

    // ── Configure ──────────────────────────────────────────
    mobile_sdk::MDDriverConfig config;
    config.port_front = port_front;
    config.port_rear  = port_rear;
    config.baudrate   = 19200;

    mobile_sdk::MDDriver driver(config);

    printf("[INIT] Opening serial ports...\n");
    if (!driver.open())
    {
        printf("[FAIL] Serial ports open failed!\n");
        return 1;
    }
    printf("[OK] Ready! 키보드로 조종하세요.\n\n");

    enableRawMode();

    // ── Control parameters ─────────────────────────────────
    double linear_speed  = 1.0;   // rad/s (바퀴 회전 속도)
    double angular_speed = 1.0;   // rad/s (회전 시 바퀴 속도)
    const double SPEED_STEP = 0.2;
    const double MAX_SPEED  = 5.0;
    const double MIN_SPEED  = 0.2;
    const double DRIVE_DIRECTION = -1.0;  // 로봇 전방이 휠 양의 회전 방향과 반대

    double target_fl = 0, target_fr = 0, target_rl = 0, target_rr = 0;
    const char* motion_name = "정지";

    auto last_cmd_time = std::chrono::steady_clock::now();
    bool has_command = false;

    // ── Control loop ───────────────────────────────────────
    while (g_running)
    {
        int key = readKey();

        if (key >= 0)
        {
            has_command = true;
            last_cmd_time = std::chrono::steady_clock::now();

            switch (key)
            {
                case 'w':  // 전진
                    target_fl =  DRIVE_DIRECTION * linear_speed;
                    target_fr =  DRIVE_DIRECTION * linear_speed;
                    target_rl =  DRIVE_DIRECTION * linear_speed;
                    target_rr =  DRIVE_DIRECTION * linear_speed;
                    motion_name = "전진 ↑";
                    break;

                case 's':  // 후진
                    target_fl = -DRIVE_DIRECTION * linear_speed;
                    target_fr = -DRIVE_DIRECTION * linear_speed;
                    target_rl = -DRIVE_DIRECTION * linear_speed;
                    target_rr = -DRIVE_DIRECTION * linear_speed;
                    motion_name = "후진 ↓";
                    break;

                case 'a':  // 좌회전 (제자리)
                    target_fl = -angular_speed;
                    target_fr =  angular_speed;
                    target_rl = -angular_speed;
                    target_rr =  angular_speed;
                    motion_name = "좌회전 ←";
                    break;

                case 'd':  // 우회전 (제자리)
                    target_fl =  angular_speed;
                    target_fr = -angular_speed;
                    target_rl =  angular_speed;
                    target_rr = -angular_speed;
                    motion_name = "우회전 →";
                    break;

                case 'q':  // 좌전진 (좌측 곡선)
                    target_fl =  DRIVE_DIRECTION * linear_speed * 0.3;
                    target_fr =  DRIVE_DIRECTION * linear_speed;
                    target_rl =  DRIVE_DIRECTION * linear_speed * 0.3;
                    target_rr =  DRIVE_DIRECTION * linear_speed;
                    motion_name = "좌전진 ↖";
                    break;

                case 'e':  // 우전진 (우측 곡선)
                    target_fl =  DRIVE_DIRECTION * linear_speed;
                    target_fr =  DRIVE_DIRECTION * linear_speed * 0.3;
                    target_rl =  DRIVE_DIRECTION * linear_speed;
                    target_rr =  DRIVE_DIRECTION * linear_speed * 0.3;
                    motion_name = "우전진 ↗";
                    break;

                case 'z':  // 좌후진
                    target_fl = -DRIVE_DIRECTION * linear_speed * 0.3;
                    target_fr = -DRIVE_DIRECTION * linear_speed;
                    target_rl = -DRIVE_DIRECTION * linear_speed * 0.3;
                    target_rr = -DRIVE_DIRECTION * linear_speed;
                    motion_name = "좌후진 ↙";
                    break;

                case 'c':  // 우후진
                    target_fl = -DRIVE_DIRECTION * linear_speed;
                    target_fr = -DRIVE_DIRECTION * linear_speed * 0.3;
                    target_rl = -DRIVE_DIRECTION * linear_speed;
                    target_rr = -DRIVE_DIRECTION * linear_speed * 0.3;
                    motion_name = "우후진 ↘";
                    break;

                case ' ':  // 정지
                    target_fl = target_fr = target_rl = target_rr = 0;
                    motion_name = "정지 ■";
                    break;

                case '+': case '=':  // 속도 증가
                    linear_speed  = std::min(linear_speed  + SPEED_STEP, MAX_SPEED);
                    angular_speed = std::min(angular_speed + SPEED_STEP, MAX_SPEED);
                    break;

                case '-': case '_':  // 속도 감소
                    linear_speed  = std::max(linear_speed  - SPEED_STEP, MIN_SPEED);
                    angular_speed = std::max(angular_speed - SPEED_STEP, MIN_SPEED);
                    break;

                case 27:  // ESC
                    g_running = false;
                    break;
            }
        }

        // Auto-stop: 키 입력 없이 300ms 지나면 정지
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_cmd_time).count();
        if (has_command && elapsed > 0.3)
        {
            target_fl = target_fr = target_rl = target_rr = 0;
            motion_name = "정지 ■";
            has_command = false;
        }

        // Send velocity command
        driver.writeVelocity(target_fl, target_fr, target_rl, target_rr);
        driver.readState();

        // Display
        printf("\r  [%s]  속도=%.1f rad/s  "
               "FL=%.1f FR=%.1f RL=%.1f RR=%.1f (°/s)  "
               "Pos: %.0f° %.0f° %.0f° %.0f°    ",
               motion_name, linear_speed,
               driver.getVelocity(mobile_sdk::FRONT_LEFT)  * 57.2958,
               driver.getVelocity(mobile_sdk::FRONT_RIGHT) * 57.2958,
               driver.getVelocity(mobile_sdk::REAR_LEFT)   * 57.2958,
               driver.getVelocity(mobile_sdk::REAR_RIGHT)  * 57.2958,
               driver.getPosition(mobile_sdk::FRONT_LEFT)  * 57.2958,
               driver.getPosition(mobile_sdk::FRONT_RIGHT) * 57.2958,
               driver.getPosition(mobile_sdk::REAR_LEFT)   * 57.2958,
               driver.getPosition(mobile_sdk::REAR_RIGHT)  * 57.2958);
        fflush(stdout);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20 Hz
    }

    // ── Cleanup ────────────────────────────────────────────
    printf("\n\n[STOP] 모터 정지 중...\n");
    driver.emergencyStop();
    disableRawMode();
    driver.close();
    printf("[DONE] 종료.\n");

    return 0;
}
