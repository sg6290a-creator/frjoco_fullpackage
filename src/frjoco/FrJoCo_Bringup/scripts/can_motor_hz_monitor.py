#!/usr/bin/env python3
"""Passive CAN monitor for per-motor command/feedback rates.

The monitor opens a SocketCAN RAW socket and only listens. It does not send any
motor commands, so it can run next to ros2_control while the arm is already
active.
"""

import argparse
import os
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass, field


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_SFF_MASK = 0x000007FF
CAN_EFF_MASK = 0x1FFFFFFF

RMD_TX_BASE = 0x140
RMD_RX_BASE = 0x240
RMD_IDS = {1: "J3 RMD", 2: "J4 RMD", 3: "J5 RMD", 4: "J6 RMD"}
RMD_RESPONSE_CMDS = {0x9C, 0xA4, 0x60, 0x43, 0x80, 0x88}

RS_FEEDBACK = 0x02
RS_HOST_COMMANDS = {0x01, 0x03, 0x04, 0x06, 0x11, 0x12}
RS_IDS = {1: "J1 RS03", 127: "J2 RS03"}

CAN_FRAME = struct.Struct("=IB3x8s")


@dataclass
class MotorStats:
    label: str
    rx_total: int = 0
    tx_total: int = 0
    rx_window: int = 0
    tx_window: int = 0
    last_rx: float | None = None
    last_tx: float | None = None
    rx_dt_sum: float = 0.0
    rx_dt_count: int = 0
    rx_max_gap: float = 0.0
    recent_commands: dict[int, int] = field(default_factory=dict)

    def mark_rx(self, now: float) -> None:
        if self.last_rx is not None:
            dt = now - self.last_rx
            self.rx_dt_sum += dt
            self.rx_dt_count += 1
            self.rx_max_gap = max(self.rx_max_gap, dt)
        self.last_rx = now
        self.rx_total += 1
        self.rx_window += 1

    def mark_tx(self, now: float, cmd: int) -> None:
        self.last_tx = now
        self.tx_total += 1
        self.tx_window += 1
        self.recent_commands[cmd] = self.recent_commands.get(cmd, 0) + 1

    def take_window(self, window_sec: float) -> tuple[float, float]:
        rx_hz = self.rx_window / window_sec if window_sec > 0 else 0.0
        tx_hz = self.tx_window / window_sec if window_sec > 0 else 0.0
        self.rx_window = 0
        self.tx_window = 0
        return tx_hz, rx_hz

    def avg_rx_dt_ms(self) -> float:
        if self.rx_dt_count == 0:
            return 0.0
        return 1000.0 * self.rx_dt_sum / self.rx_dt_count

    def age_ms(self, now: float) -> float | None:
        if self.last_rx is None:
            return None
        return 1000.0 * (now - self.last_rx)


def build_stats() -> dict[str, MotorStats]:
    stats: dict[str, MotorStats] = {}
    for motor_id, label in RS_IDS.items():
        stats[f"rs:{motor_id}"] = MotorStats(f"{label} id={motor_id}")
    for motor_id, label in RMD_IDS.items():
        stats[f"rmd:{motor_id}"] = MotorStats(f"{label} id={motor_id}")
    return stats


def decode_frame(frame: bytes):
    can_id_raw, dlc, data = CAN_FRAME.unpack(frame)
    if can_id_raw & (CAN_RTR_FLAG | CAN_ERR_FLAG):
        return None

    is_extended = bool(can_id_raw & CAN_EFF_FLAG)
    arb_id = can_id_raw & (CAN_EFF_MASK if is_extended else CAN_SFF_MASK)
    return is_extended, arb_id, dlc, data[:dlc]


def classify_frame(is_extended: bool, arb_id: int, data: bytes):
    if is_extended:
        msg_type = (arb_id >> 24) & 0x1F
        if msg_type == RS_FEEDBACK:
            motor_id = (arb_id >> 8) & 0xFF
            if motor_id in RS_IDS:
                return f"rs:{motor_id}", "rx", msg_type
        elif msg_type in RS_HOST_COMMANDS:
            motor_id = arb_id & 0xFF
            if motor_id in RS_IDS:
                return f"rs:{motor_id}", "tx", msg_type
        return None

    if RMD_RX_BASE + 1 <= arb_id <= RMD_RX_BASE + 4:
        motor_id = arb_id - RMD_RX_BASE
        if motor_id in RMD_IDS and data and data[0] in RMD_RESPONSE_CMDS:
            return f"rmd:{motor_id}", "rx", data[0]

    if RMD_TX_BASE + 1 <= arb_id <= RMD_TX_BASE + 4:
        motor_id = arb_id - RMD_TX_BASE
        if motor_id in RMD_IDS and data:
            return f"rmd:{motor_id}", "tx", data[0]

    return None


def clear_screen(enabled: bool) -> None:
    if enabled:
        sys.stdout.write("\033[2J\033[H")


def command_summary(stats: MotorStats) -> str:
    if not stats.recent_commands:
        return "-"
    parts = []
    for cmd, count in sorted(stats.recent_commands.items()):
        parts.append(f"0x{cmd:02X}:{count}")
    stats.recent_commands.clear()
    return " ".join(parts)


def print_report(stats: dict[str, MotorStats], window_sec: float, now: float, clear: bool) -> None:
    clear_screen(clear)
    print(
        "CAN motor rate monitor "
        f"| window={window_sec:.2f}s | passive listen only | Ctrl+C stop"
    )
    print(
        f"{'motor':<16} {'tx_hz':>8} {'rx_hz':>8} {'rx_total':>9} "
        f"{'avg_rx_dt':>10} {'max_gap':>9} {'age':>8} {'cmds/window'}"
    )
    print("-" * 92)

    for key in ("rs:1", "rs:127", "rmd:1", "rmd:2", "rmd:3", "rmd:4"):
        s = stats[key]
        tx_hz, rx_hz = s.take_window(window_sec)
        age = s.age_ms(now)
        age_txt = "---" if age is None else f"{age:7.1f}"
        print(
            f"{s.label:<16} {tx_hz:8.1f} {rx_hz:8.1f} {s.rx_total:9d} "
            f"{s.avg_rx_dt_ms():9.2f} {1000.0 * s.rx_max_gap:8.1f} "
            f"{age_txt:>8} {command_summary(s)}"
        )

    sys.stdout.flush()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Passive per-motor CAN TX/RX Hz monitor for the FRLab arm."
    )
    parser.add_argument(
        "can_interface",
        nargs="?",
        default="can2",
        help="SocketCAN interface to listen on. Default: can2",
    )
    parser.add_argument(
        "--window",
        type=float,
        default=1.0,
        help="Reporting window in seconds. Default: 1.0",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Do not clear the terminal between reports.",
    )
    args = parser.parse_args()

    if args.window <= 0.0:
        print("--window must be positive", file=sys.stderr)
        return 2

    stats = build_stats()
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((args.can_interface,))
        sock.setblocking(False)
    except OSError as exc:
        print(f"Failed to bind {args.can_interface}: {exc}", file=sys.stderr)
        print(
            "Check that the interface is up, e.g. "
            f"`ip link show {args.can_interface}`.",
            file=sys.stderr,
        )
        return 1

    next_report = time.monotonic() + args.window
    clear = not args.no_clear and os.isatty(sys.stdout.fileno())

    try:
        while True:
            timeout = max(0.0, next_report - time.monotonic())
            readable, _, _ = select.select([sock], [], [], timeout)
            now = time.monotonic()

            if readable:
                while True:
                    try:
                        frame = sock.recv(CAN_FRAME.size)
                    except BlockingIOError:
                        break
                    decoded = decode_frame(frame)
                    if decoded is None:
                        continue
                    classified = classify_frame(*decoded)
                    if classified is None:
                        continue
                    key, direction, command = classified
                    if direction == "rx":
                        stats[key].mark_rx(now)
                    else:
                        stats[key].mark_tx(now, command)

            if now >= next_report:
                elapsed = args.window + (now - next_report)
                print_report(stats, elapsed, now, clear)
                next_report = now + args.window

    except KeyboardInterrupt:
        print()
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
