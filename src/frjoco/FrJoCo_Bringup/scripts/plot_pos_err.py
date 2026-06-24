#!/usr/bin/env python3
"""
DLS pos_err + joint_states 실시간 플롯.

실행:
  python3 plot_pos_err.py
"""

import threading
import time
import re
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Log

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

WINDOW_SEC = 30.0
MAX_PTS = 10000
N_JOINTS = 6


class Plotter(Node):
    def __init__(self):
        super().__init__("error_plotter")
        self._lock = threading.Lock()

        # --- pos_err trial ---
        self._t0 = None
        self._recording = False
        self._err_t  = deque(maxlen=MAX_PTS)
        self._err_v  = deque(maxlen=MAX_PTS)
        self._trials = []          # 완료된 이전 trial
        self._trial_label = ""

        # --- joint_states ---
        self._js_t  = deque(maxlen=MAX_PTS)   # 시간 (절대 monotonic)
        self._js_q  = [deque(maxlen=MAX_PTS) for _ in range(N_JOINTS)]
        self._js_t0 = None  # joint_states 용 t0 (첫 수신 시각)

        self.create_subscription(Float64MultiArray,
            "/dls_controller/error", self._err_cb, 10)
        self.create_subscription(JointState,
            "/joint_states", self._js_cb, 10)
        self.create_subscription(Log,
            "/rosout", self._rosout_cb, 100)

    def _rosout_cb(self, msg: Log):
        if "New target ACCEPTED" not in msg.msg:
            return
        with self._lock:
            if self._err_t:
                self._trials.append((list(self._err_t), list(self._err_v), self._trial_label))
                if len(self._trials) > 4:
                    self._trials.pop(0)
            self._t0 = time.monotonic()
            self._err_t.clear()
            self._err_v.clear()
            self._recording = True
            m = re.search(r"xyz=\[([^\]]+)\]", msg.msg)
            self._trial_label = f"→[{m.group(1)}]" if m else "→ new"

    def _err_cb(self, msg: Float64MultiArray):
        if not msg.data:
            return
        with self._lock:
            if not self._recording or self._t0 is None:
                return
            self._err_t.append(time.monotonic() - self._t0)
            self._err_v.append(msg.data[0])

    def _js_cb(self, msg: JointState):
        now = time.monotonic()
        with self._lock:
            if self._js_t0 is None:
                self._js_t0 = now
            t = now - self._js_t0
            self._js_t.append(t)
            for i in range(N_JOINTS):
                val = msg.position[i] if i < len(msg.position) else float("nan")
                self._js_q[i].append(val)

    def snapshot(self):
        with self._lock:
            return {
                "err_t":   list(self._err_t),
                "err_v":   list(self._err_v),
                "trials":  list(self._trials),
                "label":   self._trial_label,
                "js_t":    list(self._js_t),
                "js_q":    [list(d) for d in self._js_q],
            }


def main():
    rclpy.init()
    node = Plotter()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    COLORS = ["#1f77b4","#ff7f0e","#2ca02c","#d62728","#9467bd","#8c564b"]

    fig, (ax_err, ax_js) = plt.subplots(2, 1, figsize=(11, 8))
    fig.suptitle("DLS Controller Monitor", fontsize=13)

    # ── 위: pos_err ──
    ax_err.set_title("Position Error")
    ax_err.set_xlabel("Time since target accepted (s)")
    ax_err.set_ylabel("pos_err (m)")
    ax_err.set_xlim(0, WINDOW_SEC)
    ax_err.set_ylim(0, 0.6)
    ax_err.grid(True, alpha=0.4)
    ax_err.axhline(0.005, color="green", linestyle="--", lw=1, label="goal_tolerance 5mm")
    line_cur, = ax_err.plot([], [], color=COLORS[0], lw=2, label="current")
    prev_lines = [ax_err.plot([], [], color=c, lw=1, alpha=0.35, ls="--")[0]
                  for c in COLORS[1:5]]
    ax_err.legend(loc="upper right", fontsize=8)

    # ── 아래: joint_states ──
    ax_js.set_title("Joint Positions (joint_states)")
    ax_js.set_xlabel("Time (s)")
    ax_js.set_ylabel("position (rad)")
    ax_js.set_xlim(0, WINDOW_SEC)
    ax_js.set_ylim(-7, 7)
    ax_js.grid(True, alpha=0.4)
    js_lines = [ax_js.plot([], [], color=COLORS[i], lw=1.2,
                            label=f"j{i+1}")[0] for i in range(N_JOINTS)]
    ax_js.legend(loc="upper right", fontsize=8, ncol=3)

    plt.tight_layout()

    def update(_):
        d = node.snapshot()

        # pos_err
        if d["err_t"]:
            xmax = max(WINDOW_SEC, d["err_t"][-1] + 1)
            ymax = max(0.1, max(d["err_v"]) * 1.1)
            ax_err.set_xlim(0, xmax)
            ax_err.set_ylim(0, ymax)
            line_cur.set_data(d["err_t"], d["err_v"])
            line_cur.set_label(f"current {d['label']}")
        for i, tr in enumerate(d["trials"][-4:]):
            prev_lines[i].set_data(tr[0], tr[1])
            prev_lines[i].set_label(tr[2])
        ax_err.legend(loc="upper right", fontsize=8)

        # joint_states
        js_t = d["js_t"]
        if js_t:
            xmax = max(WINDOW_SEC, js_t[-1] + 1)
            ax_js.set_xlim(max(0, js_t[-1] - WINDOW_SEC), js_t[-1] + 0.5)
            for i, ln in enumerate(js_lines):
                ln.set_data(js_t, d["js_q"][i])
            vals = [v for q in d["js_q"] for v in q if v == v]
            if vals:
                ax_js.set_ylim(min(vals) - 0.2, max(vals) + 0.2)

        return [line_cur] + prev_lines + js_lines

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
