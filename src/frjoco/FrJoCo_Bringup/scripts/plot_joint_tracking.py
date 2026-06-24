#!/usr/bin/env python3
"""
Joint tracking monitor — reference(q_target) vs actual(joint_states)
토픽:
  /dls_controller/joint_commands  : Float64MultiArray (q_target)
  /joint_states                   : sensor_msgs/JointState
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from collections import deque

HISTORY = 600   # 샘플 수 (100Hz × 6s)
DOF = 6
JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple', 'tab:brown']


class TrackingMonitor(Node):
    def __init__(self):
        super().__init__('plot_joint_tracking')

        self.lock = threading.Lock()
        self.t_ref = deque(maxlen=HISTORY)
        self.t_act = deque(maxlen=HISTORY)
        self.ref = [deque(maxlen=HISTORY) for _ in range(DOF)]
        self.act = [deque(maxlen=HISTORY) for _ in range(DOF)]
        self.t0 = None

        self.create_subscription(Float64MultiArray,
            '/dls_controller/joint_commands', self._cb_ref, 10)
        self.create_subscription(JointState,
            '/joint_states', self._cb_act, 10)

    def _now(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        return t - self.t0

    def _cb_ref(self, msg):
        if len(msg.data) < DOF:
            return
        t = self._now()
        with self.lock:
            self.t_ref.append(t)
            for i in range(DOF):
                self.ref[i].append(msg.data[i])

    def _cb_act(self, msg):
        if len(msg.position) < DOF:
            return
        t = self._now()
        with self.lock:
            self.t_act.append(t)
            for i in range(DOF):
                self.act[i].append(msg.position[i])


def main():
    rclpy.init(args=sys.argv)
    node = TrackingMonitor()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle('Joint Tracking: Reference vs Actual', fontsize=13)
    gs = gridspec.GridSpec(3, 2, hspace=0.45, wspace=0.35)
    axes = [fig.add_subplot(gs[i // 2, i % 2]) for i in range(DOF)]

    lines_ref = []
    lines_act = []
    lines_err = []

    # 각 joint subplot + 하단 error subplot은 twin axis 사용
    ax_err = []
    for i, ax in enumerate(axes):
        ax.set_title(JOINT_NAMES[i], fontsize=10)
        ax.set_ylabel('rad', fontsize=8)
        ax.tick_params(labelsize=7)
        lr, = ax.plot([], [], color=COLORS[i], lw=1.5, label='ref')
        la, = ax.plot([], [], color=COLORS[i], lw=1.5, ls='--', label='act', alpha=0.7)
        lines_ref.append(lr)
        lines_act.append(la)

        ax2 = ax.twinx()
        ax2.set_ylabel('err(rad)', fontsize=7, color='gray')
        ax2.tick_params(labelsize=6, colors='gray')
        le, = ax2.plot([], [], color='gray', lw=0.8, ls=':', label='err')
        lines_err.append(le)
        ax_err.append(ax2)

        ax.legend(fontsize=7, loc='upper left')

    plt.ion()
    plt.show()

    while rclpy.ok():
        with node.lock:
            t_ref = list(node.t_ref)
            t_act = list(node.t_act)
            ref = [list(node.ref[i]) for i in range(DOF)]
            act = [list(node.act[i]) for i in range(DOF)]

        for i in range(DOF):
            ax = axes[i]

            if t_ref and ref[i]:
                lines_ref[i].set_data(t_ref, ref[i])
            if t_act and act[i]:
                lines_act[i].set_data(t_act, act[i])

            # error: ref - act (같은 시간 축 interpolation)
            if t_ref and t_act and ref[i] and act[i]:
                t_r = np.array(t_ref)
                t_a = np.array(t_act)
                r   = np.array(ref[i])
                a   = np.array(act[i])
                # ref를 act 시간축에 interpolate
                t_common = t_a[t_a >= t_r[0]]
                if len(t_common) > 1:
                    r_interp = np.interp(t_common, t_r, r)
                    a_clip   = a[t_a >= t_r[0]]
                    err      = r_interp - a_clip
                    lines_err[i].set_data(t_common, err)
                    ax_err[i].relim()
                    ax_err[i].autoscale_view()

            ax.relim()
            ax.autoscale_view()

        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
