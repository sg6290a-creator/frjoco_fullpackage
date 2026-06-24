#!/usr/bin/env python3
"""
nav_trials.py  —  navigation trial recorder + visualizer

출력 구조
---------
~/nav_trials/
  001/
    summary.csv   ← 이 시험 1행 요약
    path.csv      ← 이 시험 경로 포인트
    result.png    ← 지도 위 경로 이미지 (자동 생성)
  002/ ...
  all_summary.csv ← 전체 누적 요약
  all_paths.csv   ← 전체 누적 경로

서브커맨드
----------
record      ROS2 노드 실행; Ctrl+C 종료 시 자동 시각화
visualize   CSV → 이미지 생성
              --dir ~/nav_trials/001          특정 시험 폴더
              --summary S.csv --paths P.csv   커스텀 CSV 지정
              (인자 없으면 all_*.csv 전체)
print       터미널 요약 테이블

사용 예시
---------
python3 nav_trials.py record

python3 nav_trials.py visualize                        # 전체
python3 nav_trials.py visualize --dir ~/nav_trials/003 # 특정 시험
python3 nav_trials.py visualize --summary my.csv --paths my_path.csv --output out.png

python3 nav_trials.py print
"""

import argparse
import csv
import math
import os
import sys
from datetime import datetime

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None

# --------------------------------------------------------------------------- #
OUTPUT_DIR   = os.path.expanduser('~/nav_trials')
ALL_SUMMARY  = os.path.join(OUTPUT_DIR, 'all_summary.csv')
ALL_PATHS    = os.path.join(OUTPUT_DIR, 'all_paths.csv')
if get_package_share_directory is not None:
    try:
        DEFAULT_MAP = os.path.join(
            get_package_share_directory('robot_nav2'), 'maps', 'my_map.yaml')
    except Exception:
        DEFAULT_MAP = os.path.expanduser('~/nav_trials/my_map.yaml')
else:
    DEFAULT_MAP = os.path.expanduser('~/nav_trials/my_map.yaml')
XY_TOLERANCE = 0.20

SUMMARY_HEADER = [
    'trial_id', 'datetime',
    'goal_x', 'goal_y', 'goal_yaw',
    'start_x', 'start_y',
    'final_x', 'final_y', 'final_yaw',
    'xy_error_m', 'yaw_error_rad',
    'result', 'duration_sec', 'path_points',
]
PATHS_HEADER = ['trial_id', 'seq', 'x', 'y', 'yaw', 'elapsed_sec']


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _ensure_csv(path, header):
    if not os.path.exists(path):
        with open(path, 'w', newline='') as f:
            csv.writer(f).writerow(header)


# =========================================================================== #
# 시각화 핵심 함수 (record 자동 호출 + visualize 서브커맨드 공유)
# =========================================================================== #
def _render(summary_csv, paths_csv, output_png, map_yaml=DEFAULT_MAP,
            trial_filter=None, title='Navigation Trial Results'):
    """
    summary_csv, paths_csv → output_png 이미지 생성.
    trial_filter: None(전체) 또는 정수 리스트.
    성공 시 True, 데이터 없으면 False 반환.
    """
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        from matplotlib.lines import Line2D
        import numpy as np
        import yaml
        from PIL import Image
    except ImportError as e:
        print(f'[visualize] 패키지 부족: {e}\n'
              'pip install matplotlib pillow pyyaml numpy', file=sys.stderr)
        return False

    # 지도 로드
    try:
        with open(map_yaml) as f:
            meta = yaml.safe_load(f)
        img_path = os.path.join(os.path.dirname(map_yaml), meta['image'])
        img = np.array(Image.open(img_path).convert('L'))
        res, origin = meta['resolution'], meta['origin']
        h, w = img.shape
    except Exception as e:
        print(f'[visualize] 지도 로드 실패: {e}', file=sys.stderr)
        return False

    # CSV 로드
    trials = {}
    try:
        with open(summary_csv) as f:
            for row in csv.DictReader(f):
                tid = int(row['trial_id'])
                trials[tid] = row
    except Exception as e:
        print(f'[visualize] summary CSV 로드 실패: {e}', file=sys.stderr)
        return False

    if not trials:
        print('[visualize] 데이터 없음 — 시각화 건너뜀')
        return False

    paths = {}
    try:
        with open(paths_csv) as f:
            for row in csv.DictReader(f):
                tid = int(row['trial_id'])
                paths.setdefault(tid, []).append(
                    (float(row['x']), float(row['y'])))
    except Exception:
        pass

    if trial_filter:
        trials = {k: v for k, v in trials.items() if k in trial_filter}

    sorted_trials = sorted(trials.items())
    n = len(sorted_trials)
    cmap   = plt.cm.get_cmap('tab10')
    colors = [cmap(i % 10) for i in range(n)]

    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 11,
        'axes.titlesize': 12, 'axes.labelsize': 11,
        'legend.fontsize': 9, 'figure.dpi': 150,
    })

    fig, ax = plt.subplots(figsize=(12, 8))
    fig.suptitle(title, fontsize=14, fontweight='bold', y=1.01)

    map_ext = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
    ax.imshow(img, cmap='gray', origin='upper', extent=map_ext, alpha=0.75)
    ax.set_aspect('equal')
    ax.set_title('Robot Trajectories', pad=8)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')

    handles = []
    for idx, (tid, t) in enumerate(sorted_trials):
        color   = colors[idx]
        success = t['result'] == 'SUCCESS'
        ls      = '-' if success else '--'
        gx, gy  = float(t['goal_x']),  float(t['goal_y'])
        fx, fy  = float(t['final_x']), float(t['final_y'])
        err     = float(t['xy_error_m'])

        if tid in paths and len(paths[tid]) > 1:
            xs = [p[0] for p in paths[tid]]
            ys = [p[1] for p in paths[tid]]
            ax.plot(xs, ys, color=color, ls=ls, lw=2.0,
                    alpha=0.9, zorder=2, solid_capstyle='round')
            ax.plot(xs[0], ys[0], 'o', color=color, ms=7,
                    markeredgecolor='white', markeredgewidth=0.8, zorder=4)

        ax.plot(gx, gy, '*', color=color, ms=13,
                markeredgecolor='white', markeredgewidth=0.5, zorder=5)
        ax.plot(fx, fy, 's', color=color, ms=7,
                markeredgecolor='white', markeredgewidth=0.5, zorder=5)
        ax.plot([gx, fx], [gy, fy], color=color, ls=':', lw=1.0,
                alpha=0.6, zorder=3)

        sym   = 'S' if success else 'F'
        label = f'T{tid:03d} [{sym}]  {err:.3f} m'
        handles.append(mpatches.Patch(color=color, label=label))

    marker_legend = [
        Line2D([0],[0], marker='o', color='k', ls='None', ms=7,
               markeredgecolor='white', label='Start'),
        Line2D([0],[0], marker='*', color='k', ls='None', ms=11,
               markeredgecolor='white', label='Goal'),
        Line2D([0],[0], marker='s', color='k', ls='None', ms=7,
               markeredgecolor='white', label='Final'),
        Line2D([0],[0], color='k', ls='-',  lw=1.8, label='Path (success)'),
        Line2D([0],[0], color='k', ls='--', lw=1.8, label='Path (failed)'),
    ]
    leg = ax.legend(handles=handles + marker_legend,
                    loc='upper left', fontsize=8.5,
                    framealpha=0.90, edgecolor='#aaaaaa',
                    handlelength=1.6, borderpad=0.6, labelspacing=0.35)
    leg.get_frame().set_linewidth(0.6)

    fig.tight_layout()
    os.makedirs(os.path.dirname(output_png) or '.', exist_ok=True)
    plt.savefig(output_png, dpi=300, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    plt.close(fig)
    print(f'[visualize] 저장: {output_png}')
    return True


# =========================================================================== #
# RECORD
# =========================================================================== #
def cmd_record(_args):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from action_msgs.msg import GoalStatusArray

    class NavTrialRecorder(Node):
        def __init__(self):
            super().__init__('nav_trial_recorder')

            os.makedirs(OUTPUT_DIR, exist_ok=True)
            _ensure_csv(ALL_SUMMARY, SUMMARY_HEADER)
            _ensure_csv(ALL_PATHS,   PATHS_HEADER)

            self._trial_id      = 0
            self._recording     = False
            self._goal          = None
            self._start_pose    = None
            self._current_pose  = None
            self._path          = []
            self._t0            = None
            self._prev_statuses = set()
            self._finished_ids  = []   # 이번 세션에 완료된 trial id 목록

            from tf2_ros import Buffer, TransformListener
            self._tf_buffer   = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

            self.create_subscription(
                PoseStamped, '/goal_pose', self._on_goal, 10)
            self.create_subscription(
                GoalStatusArray,
                '/navigate_to_pose/_action/status', self._on_status, 10)

            self.create_timer(0.02, self._poll_tf)

            self.get_logger().info(f'NavTrialRecorder ready. Output: {OUTPUT_DIR}')
            self.get_logger().info('RViz2 에서 Nav2 Goal 을 보내면 기록이 시작됩니다.')

        # ------------------------------------------------------------------ #
        def _next_trial_id(self):
            max_id = 0
            try:
                with open(ALL_SUMMARY) as f:
                    for row in csv.DictReader(f):
                        try:
                            tid = int(row['trial_id'])
                            if tid > max_id:
                                max_id = tid
                        except (ValueError, KeyError):
                            pass
            except FileNotFoundError:
                pass
            return max_id + 1

        def _get_pose_from_tf(self):
            from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
            from geometry_msgs.msg import Pose
            try:
                tf = self._tf_buffer.lookup_transform(
                    'map', 'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05))
                p = Pose()
                p.position.x  = tf.transform.translation.x
                p.position.y  = tf.transform.translation.y
                p.position.z  = tf.transform.translation.z
                p.orientation = tf.transform.rotation
                return p
            except (LookupException, ConnectivityException, ExtrapolationException):
                return None

        def _on_goal(self, msg: PoseStamped):
            if self._recording:
                self.get_logger().warn(
                    f'[Trial {self._trial_id:03d}] 새 목표 수신 — INTERRUPTED 처리')
                self._finish('INTERRUPTED')

            self._trial_id      = self._next_trial_id()
            self._goal          = msg
            self._start_pose    = self._current_pose
            self._path          = []
            self._prev_statuses = set()
            self._t0            = self.get_clock().now()
            self._recording     = True

            gx = msg.pose.position.x
            gy = msg.pose.position.y
            self.get_logger().info(
                f'[Trial {self._trial_id:03d}] 기록 시작 → goal=({gx:.2f}, {gy:.2f})')

        def _poll_tf(self):
            pose = self._get_pose_from_tf()
            if pose is None:
                return
            self._current_pose = pose
            if not self._recording:
                return
            elapsed = (self.get_clock().now() - self._t0).nanoseconds / 1e9
            self._path.append((elapsed,
                                pose.position.x,
                                pose.position.y,
                                _yaw(pose.orientation)))

        def _on_status(self, msg: GoalStatusArray):
            if not self._recording:
                return
            for s in msg.status_list:
                if s.status == 4 and 4 not in self._prev_statuses:
                    self._prev_statuses.add(4); self._finish('SUCCESS'); return
                if s.status == 6 and 6 not in self._prev_statuses:
                    self._prev_statuses.add(6); self._finish('ABORTED');  return
                if s.status == 5 and 5 not in self._prev_statuses:
                    self._prev_statuses.add(5); self._finish('CANCELED'); return

        def _finish(self, result: str):
            self._recording = False
            duration = (self.get_clock().now() - self._t0).nanoseconds / 1e9

            fp   = self._current_pose
            fx   = fp.position.x        if fp else float('nan')
            fy   = fp.position.y        if fp else float('nan')
            fyaw = _yaw(fp.orientation) if fp else float('nan')

            sp = self._start_pose
            sx = sp.position.x if sp else float('nan')
            sy = sp.position.y if sp else float('nan')

            gx   = self._goal.pose.position.x
            gy   = self._goal.pose.position.y
            gyaw = _yaw(self._goal.pose.orientation)

            xy_err  = math.sqrt((fx - gx)**2 + (fy - gy)**2)
            yaw_err = abs(math.atan2(math.sin(fyaw - gyaw),
                                     math.cos(fyaw - gyaw)))

            row = [
                self._trial_id,
                datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                f'{gx:.4f}',  f'{gy:.4f}',  f'{gyaw:.4f}',
                f'{sx:.4f}',  f'{sy:.4f}',
                f'{fx:.4f}',  f'{fy:.4f}',  f'{fyaw:.4f}',
                f'{xy_err:.4f}', f'{yaw_err:.4f}',
                result, f'{duration:.2f}', len(self._path),
            ]
            path_rows = [
                [self._trial_id, seq,
                 f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}', f'{elapsed:.2f}']
                for seq, (elapsed, x, y, yaw) in enumerate(self._path)
            ]

            # ── 전체 누적 CSV (all_*.csv) ──
            with open(ALL_SUMMARY, 'a', newline='') as f:
                csv.writer(f).writerow(row)
            with open(ALL_PATHS, 'a', newline='') as f:
                w = csv.writer(f)
                for r in path_rows:
                    w.writerow(r)

            # ── 시험별 폴더 (001/, 002/, ...) ──
            trial_dir  = os.path.join(OUTPUT_DIR, f'{self._trial_id:03d}')
            trial_sum  = os.path.join(trial_dir, 'summary.csv')
            trial_path = os.path.join(trial_dir, 'path.csv')
            trial_png  = os.path.join(trial_dir, 'result.png')
            os.makedirs(trial_dir, exist_ok=True)

            with open(trial_sum, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(SUMMARY_HEADER)
                w.writerow(row)
            with open(trial_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(PATHS_HEADER)
                for r in path_rows:
                    w.writerow(r)

            judge = '✓ SUCCESS' if result == 'SUCCESS' else f'✗ {result}'
            self.get_logger().info(
                f'[Trial {self._trial_id:03d}] {judge} | '
                f'xy_err={xy_err:.3f} m  yaw_err={math.degrees(yaw_err):.1f}° | '
                f't={duration:.1f}s | pts={len(self._path)}')
            self.get_logger().info(
                f'  CSV → {trial_dir}/')

            self._finished_ids.append(self._trial_id)

            # ── 이 시험 단독 PNG ──
            _render(trial_sum, trial_path, trial_png,
                    title=f'Trial {self._trial_id:03d} — {result}')

        def auto_visualize_all(self):
            """종료 시 이번 세션 전체 시험을 one-shot 이미지로도 저장."""
            if not self._finished_ids:
                return
            all_png = os.path.join(OUTPUT_DIR, 'all_result.png')
            self.get_logger().info(
                f'세션 종합 이미지 생성 중 → {all_png}')
            _render(ALL_SUMMARY, ALL_PATHS, all_png,
                    title='All Trials')

    rclpy.init()
    node = NavTrialRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node._recording:
            node.get_logger().warn('Ctrl+C — 진행 중 시험을 INTERRUPTED 로 저장합니다.')
            node._finish('INTERRUPTED')
    finally:
        node.auto_visualize_all()
        node.destroy_node()
        rclpy.shutdown()


# =========================================================================== #
# VISUALIZE
# =========================================================================== #
def cmd_visualize(args):
    if args.dir:
        d = os.path.expanduser(args.dir)
        summary = os.path.join(d, 'summary.csv')
        paths   = os.path.join(d, 'path.csv')
        output  = args.output or os.path.join(d, 'result.png')
        tid     = os.path.basename(d.rstrip('/'))
        title   = f'Trial {tid}'
    else:
        summary = args.summary or ALL_SUMMARY
        paths   = args.paths   or ALL_PATHS
        output  = args.output  or os.path.join(OUTPUT_DIR, 'all_result.png')
        title   = 'Navigation Trial Results'

    trial_filter = args.trials if args.trials else None
    ok = _render(summary, paths, output, map_yaml=args.map,
                 trial_filter=trial_filter, title=title)
    if not ok:
        sys.exit(1)


# =========================================================================== #
# PRINT
# =========================================================================== #
def cmd_print(args):
    summary_file = args.summary or ALL_SUMMARY
    trials = {}
    try:
        with open(summary_file) as f:
            for row in csv.DictReader(f):
                trials[int(row['trial_id'])] = row
    except FileNotFoundError:
        print(f'파일 없음: {summary_file}'); return
    if not trials:
        print('데이터 없음'); return

    header = (f'{"ID":>5}  {"결과":^12}  {"xy_err(m)":>9}  '
              f'{"yaw_err(°)":>10}  {"t(s)":>6}  datetime')
    print(header)
    print('-' * len(header))
    for tid, t in sorted(trials.items()):
        yaw_deg = math.degrees(float(t['yaw_error_rad']))
        print(f'{tid:>05d}  {t["result"]:^12}  '
              f'{float(t["xy_error_m"]):>9.3f}  '
              f'{yaw_deg:>10.1f}  '
              f'{float(t["duration_sec"]):>6.1f}  '
              f'{t["datetime"]}')

    errors = [float(t['xy_error_m']) for t in trials.values()]
    n_suc  = sum(1 for t in trials.values() if t['result'] == 'SUCCESS')
    print()
    print(f'  성공률: {n_suc}/{len(trials)} | '
          f'오차 평균: {sum(errors)/len(errors):.3f} m | '
          f'최대: {max(errors):.3f} m | 최소: {min(errors):.3f} m')


# =========================================================================== #
# MAIN
# =========================================================================== #
def main():
    parser = argparse.ArgumentParser(
        description='Nav2 시험 기록 및 시각화 도구',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    sub = parser.add_subparsers(dest='cmd', required=True)

    # record
    sub.add_parser('record', help='ROS2 기록 노드 실행 (종료 시 자동 시각화)')

    # visualize
    p_vis = sub.add_parser('visualize', help='경로 이미지 생성')
    p_vis.add_argument('--dir',     help='시험 폴더 경로 (예: ~/nav_trials/003)')
    p_vis.add_argument('--summary', help='summary CSV 경로 (--dir 없을 때)')
    p_vis.add_argument('--paths',   help='path CSV 경로    (--dir 없을 때)')
    p_vis.add_argument('--output',  help='출력 PNG 경로')
    p_vis.add_argument('--map',     default=DEFAULT_MAP, help='map.yaml 경로')
    p_vis.add_argument('--trials',  nargs='+', type=int, help='특정 trial_id 만')

    # print
    p_pr = sub.add_parser('print', help='터미널 요약 테이블')
    p_pr.add_argument('--summary', help='summary CSV 경로 (기본: all_summary.csv)')

    args = parser.parse_args()
    if args.cmd == 'record':
        cmd_record(args)
    elif args.cmd == 'visualize':
        cmd_visualize(args)
    elif args.cmd == 'print':
        cmd_print(args)


if __name__ == '__main__':
    main()
