#!/usr/bin/env python3
"""
sign_test.py — 바퀴 부호 확인용 미세 이동 테스트

사용법:
  python3 sign_test.py          # 대화형 메뉴
  python3 sign_test.py forward  # 직접 실행

mobile_teleop.launch.py 가 먼저 떠 있어야 합니다.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

TOPIC   = '/diff_drive_controller/cmd_vel_unstamped'
DURATION = 0.6   # 각 명령 지속 시간 (초) — 너무 짧으면 모터가 반응 못 함
LINEAR  = 0.015  # m/s — 매우 느리게
ANGULAR = 0.03   # rad/s


TESTS = {
    'forward':  ( LINEAR,  0.0,      '앞으로  (+linear.x)'),
    'backward': (-LINEAR,  0.0,      '뒤로    (-linear.x)'),
    'left':     ( 0.0,     ANGULAR,  '좌회전  (+angular.z)  : 왼쪽=후진, 오른쪽=전진'),
    'right':    ( 0.0,    -ANGULAR,  '우회전  (-angular.z)  : 왼쪽=전진, 오른쪽=후진'),
}


class SignTest(Node):
    def __init__(self):
        super().__init__('sign_test')
        self._pub = self.create_publisher(Twist, TOPIC, 10)
        time.sleep(0.3)  # publisher 등록 대기

    def send(self, linear_x: float, angular_z: float, duration: float):
        msg = Twist()
        msg.linear.x  = linear_x
        msg.angular.z = angular_z

        t_end = time.time() + duration
        while time.time() < t_end:
            self._pub.publish(msg)
            time.sleep(0.05)

        # 정지
        self._pub.publish(Twist())
        time.sleep(0.3)

    def run(self, test_key: str):
        lx, az, desc = TESTS[test_key]
        self.get_logger().info(f'[{test_key}] {desc}')
        self.get_logger().info(
            f'  linear.x={lx:+.3f}  angular.z={az:+.3f}  t={DURATION}s')
        self.send(lx, az, DURATION)
        self.get_logger().info('  완료 — 로봇 이동 방향 확인하세요.')


def interactive(node: SignTest):
    menu = '\n'.join([
        '',
        '=== 부호 테스트 메뉴 ===',
        *[f'  {k:<10} : {v[2]}' for k, v in TESTS.items()],
        '  all      : 순서대로 모두 실행 (사이 3초 대기)',
        '  q        : 종료',
        '',
    ])

    while True:
        print(menu, end='')
        key = input('선택 > ').strip().lower()
        if key == 'q':
            break
        elif key == 'all':
            for k in TESTS:
                node.run(k)
                print('  3초 대기...')
                time.sleep(3.0)
        elif key in TESTS:
            node.run(key)
        else:
            print('  알 수 없는 명령입니다.')


def main():
    rclpy.init()
    node = SignTest()

    try:
        if len(sys.argv) > 1:
            key = sys.argv[1].lower()
            if key not in TESTS:
                print(f'알 수 없는 명령: {key}')
                print('사용 가능:', ', '.join(TESTS.keys()))
                sys.exit(1)
            node.run(key)
        else:
            interactive(node)
    except KeyboardInterrupt:
        node._pub.publish(Twist())  # 비상 정지
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
