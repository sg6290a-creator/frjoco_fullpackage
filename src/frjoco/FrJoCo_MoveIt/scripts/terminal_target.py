#!/usr/bin/env python3
"""
terminal_target.py
터미널 입력 → PoseStamped(/target_pose) 발행 → vision_move.py 가 계획·실행

입력 형식:
  x y z    목표 EE 위치 (방향은 vision_move.py 가 자동 look-at 계산)
  q        종료

좌표 기준: base_link (기본값)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty


class TerminalTarget(Node):
    def __init__(self):
        super().__init__('terminal_target')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('topic',    '/target_pose')

        self.frame_id = self.get_parameter('frame_id').value
        topic         = self.get_parameter('topic').value

        self.pub      = self.create_publisher(PoseStamped, topic, 1)
        self.home_pub = self.create_publisher(Empty, '/go_home', 1)

        print(
            f'\n목표 입력 (frame={self.frame_id}, topic={topic})\n'
            '  x y z    → EE 위치 이동 (방향 자동 계산)\n'
            '  home     → 홈 포즈로 이동\n'
            '  q        → 종료\n'
        )

    def run(self):
        import time
        time.sleep(0.5)

        while rclpy.ok():
            try:
                raw = input('> ').strip()
            except EOFError:
                break

            if raw.lower() in ('q', 'quit', 'exit'):
                break

            if raw.lower() == 'home':
                self.home_pub.publish(Empty())
                print('  → 홈 포즈로 이동')
                continue

            parts = raw.split()
            if len(parts) != 3:
                print('  입력: x y z')
                continue

            try:
                x, y, z = map(float, parts)
            except ValueError:
                print('  숫자 변환 실패')
                continue

            msg = PoseStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.w = 1.0  # vision_move.py 가 look-at 으로 덮어씀

            self.pub.publish(msg)
            print(f'  → 목표: ({x:.3f}, {y:.3f}, {z:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = TerminalTarget()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
