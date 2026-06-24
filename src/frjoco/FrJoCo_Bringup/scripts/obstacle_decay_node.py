#!/usr/bin/env python3
"""
Global costmap 주기적 리셋 — 초기 포즈 오차로 생긴 가짜 장애물 소멸용

nav2_params.yaml 의 observation_persistence(10s) 와 짝을 이룸:
  리셋 → static_layer 즉시 재구독 → obstacle_layer 가 최근 10s 스캔만으로 재마킹
  → 10초 이상 탐지 안 된 장애물은 사라짐.

local_costmap 는 rolling_window 가 자연 소멸시키므로 건드리지 않음.
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap


DECAY_SECONDS = 10.0   # nav2_params.yaml observation_persistence 와 맞출 것


class ObstacleDecayNode(Node):
    def __init__(self):
        super().__init__('obstacle_decay_node')

        self._cli = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )

        self.create_timer(DECAY_SECONDS, self._on_timer)
        self.get_logger().info(
            f'obstacle_decay_node 시작 — global costmap 을 {DECAY_SECONDS:.0f}초마다 리셋합니다.'
        )

    def _on_timer(self):
        if not self._cli.service_is_ready():
            self.get_logger().warn('clear_entirely 서비스 아직 미준비 — 다음 주기에 재시도')
            return

        future = self._cli.call_async(ClearEntireCostmap.Request())
        future.add_done_callback(self._on_done)

    def _on_done(self, future):
        try:
            future.result()
            self.get_logger().debug('global costmap 리셋 완료')
        except Exception as e:
            self.get_logger().warn(f'global costmap 리셋 실패: {e}')


def main():
    rclpy.init()
    node = ObstacleDecayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
