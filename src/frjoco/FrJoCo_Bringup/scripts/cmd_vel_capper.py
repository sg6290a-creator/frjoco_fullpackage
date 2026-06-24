#!/usr/bin/env python3
"""
cmd_vel_capper — /cmd_vel 의 linear.x / angular.z 를 안전 한도 내로 제한해서
                 /diff_drive_controller/cmd_vel_unstamped 로 발행.

목적:
  Nav2 controller_server (RPP) 는 자체적으로 desired_linear_vel 등을 지키지만,
  behavior_server (BackUp / Spin / DriveOnHeading) 의 출력은
  velocity_smoother 를 우회해서 /cmd_vel 에 직접 publish 한다.
  이 때문에 stuck 상황에서 BackUp 이 0.025 m/s 같은 빠른 후진을 명령해
  실제 로봇이 갑자기 빠르게 후진하는 사고가 발생.

  본 노드는 source 와 무관하게 /cmd_vel 의 모든 명령을 강제로 clip 한 뒤
  하드웨어 토픽으로 forward → 어떤 nav2 컴포넌트가 보내든 한계 속도 보장.

  topic_tools/relay 를 대체.

파라미터:
  ~max_linear   (double, 0.01)
  ~max_angular  (double, 0.02)
  ~in_topic     (str, '/cmd_vel')
  ~out_topic    (str, '/diff_drive_controller/cmd_vel_unstamped')
  ~log_clipped  (bool, True)   clip 발생 시 throttle 로그
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelCapper(Node):
    def __init__(self):
        super().__init__('cmd_vel_capper')

        self.declare_parameter('max_linear',  0.01)
        self.declare_parameter('max_angular', 0.02)
        self.declare_parameter('in_topic',    '/cmd_vel')
        self.declare_parameter('out_topic',   '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('log_clipped', True)

        self.max_lin = float(self.get_parameter('max_linear').value)
        self.max_ang = float(self.get_parameter('max_angular').value)
        in_topic     = self.get_parameter('in_topic').value
        out_topic    = self.get_parameter('out_topic').value
        self.log_clip = bool(self.get_parameter('log_clipped').value)

        self._pub = self.create_publisher(Twist, out_topic, 10)
        self._sub = self.create_subscription(Twist, in_topic, self._on_cmd, 10)

        self.get_logger().info(
            f'[cmd_vel_capper] {in_topic} → {out_topic} | '
            f'|lin|≤{self.max_lin}  |ang|≤{self.max_ang}')

    @staticmethod
    def _clip(v: float, lim: float) -> (float, bool):
        if   v >  lim: return  lim, True
        elif v < -lim: return -lim, True
        return v, False

    def _on_cmd(self, msg: Twist):
        lx, lin_clipped = self._clip(msg.linear.x,  self.max_lin)
        az, ang_clipped = self._clip(msg.angular.z, self.max_ang)

        out = Twist()
        out.linear.x  = lx
        out.angular.z = az
        # linear.y, linear.z, angular.x, angular.y → diff drive 에서 무시되므로 0 유지
        self._pub.publish(out)

        if self.log_clip and (lin_clipped or ang_clipped):
            self.get_logger().warn(
                f'CLIPPED  in=(lin={msg.linear.x:+.4f}, ang={msg.angular.z:+.4f}) '
                f'out=(lin={lx:+.4f}, ang={az:+.4f})',
                throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = CmdVelCapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
