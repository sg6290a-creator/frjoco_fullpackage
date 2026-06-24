#!/usr/bin/env python3
"""
sim_robot — Lightweight differential-drive simulator (no Gazebo)

기능
  /diff_drive_controller/cmd_vel_unstamped (Twist) 를 받아
  단순 unicycle 운동학으로 pose 적분 → 다음을 publish:
    - /diff_drive_controller/odom   (nav_msgs/Odometry)
    - TF  odom → base_footprint
    - TF  map  → odom               (identity, sim 용 localization 가정)

사용 시나리오
  실 센서 / RTAB-Map / 모터 없이 4F 정적 지도 위에서
  nav2 의 path planning + RPP follow 동작만 검증하고 싶을 때.

파라미터
  ~init_x, ~init_y, ~init_yaw   초기 spawn 위치 (map frame, m / rad)
  ~rate                          적분 주기 (Hz, 기본 50)
  ~cmd_topic                     구독 cmd_vel 토픽
  ~odom_topic                    publish 할 odom 토픽
  ~odom_frame, ~base_frame, ~map_frame
  ~publish_map_odom_tf           True 면 map→odom identity TF 도 발행
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class SimRobot(Node):
    def __init__(self):
        super().__init__('sim_robot')

        self.declare_parameter('init_x',   0.0)
        self.declare_parameter('init_y',   0.0)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('rate',     50.0)
        self.declare_parameter('cmd_topic',  '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('odom_topic', '/diff_drive_controller/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame',  'map')
        self.declare_parameter('publish_map_odom_tf', True)
        self.declare_parameter('cmd_timeout', 0.5)  # 명령 끊기면 정지

        self.x   = float(self.get_parameter('init_x').value)
        self.y   = float(self.get_parameter('init_y').value)
        self.yaw = float(self.get_parameter('init_yaw').value)
        rate     = float(self.get_parameter('rate').value)
        cmd_topic  = self.get_parameter('cmd_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame  = self.get_parameter('map_frame').value
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.vx   = 0.0
        self.vyaw = 0.0
        self.last_cmd_t = self.get_clock().now()
        self.last_step_t = self.get_clock().now()

        self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        self._tf  = TransformBroadcaster(self)
        self._stf = StaticTransformBroadcaster(self)

        # map → odom identity (sim 한정 — 실제로는 localizer 가 발행)
        if bool(self.get_parameter('publish_map_odom_tf').value):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame
            t.child_frame_id  = self.odom_frame
            t.transform.rotation.w = 1.0
            self._stf.sendTransform(t)
            self.get_logger().info(
                f'[sim_robot] map→odom identity TF 발행 ({self.map_frame}→{self.odom_frame})')

        self.create_timer(1.0 / rate, self._step)

        self.get_logger().info(
            f'[sim_robot] spawn at ({self.x:.2f}, {self.y:.2f}, '
            f'{math.degrees(self.yaw):.1f}°)  rate={rate}Hz')
        self.get_logger().info(
            f'[sim_robot] sub: {cmd_topic}   pub: {odom_topic} + TF')

    def _on_cmd(self, msg: Twist):
        self.vx   = msg.linear.x
        self.vyaw = msg.angular.z
        self.last_cmd_t = self.get_clock().now()

    def _step(self):
        now = self.get_clock().now()
        dt = (now - self.last_step_t).nanoseconds * 1e-9
        self.last_step_t = now
        if dt <= 0.0 or dt > 0.5:
            return  # 비정상 dt 무시

        # 명령 timeout 시 정지
        if (now - self.last_cmd_t).nanoseconds * 1e-9 > self.cmd_timeout:
            self.vx = 0.0
            self.vyaw = 0.0

        # 단순 unicycle 적분 (mid-yaw 사용해 작은 dt 보정)
        dyaw = self.vyaw * dt
        mid_yaw = self.yaw + dyaw * 0.5
        self.x += self.vx * math.cos(mid_yaw) * dt
        self.y += self.vx * math.sin(mid_yaw) * dt
        self.yaw += dyaw

        stamp = now.to_msg()
        q = yaw_to_quat(self.yaw)

        # TF: odom → base_footprint
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id  = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = q
        self._tf.sendTransform(tf)

        # /odom
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id  = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation = q
        od.twist.twist.linear.x = self.vx
        od.twist.twist.angular.z = self.vyaw
        # 작은 covariance — nav2 가 쓰지 않더라도 채워둠
        for i in (0, 7, 14, 21, 28, 35):
            od.pose.covariance[i]  = 0.001
            od.twist.covariance[i] = 0.001
        self._odom_pub.publish(od)


def main():
    rclpy.init()
    node = SimRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
