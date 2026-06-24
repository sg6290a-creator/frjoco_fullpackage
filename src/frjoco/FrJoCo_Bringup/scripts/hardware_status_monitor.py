#!/usr/bin/env python3
import math
import os
from typing import Dict, Optional, Tuple

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger


RESET = '\033[0m'
GREEN = '\033[92m'
RED = '\033[91m'
CYAN = '\033[96m'
MONITOR_VERSION = 'fullpackage-ox-only-2026-06-02'


def mark(ok: bool) -> str:
    if ok is True:
        return f'{GREEN}O{RESET}'
    return f'{RED}X{RESET}'


class HardwareStatusMonitor(Node):
    ARM_JOINTS = (
        ('J1', 'shoulder_pan_joint'),
        ('J2', 'shoulder_lift_joint'),
        ('J3', 'elbow_joint'),
        ('J4', 'wrist_1_joint'),
        ('J5', 'wrist_2_joint'),
        ('J6', 'wrist_3_joint'),
    )

    def __init__(self):
        super().__init__('hardware_status_monitor')
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('color_topic', '/d405/color/image_raw')
        self.declare_parameter('depth_topic', '/d405/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/d405/aligned_depth_to_color/camera_info')
        self.declare_parameter('yolo_image_topic', '/yolo_seg/detection_image')
        self.declare_parameter('yolo_target_topic', '/yolo_seg/target_info')
        self.declare_parameter('thin_part_service', '/thin_part/estimate')
        self.declare_parameter('execute_service', '/execute_pick_place')
        self.declare_parameter('check_period_sec', 2.0)
        self.declare_parameter('stale_after_sec', 3.0)
        self.declare_parameter('force_print_sec', 10.0)

        self.can_interface = str(self.get_parameter('can_interface').value)
        self.color_topic = str(self.get_parameter('color_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.yolo_image_topic = str(self.get_parameter('yolo_image_topic').value)
        self.yolo_target_topic = str(self.get_parameter('yolo_target_topic').value)
        self.thin_part_service = str(self.get_parameter('thin_part_service').value)
        self.execute_service = str(self.get_parameter('execute_service').value)
        self.stale_after_sec = float(self.get_parameter('stale_after_sec').value)
        self.force_print_sec = float(self.get_parameter('force_print_sec').value)

        self.controllers: Dict[str, str] = {}
        self.controllers_ready = False
        self.controller_request_pending = False
        self.last_joint_positions: Dict[str, float] = {}
        self.last_joint_time: Optional[float] = None
        self.last_topic_times: Dict[str, float] = {}
        self.last_signature: Optional[Tuple] = None
        self.last_print_time = 0.0

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(Image, self.color_topic, self._topic_cb('color'), 10)
        self.create_subscription(Image, self.depth_topic, self._topic_cb('depth'), 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._topic_cb('camera_info'), 10)
        self.create_subscription(Image, self.yolo_image_topic, self._topic_cb('yolo_image'), 10)
        self.create_subscription(Float32MultiArray, self.yolo_target_topic, self._topic_cb('yolo_target'), 10)

        self.controller_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )
        self.thin_part_client = self.create_client(Trigger, self.thin_part_service)
        self.execute_client = self.create_client(Trigger, self.execute_service)

        period = float(self.get_parameter('check_period_sec').value)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'Manipulator/Vision OX monitor started. '
            f'version={MONITOR_VERSION} file={__file__}'
        )

    def _topic_cb(self, key: str):
        def _cb(_msg):
            self.last_topic_times[key] = self._now_sec()

        return _cb

    def _joint_state_cb(self, msg: JointState):
        positions: Dict[str, float] = {}
        for idx, name in enumerate(msg.name):
            if idx < len(msg.position):
                positions[name] = float(msg.position[idx])

        self.last_joint_positions = positions
        self.last_joint_time = self._now_sec()

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _tick(self):
        self._request_controllers()

        snapshot = self._snapshot()
        signature = tuple((item[0], item[1], item[2]) for item in snapshot)
        now = self._now_sec()
        force = (now - self.last_print_time) >= self.force_print_sec
        if signature == self.last_signature and not force:
            return

        self.last_signature = signature
        self.last_print_time = now
        self._print_snapshot(snapshot)

    def _request_controllers(self):
        if self.controller_request_pending:
            return
        if not self.controller_client.service_is_ready():
            self.controller_client.wait_for_service(timeout_sec=0.0)
            self.controllers_ready = False
            return

        self.controller_request_pending = True
        future = self.controller_client.call_async(ListControllers.Request())
        future.add_done_callback(self._controllers_done_cb)

    def _controllers_done_cb(self, future):
        self.controller_request_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.controllers_ready = False
            self.get_logger().warn(f'controller 상태 조회 실패: {exc}')
            return

        self.controllers = {ctrl.name: ctrl.state for ctrl in response.controller}
        self.controllers_ready = True

    def _controller_active(self, name: str) -> bool:
        return self.controllers.get(name) == 'active'

    def _can_exists(self) -> bool:
        return os.path.exists(os.path.join('/sys/class/net', self.can_interface))

    def _fresh_time(self, timestamp: Optional[float]) -> bool:
        if timestamp is None:
            return False
        return (self._now_sec() - timestamp) <= self.stale_after_sec

    def _joint_state_fresh(self) -> bool:
        return self._fresh_time(self.last_joint_time)

    def _topic_fresh(self, key: str) -> bool:
        return self._fresh_time(self.last_topic_times.get(key))

    def _joint_ok(self, joint_name: str) -> Tuple[bool, str]:
        can_ok = self._can_exists()
        jsb = self._controller_active('joint_state_broadcaster')
        arm_ctrl = self._controller_active('joint_trajectory_controller')
        position = self.last_joint_positions.get(joint_name)
        has_position = position is not None and math.isfinite(position)
        fresh = self._joint_state_fresh()

        if has_position:
            value = f'pos={position:+.3f}rad'
        else:
            value = 'pos=없음'

        controller_detail = (
            self.controllers.get('joint_trajectory_controller', '대기')
            if self.controllers_ready
            else '조회대기'
        )

        # O/X only: controller 조회 전이거나 joint_state가 아직 없으면 X로 표시한다.
        ok = can_ok and jsb and arm_ctrl and fresh and has_position

        detail = (
            f'can={self.can_interface} '
            f'controller={controller_detail} '
            f'joint_state={"수신" if fresh and has_position else "미수신"} '
            f'{value}'
        )
        return ok, detail

    def _service_ok(self, client) -> bool:
        client.wait_for_service(timeout_sec=0.0)
        return client.service_is_ready()

    def _snapshot(self):
        snapshot = [
            (
                'CAN',
                self._can_exists(),
                f'{self.can_interface} {"존재" if self._can_exists() else "없음"}',
            )
        ]

        for label, joint_name in self.ARM_JOINTS:
            ok, detail = self._joint_ok(joint_name)
            snapshot.append((f'{label} {joint_name}', ok, detail))

        snapshot.extend([
            (
                '카메라 컬러',
                self._topic_fresh('color'),
                f'{self.color_topic} 최근 수신={"예" if self._topic_fresh("color") else "아니오"}',
            ),
            (
                '카메라 Depth',
                self._topic_fresh('depth'),
                f'{self.depth_topic} 최근 수신={"예" if self._topic_fresh("depth") else "아니오"}',
            ),
            (
                '카메라 Info',
                self._topic_fresh('camera_info'),
                f'{self.camera_info_topic} 최근 수신={"예" if self._topic_fresh("camera_info") else "아니오"}',
            ),
            (
                'YOLO Seg',
                self._topic_fresh('yolo_image'),
                f'{self.yolo_image_topic} 최근 수신={"예" if self._topic_fresh("yolo_image") else "아니오"}',
            ),
            (
                '객체 검출',
                self._topic_fresh('yolo_target'),
                f'{self.yolo_target_topic} 최근 수신={"예" if self._topic_fresh("yolo_target") else "아니오"}',
            ),
            (
                '파지 계산',
                self._service_ok(self.thin_part_client),
                f'{self.thin_part_service} service {"준비" if self._service_ok(self.thin_part_client) else "대기"}',
            ),
            (
                '실행 Bridge',
                self._service_ok(self.execute_client),
                f'{self.execute_service} service {"준비" if self._service_ok(self.execute_client) else "대기"}',
            ),
        ])
        return snapshot

    @staticmethod
    def _print_snapshot(snapshot):
        print(f'\n{CYAN}========== Manipulator / Vision OX =========={RESET}', flush=True)
        for label, ok, detail in snapshot:
            print(f'{mark(ok)} {label:<24} | {detail}', flush=True)
        print(f'{CYAN}============================================={RESET}\n', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
