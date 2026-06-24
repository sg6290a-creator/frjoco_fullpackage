#!/usr/bin/env python3
"""
Triggered thin-part grasp estimator node.

Terminal role
-------------
Terminal 3: run this node and press g + Enter to calculate grasp coordinate/angle
from the latest YOLO-seg mask, target info, aligned depth, and camera info.

What it does
------------
- Subscribes /yolo_seg/target_mask and /yolo_seg/target_info from yolo26_seg_mask_publisher_node.py.
- Subscribes aligned depth and camera intrinsics.
- Does NOT run YOLO.
- Only when triggered, computes:
  thin_center_pixel, thin_angle_deg, thin_width_px, depth_m, XYZ point.
- Publishes the same class-specific PointStamped topics plus /thin_part/grasp_info
  containing angle and debug values.

Trigger methods
---------------
1. Keyboard in this terminal: press g + Enter, t + Enter, or just Enter.
2. ROS2 service: ros2 service call /thin_part/estimate std_srvs/srv/Trigger {}

Published /thin_part/grasp_info layout
--------------------------------------
[
  class_id, confidence,
  bbox_x1, bbox_y1, bbox_x2, bbox_y2,
  thin_u, thin_v,
  thin_angle_deg, thin_width_px,
  object_axis_deg, quality_score,
  depth_m,
  x_m, y_m, z_m,
  mask_area_px,
  safe_width_score, selection_mode
]
"""

import math
import threading
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class ThinPartGraspTriggerNode(Node):
    def __init__(self):
        super().__init__('thin_part_grasp_trigger_node')

        # Input topics
        self.declare_parameter('mask_topic', '/yolo_seg/target_mask')
        self.declare_parameter('target_info_topic', '/yolo_seg/target_info')
        self.declare_parameter('color_topic', '/d405/color/image_raw')
        self.declare_parameter('depth_topic', '/d405/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/d405/aligned_depth_to_color/camera_info')
        self.declare_parameter('camera_frame', 'd405_optical_frame')

        # Runtime options
        self.declare_parameter('enable_keyboard_trigger', True)
        self.declare_parameter('show_visualization', False)

        # Depth filtering
        self.declare_parameter('depth_roi_half_size', 3)
        self.declare_parameter('min_depth_m', 0.01)
        self.declare_parameter('max_depth_m', 2.0)

        # Thin-part extraction
        self.declare_parameter('mask_downsample', 0.5)
        self.declare_parameter('thin_margin_ratio', 0.15)
        self.declare_parameter('smooth_window', 7)
        self.declare_parameter('min_mask_area_px', 80)
        self.declare_parameter('min_valid_columns', 8)

        # Safe grasp filtering
        self.declare_parameter('min_safe_width_px', 8.0)
        self.declare_parameter('max_safe_width_px', 0.0)
        self.declare_parameter('target_width_percentile', 15.0)
        self.declare_parameter('reject_unsafe_thin_part', True)
        self.declare_parameter('min_width_contrast', 0.05)

        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.enable_keyboard_trigger = bool(self.get_parameter('enable_keyboard_trigger').value)
        self.show_visualization = bool(self.get_parameter('show_visualization').value)

        self.depth_roi_half_size = int(self.get_parameter('depth_roi_half_size').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)

        self.mask_downsample = float(self.get_parameter('mask_downsample').value)
        self.thin_margin_ratio = float(self.get_parameter('thin_margin_ratio').value)
        self.smooth_window = int(self.get_parameter('smooth_window').value)
        self.min_mask_area_px = int(self.get_parameter('min_mask_area_px').value)
        self.min_valid_columns = int(self.get_parameter('min_valid_columns').value)

        self.min_safe_width_px = float(self.get_parameter('min_safe_width_px').value)
        self.max_safe_width_px = float(self.get_parameter('max_safe_width_px').value)
        self.target_width_percentile = float(self.get_parameter('target_width_percentile').value)
        self.reject_unsafe_thin_part = bool(self.get_parameter('reject_unsafe_thin_part').value)
        self.min_width_contrast = float(self.get_parameter('min_width_contrast').value)

        self.lock = threading.Lock()

        # Latest state
        self.latest_mask_msg: Optional[Image] = None
        self.latest_target_info: Optional[np.ndarray] = None
        self.latest_depth_msg: Optional[Image] = None
        self.latest_color_msg: Optional[Image] = None
        self.intrinsics_received = False
        self.fx = self.fy = self.cx = self.cy = None

        # Class-specific point topics for current labels:
        # 0=hammer, 1=pliers, 2=screwdriver.
        self.hammer_pub = self.create_publisher(PointStamped, 'hammer_target_point', 10)
        self.pliers_pub = self.create_publisher(PointStamped, 'pliers_target_point', 10)
        self.screwdriver_pub = self.create_publisher(PointStamped, 'screwdriver_target_point', 10)
        self.pub_map: Dict[int, object] = {0: self.hammer_pub, 1: self.pliers_pub, 2: self.screwdriver_pub}

        self.generic_point_pub = self.create_publisher(PointStamped, '/thin_part/grasp_point', 10)
        self.grasp_info_pub = self.create_publisher(Float32MultiArray, '/thin_part/grasp_info', 10)
        self.debug_image_pub = self.create_publisher(Image, '/thin_part/debug_image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/thin_part/grasp_markers', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.mask_sub = self.create_subscription(Image, self.get_parameter('mask_topic').value, self.mask_callback, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(Float32MultiArray, self.get_parameter('target_info_topic').value, self.target_info_callback, qos)
        self.color_sub = self.create_subscription(Image, self.get_parameter('color_topic').value, self.color_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(Image, self.get_parameter('depth_topic').value, self.depth_callback, qos_profile_sensor_data)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self.camera_info_callback, qos_profile_sensor_data)

        self.trigger_srv = self.create_service(Trigger, '/thin_part/estimate', self.trigger_callback)

        if self.enable_keyboard_trigger:
            self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
            self.keyboard_thread.start()

        self.get_logger().info(
            'Thin-Part Grasp Trigger Node Ready. '
            f"color={self.get_parameter('color_topic').value}, "
            f"depth={self.get_parameter('depth_topic').value}, "
            f"camera_info={self.get_parameter('camera_info_topic').value}"
        )

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------
    @staticmethod
    def image_msg_to_array(msg: Image, dtype: np.dtype, channels: int) -> np.ndarray:
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)
        itemsize = np.dtype(dtype).itemsize
        data = np.frombuffer(msg.data, dtype=dtype)
        row_items = width * channels

        if step and step >= row_items * itemsize:
            step_items = step // itemsize
            image = data.reshape((height, step_items))[:, :row_items]
            if channels > 1:
                image = image.reshape((height, width, channels))
        else:
            shape = (height, width, channels) if channels > 1 else (height, width)
            image = data.reshape(shape)
        return image.copy()

    @classmethod
    def image_msg_to_bgr(cls, msg: Image) -> np.ndarray:
        encoding = (msg.encoding or '').lower()
        if encoding in ('bgr8', 'rgb8'):
            image = cls.image_msg_to_array(msg, np.uint8, 3)
        elif encoding in ('bgra8', 'rgba8'):
            image = cls.image_msg_to_array(msg, np.uint8, 4)
        elif encoding in ('mono8', '8uc1'):
            image = cls.image_msg_to_array(msg, np.uint8, 1)
        else:
            raise ValueError(f'Unsupported color image encoding: {msg.encoding}')

        if encoding == 'rgb8':
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if encoding in ('mono8', '8uc1'):
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return image

    @classmethod
    def image_msg_to_mono(cls, msg: Image) -> np.ndarray:
        encoding = (msg.encoding or '').lower()
        if encoding in ('mono8', '8uc1'):
            return cls.image_msg_to_array(msg, np.uint8, 1)
        return cv2.cvtColor(cls.image_msg_to_bgr(msg), cv2.COLOR_BGR2GRAY)

    @classmethod
    def image_msg_to_depth(cls, msg: Image) -> np.ndarray:
        encoding = (msg.encoding or '').lower()
        if encoding in ('16uc1', 'mono16'):
            return cls.image_msg_to_array(msg, np.uint16, 1)
        if encoding == '32fc1':
            return cls.image_msg_to_array(msg, np.float32, 1)
        raise ValueError(f'Unsupported depth image encoding: {msg.encoding}')

    @staticmethod
    def bgr_to_image_msg(image: np.ndarray, header) -> Image:
        image = np.ascontiguousarray(image, dtype=np.uint8)
        msg = Image()
        msg.header = header
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = int(image.shape[1] * 3)
        msg.data = image.tobytes()
        return msg

    def mask_callback(self, msg: Image):
        with self.lock:
            self.latest_mask_msg = msg

    def target_info_callback(self, msg: Float32MultiArray):
        with self.lock:
            self.latest_target_info = np.array(msg.data, dtype=np.float32)

    def color_callback(self, msg: Image):
        with self.lock:
            self.latest_color_msg = msg

    def depth_callback(self, msg: Image):
        with self.lock:
            self.latest_depth_msg = msg

    def camera_info_callback(self, msg: CameraInfo):
        with self.lock:
            if not self.intrinsics_received:
                self.fx = float(msg.k[0])
                self.fy = float(msg.k[4])
                self.cx = float(msg.k[2])
                self.cy = float(msg.k[5])
                self.intrinsics_received = True
                self.get_logger().info(
                    f'Camera intrinsics received: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}'
                )

    # ------------------------------------------------------------------
    # Trigger entrypoints
    # ------------------------------------------------------------------
    def keyboard_loop(self):
        print('\n[thin_part_grasp_trigger_node] Press g + Enter to estimate. Press q + Enter to ignore/quit keyboard loop.\n')
        while rclpy.ok():
            try:
                text = input('[trigger] g/Enter=estimate, q=quit input > ').strip().lower()
            except EOFError:
                return
            except KeyboardInterrupt:
                return

            if text in ('q', 'quit', 'exit'):
                self.get_logger().info('Keyboard trigger loop stopped. ROS node still running.')
                return
            if text in ('', 'g', 't', 'trigger'):
                ok, message = self.estimate_and_publish()
                if ok:
                    self.get_logger().info(message)
                else:
                    self.get_logger().warn(message)

    def trigger_callback(self, request, response):
        ok, message = self.estimate_and_publish()
        response.success = bool(ok)
        response.message = str(message)
        return response

    # ------------------------------------------------------------------
    # Core computation
    # ------------------------------------------------------------------
    def estimate_and_publish(self) -> Tuple[bool, str]:
        with self.lock:
            mask_msg = self.latest_mask_msg
            target_info = None if self.latest_target_info is None else self.latest_target_info.copy()
            depth_msg = self.latest_depth_msg
            color_msg = self.latest_color_msg
            intrinsics_ok = self.intrinsics_received
            fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy

        if mask_msg is None:
            return False, 'No target mask received yet from /yolo_seg/target_mask.'
        if target_info is None or target_info.size < 7:
            return False, 'No valid target info received yet from /yolo_seg/target_info.'
        if depth_msg is None:
            return False, 'No aligned depth image received yet.'
        if not intrinsics_ok:
            return False, 'No camera intrinsics received yet.'

        try:
            mask = self.image_msg_to_mono(mask_msg)
        except Exception as exc:
            return False, f'Mask conversion failed: {exc}'

        try:
            depth_raw = self.image_msg_to_depth(depth_msg)
            depth_array = depth_raw.astype(np.float32) / (1000.0 if depth_raw.dtype == np.uint16 else 1.0)
        except Exception as exc:
            return False, f'Depth conversion failed: {exc}'

        color_image = None
        if color_msg is not None:
            try:
                color_image = self.image_msg_to_bgr(color_msg)
            except Exception:
                color_image = None

        cls_id = int(target_info[0])
        conf = float(target_info[1])
        bbox = target_info[2:6].astype(float)
        mask_area_from_info = float(target_info[6])

        thin = self.find_thin_part_safe(mask)
        if thin is None:
            return False, 'Thin-part estimation failed or rejected by safety filters.'

        thin_u = float(thin['thin_center'][0])
        thin_v = float(thin['thin_center'][1])
        thin_angle_deg = float(thin['thin_angle_deg'])
        thin_width_px = float(thin['thin_width_px'])
        object_axis_deg = float(thin['object_axis_deg'])
        quality_score = float(thin['quality_score'])
        safe_width_score = float(thin['safe_width_score'])
        selection_mode = float(thin['selection_mode'])

        depth = self.get_median_depth_around_pixel(depth_array, int(round(thin_u)), int(round(thin_v)))
        if depth is None:
            return False, f'No valid depth around thin center ({thin_u:.1f}, {thin_v:.1f}).'

        point_3d = self.deproject_pixel_to_point(thin_u, thin_v, depth, fx, fy, cx, cy)
        if point_3d is None:
            return False, '3D deprojection failed.'

        # Publish PointStamped: class-specific topic plus generic grasp point.
        header = mask_msg.header
        point_msg = PointStamped()
        point_msg.header = header
        point_msg.header.frame_id = self.camera_frame
        point_msg.point.x = float(point_3d[0])
        point_msg.point.y = float(point_3d[1])
        point_msg.point.z = float(point_3d[2])

        self.generic_point_pub.publish(point_msg)
        if cls_id in self.pub_map:
            self.pub_map[cls_id].publish(point_msg)

        info_msg = Float32MultiArray()
        info_msg.data = [
            float(cls_id), float(conf),
            float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]),
            thin_u, thin_v,
            thin_angle_deg, thin_width_px,
            object_axis_deg, quality_score,
            float(depth),
            float(point_3d[0]), float(point_3d[1]), float(point_3d[2]),
            float(mask_area_from_info),
            safe_width_score, selection_mode,
        ]
        self.grasp_info_pub.publish(info_msg)

        marker_array = MarkerArray()
        marker_array.markers.append(self.create_marker(point_3d, header, 0, cls_id))
        self.marker_pub.publish(marker_array)

        if color_image is not None:
            debug = self.draw_debug_image(color_image, mask, bbox, thin, point_3d, depth, cls_id, conf)
            debug_msg = self.bgr_to_image_msg(debug, header)
            self.debug_image_pub.publish(debug_msg)
            if self.show_visualization:
                cv2.imshow('Thin-Part Grasp Debug', debug)
                cv2.waitKey(1)

        message = (
            f'grasp cls={cls_id}, conf={conf:.2f}, '
            f'uv=({thin_u:.1f},{thin_v:.1f}), '
            f'xyz=({point_3d[0]:.3f},{point_3d[1]:.3f},{point_3d[2]:.3f})m, '
            f'angle={thin_angle_deg:.1f}deg, width={thin_width_px:.1f}px, quality={quality_score:.2f}'
        )
        return True, message

    def get_median_depth_around_pixel(self, depth_array: np.ndarray, u: int, v: int) -> Optional[float]:
        if depth_array is None:
            return None
        h, w = depth_array.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return None

        r = self.depth_roi_half_size
        x1 = max(0, u - r)
        x2 = min(w, u + r + 1)
        y1 = max(0, v - r)
        y2 = min(h, v + r + 1)
        patch = depth_array[y1:y2, x1:x2]
        valid = patch[
            np.isfinite(patch) &
            (patch > self.min_depth_m) &
            (patch < self.max_depth_m)
        ]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    @staticmethod
    def deproject_pixel_to_point(u: float, v: float, depth: float, fx: float, fy: float, cx: float, cy: float):
        if fx is None or fy is None or cx is None or cy is None:
            return None
        x = float((u - cx) * depth / fx)
        y = float((v - cy) * depth / fy)
        z = float(depth)
        return [x, y, z]

    # ------------------------------------------------------------------
    # Thin-part algorithm: no skeletonize, no distanceTransform.
    # ------------------------------------------------------------------
    def find_thin_part_safe(self, mask: np.ndarray) -> Optional[dict]:
        binary = (mask > 0).astype(np.uint8)
        if int(np.count_nonzero(binary)) < self.min_mask_area_px:
            return None

        # Remove tiny speckles and close small holes/cracks in the mask.
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        orig_h, orig_w = binary.shape[:2]
        scale = self.mask_downsample
        if 0.0 < scale < 1.0:
            small_w = max(2, int(round(orig_w * scale)))
            small_h = max(2, int(round(orig_h * scale)))
            proc = cv2.resize(binary, (small_w, small_h), interpolation=cv2.INTER_NEAREST)
            inv_scale = 1.0 / scale
        else:
            proc = binary
            inv_scale = 1.0

        ys, xs = np.where(proc > 0)
        if len(xs) < self.min_mask_area_px:
            return None

        points = np.stack([xs, ys], axis=1).astype(np.float32)
        mean = points.mean(axis=0)
        centered = points - mean

        # PCA for global mask axis.
        cov = np.cov(centered.T)
        if not np.all(np.isfinite(cov)):
            return None
        eigvals, eigvecs = np.linalg.eig(cov)
        axis = np.real(eigvecs[:, int(np.argmax(np.real(eigvals)))])
        angle_rad = math.atan2(float(axis[1]), float(axis[0]))
        object_axis_deg = self.normalize_angle_deg(math.degrees(angle_rad))

        # Rotate global axis to x-axis.
        c = math.cos(-angle_rad)
        s = math.sin(-angle_rad)
        rot = np.array([[c, -s], [s, c]], dtype=np.float32)
        rotated = centered @ rot.T
        xr = rotated[:, 0]
        yr = rotated[:, 1]

        x_min = int(np.floor(xr.min()))
        x_max = int(np.ceil(xr.max()))
        width_len = x_max - x_min + 1
        if width_len < self.min_valid_columns:
            return None

        cols = np.round(xr - x_min).astype(np.int32)
        cols = np.clip(cols, 0, width_len - 1)

        y_min_arr = np.full(width_len, np.inf, dtype=np.float32)
        y_max_arr = np.full(width_len, -np.inf, dtype=np.float32)
        np.minimum.at(y_min_arr, cols, yr)
        np.maximum.at(y_max_arr, cols, yr)

        widths_proc = y_max_arr - y_min_arr
        valid = np.isfinite(widths_proc) & (widths_proc > 0)
        valid_cols = np.where(valid)[0]
        if len(valid_cols) < self.min_valid_columns:
            return None

        start = int(valid_cols.min())
        end = int(valid_cols.max())
        margin = int(max(1, round((end - start) * self.thin_margin_ratio)))
        search_start = min(end, start + margin)
        search_end = max(start, end - margin)
        if search_end <= search_start:
            return None

        search_valid = np.zeros_like(valid, dtype=bool)
        search_valid[search_start:search_end + 1] = True
        search_valid &= valid
        if int(np.count_nonzero(search_valid)) < self.min_valid_columns:
            return None

        # Smooth widths to suppress single-column mask noise.
        smooth_window = max(1, int(self.smooth_window))
        if smooth_window % 2 == 0:
            smooth_window += 1
        fill_value = float(np.nanmedian(widths_proc[valid]))
        finite_widths = np.where(valid, widths_proc, fill_value)
        if smooth_window > 1:
            kernel = np.ones(smooth_window, dtype=np.float32) / smooth_window
            widths_smooth_proc = np.convolve(finite_widths, kernel, mode='same')
        else:
            widths_smooth_proc = finite_widths.copy()

        # Convert width thresholds from original-image px to processed px.
        min_safe_proc = self.min_safe_width_px / inv_scale if self.min_safe_width_px > 0.0 else 0.0
        max_safe_proc = self.max_safe_width_px / inv_scale if self.max_safe_width_px > 0.0 else float('inf')

        candidate_valid = search_valid.copy()
        if min_safe_proc > 0.0:
            candidate_valid &= widths_smooth_proc >= min_safe_proc
        if np.isfinite(max_safe_proc):
            candidate_valid &= widths_smooth_proc <= max_safe_proc

        mean_search_width = float(np.mean(widths_smooth_proc[search_valid]))
        min_search_width = float(np.min(widths_smooth_proc[search_valid]))
        if mean_search_width <= 1e-6:
            return None
        contrast = (mean_search_width - min_search_width) / mean_search_width
        if contrast < self.min_width_contrast:
            return None

        selection_mode = 1.0
        if int(np.count_nonzero(candidate_valid)) >= self.min_valid_columns:
            candidate_cols = np.where(candidate_valid)[0]
            candidate_widths = widths_smooth_proc[candidate_cols]
            target_width = np.percentile(candidate_widths, np.clip(self.target_width_percentile, 0.0, 100.0))
            chosen_local_idx = int(np.argmin(np.abs(candidate_widths - target_width)))
            thin_col = int(candidate_cols[chosen_local_idx])
            safe_width_score = 1.0
        else:
            if self.reject_unsafe_thin_part:
                return None
            tmp = widths_smooth_proc.copy()
            tmp[~search_valid] = np.inf
            thin_col = int(np.argmin(tmp))
            safe_width_score = 0.0
            selection_mode = 0.0

        thin_width_proc = float(widths_smooth_proc[thin_col])
        thin_width_px = thin_width_proc * inv_scale
        center_y_proc = 0.5 * (y_min_arr[thin_col] + y_max_arr[thin_col])
        center_x_proc = float(thin_col + x_min)
        thin_rot = np.array([center_x_proc, center_y_proc], dtype=np.float32)

        # Restore to original image coords.
        inv_rot = rot.T
        thin_proc_xy = thin_rot @ inv_rot.T + mean
        thin_original = thin_proc_xy * inv_scale
        thin_u = float(thin_original[0])
        thin_v = float(thin_original[1])

        # Local centerline angle around chosen thin column.
        center_y_arr = 0.5 * (y_min_arr + y_max_arr)
        local_radius = max(3, smooth_window)
        a = max(0, thin_col - local_radius)
        b = min(width_len, thin_col + local_radius + 1)
        local_cols = np.arange(a, b)
        local_valid = valid[local_cols]

        if int(np.count_nonzero(local_valid)) >= 3:
            lx = local_cols[local_valid].astype(np.float32)
            ly = center_y_arr[local_cols[local_valid]].astype(np.float32)
            try:
                slope = float(np.polyfit(lx, ly, 1)[0])
                local_angle_rot = math.atan2(slope, 1.0)
                thin_angle_deg = self.normalize_angle_deg(math.degrees(local_angle_rot + angle_rad))
            except Exception:
                thin_angle_deg = object_axis_deg
        else:
            thin_angle_deg = object_axis_deg

        quality_score = float(np.clip(contrast, 0.0, 1.0))

        return {
            'thin_center': (thin_u, thin_v),
            'thin_angle_deg': thin_angle_deg,
            'thin_width_px': thin_width_px,
            'object_axis_deg': object_axis_deg,
            'quality_score': quality_score,
            'safe_width_score': safe_width_score,
            'selection_mode': selection_mode,
        }

    @staticmethod
    def normalize_angle_deg(angle: float) -> float:
        while angle > 90.0:
            angle -= 180.0
        while angle < -90.0:
            angle += 180.0
        return float(angle)

    # ------------------------------------------------------------------
    # Debug outputs
    # ------------------------------------------------------------------
    def draw_debug_image(self, image: np.ndarray, mask: np.ndarray, bbox: np.ndarray, thin: dict, point_3d, depth: float, cls_id: int, conf: float):
        debug = image.copy()
        binary = (mask > 0).astype(np.uint8)
        color = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255)][cls_id % 5]
        overlay = debug.copy()
        overlay[binary > 0] = color
        debug = cv2.addWeighted(overlay, 0.3, debug, 0.7, 0.0)

        x1, y1, x2, y2 = bbox.astype(int).tolist()
        cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)

        u, v = thin['thin_center']
        angle = math.radians(float(thin['thin_angle_deg']))
        length = 45
        p1 = (int(round(u - math.cos(angle) * length)), int(round(v - math.sin(angle) * length)))
        p2 = (int(round(u + math.cos(angle) * length)), int(round(v + math.sin(angle) * length)))
        center = (int(round(u)), int(round(v)))
        cv2.circle(debug, center, 6, (0, 0, 255), -1)
        cv2.line(debug, p1, p2, (255, 255, 255), 2)

        text_lines = [
            f'cls={cls_id} conf={conf:.2f}',
            f'angle={thin["thin_angle_deg"]:.1f} deg width={thin["thin_width_px"]:.1f}px',
            f'uv=({u:.1f},{v:.1f}) depth={depth:.3f}m',
            f'xyz=({point_3d[0]:.3f},{point_3d[1]:.3f},{point_3d[2]:.3f})m',
        ]
        tx, ty = 10, 24
        for line in text_lines:
            cv2.putText(debug, line, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(debug, line, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
            ty += 24
        return debug

    def create_marker(self, point, header, obj_id, cls_id):
        marker = Marker()
        marker.header = header
        marker.header.frame_id = self.camera_frame
        marker.ns = 'thin_part_grasp'
        marker.id = int(obj_id)
        marker.type = Marker.SPHERE
        marker.pose.position.x = float(point[0])
        marker.pose.position.y = float(point[1])
        marker.pose.position.z = float(point[2])
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        colors = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 1.0, 0.0]]
        marker.color.r, marker.color.g, marker.color.b = colors[cls_id % len(colors)]
        marker.color.a = 0.85
        marker.lifetime.sec = 1
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = ThinPartGraspTriggerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
