#!/usr/bin/env python3
"""
YOLO26-seg target mask publisher node.

Terminal role
-------------
Terminal 2: run this node after RealSense is already publishing color images.

What it does
------------
- Subscribes RGB image.
- Runs YOLO26 segmentation model.
- Selects one target object per frame.
- Publishes the selected object's binary mask as sensor_msgs/Image mono8.
- Publishes compact target metadata as Float32MultiArray.
- Publishes an annotated debug image.

Published /yolo_seg/target_info layout
--------------------------------------
[
  class_id, confidence,
  bbox_x1, bbox_y1, bbox_x2, bbox_y2,
  mask_area_px
]

Notes
-----
- This node is segmentation-model-only. If the model does not output masks, it warns and skips.
- The grasp/thin-part calculation is intentionally NOT done here. That work is done by
  thin_part_grasp_trigger_node.py only when triggered.
"""

from pathlib import Path
from typing import Optional, Tuple
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO


class Yolo26SegMaskPublisher(Node):
    def __init__(self):
        super().__init__('yolo26_seg_mask_publisher')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('color_topic', '/d405/color/image_raw')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_class_id', -1)  # -1: highest-confidence object from all classes
        self.declare_parameter('show_visualization', False)
        self.declare_parameter('imgsz', 672)
        self.declare_parameter('retina_masks', True)
        self.declare_parameter('mask_alpha', 0.35)
        self.declare_parameter('max_inference_hz', 8.0)
        self.declare_parameter('preview_publish_hz', 10.0)

        model_path = self.resolve_model_path(str(self.get_parameter('model_path').value))
        self.color_topic = str(self.get_parameter('color_topic').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.target_class_id = int(self.get_parameter('target_class_id').value)
        self.show_visualization = bool(self.get_parameter('show_visualization').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.retina_masks = bool(self.get_parameter('retina_masks').value)
        self.mask_alpha = float(self.get_parameter('mask_alpha').value)
        self.max_inference_hz = float(self.get_parameter('max_inference_hz').value)
        self.preview_publish_hz = float(self.get_parameter('preview_publish_hz').value)
        self.model_backend = Path(model_path).suffix.lower().lstrip('.') or 'unknown'
        self.last_inference_time = 0.0
        self.last_preview_publish_time = 0.0

        # Publishers
        self.mask_pub = self.create_publisher(Image, '/yolo_seg/target_mask', 10)
        self.target_info_pub = self.create_publisher(Float32MultiArray, '/yolo_seg/target_info', 10)
        self.detection_image_pub = self.create_publisher(Image, '/yolo_seg/detection_image', 10)

        self.color_sub = self.create_subscription(
            Image,
            self.color_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f'Subscribing color_topic: {self.color_topic}')

        # Model
        self.get_logger().info(f'Loading YOLO26 segmentation model: {model_path}')
        try:
            import torch
            self.cuda_available = torch.cuda.is_available()
            self.device = 'cuda' if self.cuda_available else 'cpu'
            self.request_half = self.cuda_available
            self.get_logger().info(f'PyTorch CUDA available: {self.cuda_available}')
            if self.cuda_available:
                self.get_logger().info(f'GPU: {torch.cuda.get_device_name(0)}')
        except Exception as exc:
            self.cuda_available = False
            self.device = 'cpu'
            self.request_half = False
            self.get_logger().warn(f'Could not check torch CUDA. Using CPU. reason={exc}')

        self.model = YOLO(model_path)
        self.get_logger().info(
            f'YOLO26-seg Mask Publisher Ready '
            f'(max_inference_hz={self.max_inference_hz:.1f}, '
            f'preview_publish_hz={self.preview_publish_hz:.1f})')

    def resolve_model_path(self, requested_path: str) -> str:
        requested_path = (requested_path or '').strip()
        if requested_path:
            resolved = Path(requested_path).expanduser()
            if resolved.is_file():
                return str(resolved)
            raise FileNotFoundError(f'Configured YOLO model does not exist: {resolved}')

        source_models_dir = Path(__file__).resolve().parents[1] / 'models'
        model_dirs = [source_models_dir]

        try:
            model_dirs.append(Path(get_package_share_directory('yolo_realsense')) / 'models')
        except Exception:
            pass

        candidates = []
        for models_dir in model_dirs:
            candidates.extend(
                [
                    models_dir / 'sam3_finetune_rot50_e20' / 'weights' / 'best.pt',
                    models_dir / 'sam3_finetune_rot50_e20' / 'weights' / 'last.pt',
                    models_dir / 'yolo26_seg.engine',
                    models_dir / 'yolo26_seg.pt',
                    models_dir / 'best_yolo26_seg.engine',
                    models_dir / 'best_yolo26_seg.pt',
                    models_dir / 'best.engine',
                    models_dir / 'best.pt',
                ]
            )
            if models_dir.is_dir():
                candidates.extend(sorted(models_dir.glob('**/weights/best.pt'), key=lambda p: p.stat().st_mtime, reverse=True))
                candidates.extend(sorted(models_dir.glob('**/weights/last.pt'), key=lambda p: p.stat().st_mtime, reverse=True))

        for candidate in candidates:
            if candidate.is_file():
                self.get_logger().info(f'Using segmentation model: {candidate}')
                return str(candidate)

        raise FileNotFoundError(
            'No segmentation model found. Set model_path:=/absolute/path/to/your_yolo26_seg.pt, '
            'or place one under yolo_realsense/models/*/weights/best.pt.'
        )

    @staticmethod
    def image_msg_to_bgr(msg: Image) -> np.ndarray:
        encoding = (msg.encoding or '').lower()
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if encoding in ('bgr8', 'rgb8'):
            channels = 3
        elif encoding in ('bgra8', 'rgba8'):
            channels = 4
        elif encoding in ('mono8', '8uc1'):
            channels = 1
        else:
            raise ValueError(f'Unsupported color image encoding: {msg.encoding}')

        row_size = width * channels
        if step and step >= row_size:
            image = data.reshape((height, step))[:, :row_size].reshape((height, width, channels))
        else:
            image = data.reshape((height, width, channels))

        image = image.copy()
        if encoding == 'rgb8':
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if encoding in ('mono8', '8uc1'):
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return image

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

    @staticmethod
    def mask_to_image_msg(mask: np.ndarray, header) -> Image:
        mask = np.ascontiguousarray(mask, dtype=np.uint8)
        msg = Image()
        msg.header = header
        msg.height = int(mask.shape[0])
        msg.width = int(mask.shape[1])
        msg.encoding = 'mono8'
        msg.is_bigendian = 0
        msg.step = int(mask.shape[1])
        msg.data = mask.tobytes()
        return msg

    @staticmethod
    def ensure_mask_size(mask: np.ndarray, target_hw: Tuple[int, int]) -> np.ndarray:
        target_h, target_w = target_hw
        if mask.shape[:2] != (target_h, target_w):
            mask = cv2.resize(mask.astype(np.uint8), (target_w, target_h), interpolation=cv2.INTER_NEAREST)
        return mask.astype(np.uint8)

    @staticmethod
    def draw_mask_overlay(frame: np.ndarray, mask: np.ndarray, color: Tuple[int, int, int], alpha: float) -> np.ndarray:
        overlay = frame.copy()
        overlay[mask > 0] = color
        return cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0.0)

    def select_target(self, result, masks_np: np.ndarray) -> Optional[Tuple[int, int, float, np.ndarray, np.ndarray]]:
        """Return (idx, cls_id, conf, xyxy, mask) for selected detection."""
        if result.boxes is None or len(result.boxes) == 0:
            return None

        best = None
        best_conf = -1.0
        boxes = result.boxes

        for i, box in enumerate(boxes):
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            if conf < self.confidence_threshold:
                continue
            if self.target_class_id >= 0 and cls_id != self.target_class_id:
                continue
            if i >= len(masks_np):
                continue
            if conf > best_conf:
                xyxy = box.xyxy[0].detach().cpu().numpy().astype(int)
                best = (i, cls_id, conf, xyxy, masks_np[i])
                best_conf = conf

        return best

    def image_callback(self, color_msg: Image):
        try:
            color_image = self.image_msg_to_bgr(color_msg)
        except Exception as exc:
            self.get_logger().error(f'Color image conversion error: {exc}')
            return

        annotated = color_image.copy()
        now = time.monotonic()

        infer_period = 0.0 if self.max_inference_hz <= 0.0 else 1.0 / self.max_inference_hz
        preview_period = 0.0 if self.preview_publish_hz <= 0.0 else 1.0 / self.preview_publish_hz
        if infer_period > 0.0 and (now - self.last_inference_time) < infer_period:
            if preview_period <= 0.0 or (now - self.last_preview_publish_time) >= preview_period:
                out_msg = self.bgr_to_image_msg(annotated, color_msg.header)
                self.detection_image_pub.publish(out_msg)
                self.last_preview_publish_time = now
            return

        self.last_inference_time = now

        try:
            results = self.model.predict(
                source=color_image,
                verbose=False,
                device=self.device,
                half=self.request_half,
                imgsz=self.imgsz,
                retina_masks=self.retina_masks,
                conf=max(0.001, min(self.confidence_threshold, 0.99)),
            )
        except Exception as exc:
            self.get_logger().error(f'YOLO inference error: {exc}', throttle_duration_sec=3.0)
            cv2.putText(
                annotated,
                'YOLO inference error',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            out_msg = self.bgr_to_image_msg(annotated, color_msg.header)
            self.detection_image_pub.publish(out_msg)
            self.last_preview_publish_time = now
            return

        selected_any = False

        for result in results:
            if result.masks is None or result.masks.data is None:
                self.get_logger().warn('Segmentation model produced no masks. Check that the loaded model is *-seg.', throttle_duration_sec=3.0)
                continue

            masks_np = result.masks.data.detach().cpu().numpy()
            masks_np = (masks_np > 0.5).astype(np.uint8) * 255

            selected = self.select_target(result, masks_np)
            if selected is None:
                continue

            idx, cls_id, conf, xyxy, mask = selected
            mask = self.ensure_mask_size(mask, color_image.shape[:2])
            x1, y1, x2, y2 = xyxy.tolist()
            mask_area = int(np.count_nonzero(mask))

            color = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255)][cls_id % 5]
            annotated = self.draw_mask_overlay(annotated, mask, color, self.mask_alpha)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            class_name = self.model.names.get(cls_id, str(cls_id)) if hasattr(self.model, 'names') else str(cls_id)
            cv2.putText(
                annotated,
                f'{class_name} {conf:.2f} area={mask_area}',
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )

            # Publish selected target mask with same header as color frame.
            mask_msg = self.mask_to_image_msg(mask, color_msg.header)
            self.mask_pub.publish(mask_msg)

            info_msg = Float32MultiArray()
            info_msg.data = [
                float(cls_id),
                float(conf),
                float(x1), float(y1), float(x2), float(y2),
                float(mask_area),
            ]
            self.target_info_pub.publish(info_msg)
            selected_any = True

        if self.show_visualization:
            cv2.imshow('YOLO26-seg Mask Publisher', annotated)
            cv2.waitKey(1)

        out_msg = self.bgr_to_image_msg(annotated, color_msg.header)
        self.detection_image_pub.publish(out_msg)
        self.last_preview_publish_time = now

        if not selected_any:
            # Still publish image; no target mask/info this frame.
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Yolo26SegMaskPublisher()
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
