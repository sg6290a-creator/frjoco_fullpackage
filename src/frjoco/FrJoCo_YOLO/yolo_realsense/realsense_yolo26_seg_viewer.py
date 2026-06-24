#!/usr/bin/env python3
"""Show RealSense color feed with YOLO26-seg detections overlaid.

Run after building the yolo_realsense package or directly from this source tree.

Examples:
    ros2 run yolo_realsense yolo_seg_viewer
    ros2 run yolo_realsense yolo_seg_viewer --ros-args -- --conf 0.35 --show-depth
    python3 yolo_realsense/realsense_yolo26_seg_viewer.py --model models/sam3_finetune_rot50_e20/weights/best.pt

Controls:
    q or Esc: quit
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = PACKAGE_ROOT / "models" / "sam3_finetune_rot50_e20" / "weights" / "best.pt"
CLASS_COLORS = [
    (40, 80, 255),   # hammer: red-ish
    (40, 220, 80),   # pliers: green
    (255, 120, 40),  # screwdriver: blue-ish
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RealSense + YOLO26-seg live detection viewer.")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="Path to YOLO segmentation .pt.")
    parser.add_argument("--camera-width", type=int, default=0, help="RealSense color width. 0 lets RealSense choose.")
    parser.add_argument("--camera-height", type=int, default=0, help="RealSense color height. 0 lets RealSense choose.")
    parser.add_argument("--fps", type=int, default=30, help="RealSense stream FPS.")
    parser.add_argument("--imgsz", type=int, default=672, help="YOLO inference size. Must be divisible by YOLO stride 32.")
    parser.add_argument("--conf", type=float, default=0.35, help="Detection confidence threshold.")
    parser.add_argument("--iou", type=float, default=0.7, help="NMS IoU threshold.")
    parser.add_argument("--device", default="auto", help="auto, cpu, 0, 1, ...")
    parser.add_argument("--half", action="store_true", help="Use FP16 on CUDA.")
    parser.add_argument("--retina-masks", action="store_true", default=True, help="Return masks at image resolution.")
    parser.add_argument("--mask-alpha", type=float, default=0.35, help="Mask overlay opacity.")
    parser.add_argument("--target-class", type=int, default=-1, help="-1 shows all classes, otherwise filter by class id.")
    parser.add_argument("--show-depth", action="store_true", help="Show aligned depth colormap next to detection view.")
    parser.add_argument("--mirror", action="store_true", help="Mirror the preview horizontally.")
    parser.add_argument("--list-profiles", action="store_true", help="Print available RealSense color/depth profiles and exit.")
    return parser.parse_args()


def resolve_path(path: Path) -> Path:
    return path if path.is_absolute() else PACKAGE_ROOT / path


def parse_device(value: str) -> Any:
    if value != "auto":
        return value
    try:
        import torch

        return 0 if torch.cuda.is_available() else "cpu"
    except Exception:
        return "cpu"


def latest_model_fallback() -> Path | None:
    candidates = []
    for base in [PACKAGE_ROOT / "models"]:
        if base.exists():
            candidates.extend(base.glob("**/weights/best.pt"))
    candidates = sorted(candidates, key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates[0] if candidates else None


def ensure_mask_size(mask: np.ndarray, hw: tuple[int, int]) -> np.ndarray:
    height, width = hw
    if mask.shape[:2] != (height, width):
        mask = cv2.resize(mask.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST)
    return mask.astype(np.uint8)


def overlay_mask(image: np.ndarray, mask: np.ndarray, color: tuple[int, int, int], alpha: float) -> np.ndarray:
    overlay = image.copy()
    overlay[mask > 0] = color
    return cv2.addWeighted(overlay, alpha, image, 1.0 - alpha, 0.0)


def draw_detections(frame: np.ndarray, result: Any, names: dict[int, str], args: argparse.Namespace) -> tuple[np.ndarray, int]:
    annotated = frame.copy()
    boxes = result.boxes
    masks = result.masks
    if boxes is None or len(boxes) == 0:
        return annotated, 0

    masks_np = None
    if masks is not None and masks.data is not None:
        masks_np = masks.data.detach().cpu().numpy()
        masks_np = (masks_np > 0.5).astype(np.uint8) * 255

    drawn = 0
    for i, box in enumerate(boxes):
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        if args.target_class >= 0 and cls_id != args.target_class:
            continue

        color = CLASS_COLORS[cls_id % len(CLASS_COLORS)]
        if masks_np is not None and i < len(masks_np):
            mask = ensure_mask_size(masks_np[i], annotated.shape[:2])
            annotated = overlay_mask(annotated, mask, color, args.mask_alpha)

        x1, y1, x2, y2 = box.xyxy[0].detach().cpu().numpy().astype(int).tolist()
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

        class_name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
        label = f"{class_name} {conf:.2f}"
        text_y = max(24, y1 - 8)
        cv2.putText(annotated, label, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4)
        cv2.putText(annotated, label, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        drawn += 1

    return annotated, drawn


def colorize_depth(depth_image: np.ndarray) -> np.ndarray:
    depth_8u = cv2.convertScaleAbs(depth_image, alpha=0.03)
    return cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)


def list_profiles(rs: Any) -> None:
    ctx = rs.context()
    if len(ctx.devices) == 0:
        print("No RealSense device found.")
        return

    for dev in ctx.devices:
        name = dev.get_info(rs.camera_info.name)
        serial = dev.get_info(rs.camera_info.serial_number)
        print(f"device: {name} serial={serial}")
        for sensor in dev.query_sensors():
            sensor_name = sensor.get_info(rs.camera_info.name)
            print(f"  sensor: {sensor_name}")
            profiles = []
            for profile in sensor.get_stream_profiles():
                try:
                    video = profile.as_video_stream_profile()
                except Exception:
                    continue
                if profile.stream_type() not in (rs.stream.color, rs.stream.depth):
                    continue
                profiles.append(
                    (
                        str(profile.stream_type()),
                        video.width(),
                        video.height(),
                        profile.fps(),
                        str(profile.format()),
                    )
                )
            for stream, width, height, fps, fmt in sorted(set(profiles)):
                print(f"    {stream:16s} {width:4d}x{height:<4d} {fps:3d}fps {fmt}")


def configure_realsense_streams(rs: Any, args: argparse.Namespace) -> tuple[Any, Any | None]:
    config = rs.config()

    if args.camera_width > 0 and args.camera_height > 0:
        config.enable_stream(rs.stream.color, args.camera_width, args.camera_height, rs.format.bgr8, args.fps)
        if args.show_depth:
            config.enable_stream(rs.stream.depth, args.camera_width, args.camera_height, rs.format.z16, args.fps)
            return config, rs.align(rs.stream.color)
        return config, None

    config.enable_stream(rs.stream.color, rs.format.bgr8, args.fps)
    if args.show_depth:
        config.enable_stream(rs.stream.depth, rs.format.z16, args.fps)
        return config, rs.align(rs.stream.color)
    return config, None


def main() -> int:
    args = parse_args()
    model_path = resolve_path(args.model)
    if not model_path.exists():
        fallback = latest_model_fallback()
        if fallback is None:
            raise FileNotFoundError(f"Model not found and no fallback best.pt exists: {model_path}")
        print(f"model not found: {model_path}")
        print(f"using latest fallback: {fallback}")
        model_path = fallback

    import pyrealsense2 as rs
    from ultralytics import YOLO

    if args.list_profiles:
        list_profiles(rs)
        return 0

    device = parse_device(args.device)
    use_half = bool(args.half and device != "cpu")
    print(f"model: {model_path}")
    print(f"device: {device}, half: {use_half}")
    print("press q or Esc to quit")

    model = YOLO(str(model_path))

    pipeline = rs.pipeline()
    config, align = configure_realsense_streams(rs, args)
    try:
        profile = pipeline.start(config)
    except RuntimeError as exc:
        hint = (
            "RealSense stream request failed. Try auto mode with no --camera-width/--camera-height, "
            "or run `ros2 run yolo_realsense yolo_seg_viewer -- --list-profiles` "
            "and pass a supported --camera-width --camera-height --fps."
        )
        raise RuntimeError(f"{exc}\n{hint}") from exc

    color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    print(f"color_stream: {color_profile.width()}x{color_profile.height()}@{color_profile.fps()}")
    prev_time = time.perf_counter()
    fps_smooth = 0.0

    try:
        while True:
            frames = pipeline.wait_for_frames()
            if align is not None:
                frames = align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            if args.mirror:
                frame = cv2.flip(frame, 1)

            results = model.predict(
                source=frame,
                verbose=False,
                device=device,
                half=use_half,
                imgsz=args.imgsz,
                conf=args.conf,
                iou=args.iou,
                retina_masks=args.retina_masks,
            )

            annotated = frame.copy()
            detection_count = 0
            if results:
                annotated, detection_count = draw_detections(annotated, results[0], model.names, args)

            now = time.perf_counter()
            instant_fps = 1.0 / max(now - prev_time, 1e-6)
            prev_time = now
            fps_smooth = instant_fps if fps_smooth == 0.0 else fps_smooth * 0.9 + instant_fps * 0.1

            status = f"YOLO26-seg | detections={detection_count} | FPS={fps_smooth:.1f} | conf={args.conf:.2f}"
            cv2.putText(annotated, status, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(annotated, status, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            if args.show_depth:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    depth_view = colorize_depth(np.asanyarray(depth_frame.get_data()))
                    depth_view = cv2.resize(depth_view, (annotated.shape[1], annotated.shape[0]))
                    annotated = np.hstack([annotated, depth_view])

            cv2.imshow("RealSense YOLO26-seg Detection", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
