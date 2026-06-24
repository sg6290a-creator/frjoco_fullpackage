# FrJoCo Fullpackage

ROS 2 Humble workspace for the FrJoCo mobile manipulator.

This repository contains the current integrated stack:

- CAN arm hardware through `frlab_manipulator_hardware`
- U2D2/Dynamixel gripper through `gripper_hardware`
- MD200T mobile base through `mobile_sdk`
- RealSense D405/D455 vision and YOLO segmentation
- MoveIt execution, Nav2, RViz, and browser UI

Generated build outputs are not tracked. Rebuild `build/`, `install/`, and
`log/` locally.

## Workspace Setup

Choose any workspace path you want. The examples below use `~/frjoco_ws`.

```bash
export FRJOCO_WS="$HOME/frjoco_ws"
mkdir -p "$(dirname "$FRJOCO_WS")"
git clone https://github.com/sg6290a-creator/frjoco_fullpackage.git "$FRJOCO_WS"
cd "$FRJOCO_WS"
```

If you already cloned this repository somewhere else, set `FRJOCO_WS` to that
directory instead:

```bash
cd /path/to/frjoco_fullpackage
export FRJOCO_WS="$(pwd)"
```

## Build

```bash
cd "$FRJOCO_WS"
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
```

## Source

Run this in every new terminal:

```bash
export FRJOCO_WS="$HOME/frjoco_ws"
cd "$FRJOCO_WS"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

Confirm that the correct workspace is active:

```bash
ros2 pkg prefix frjoco_bringup
```

Expected prefix:

```text
$FRJOCO_WS/install/frjoco_bringup
```

## Main Launch

Arm, D405/D455, YOLO, MoveIt executor, web UI, status monitor, and RViz:

```bash
export YOLO_MODEL_PATH="$FRJOCO_WS/src/frjoco/FrJoCo_YOLO/models/sam3_finetune_rot50_e20/weights/best.pt"

ros2 launch frjoco_bringup main.launch.py \
  enable_arm_hardware:=true \
  enable_mobile_hardware:=false \
  enable_gripper_hardware:=true \
  enable_realsense:=true \
  enable_yolo:=true \
  enable_web:=true \
  enable_move_executor:=true \
  enable_hardware_status:=true \
  enable_nav_sensors:=false \
  enable_nav2:=false \
  enable_rtab_livox:=false \
  model_path:="$YOLO_MODEL_PATH" \
  rviz:=true
```

Add mobile base hardware:

```bash
export YOLO_MODEL_PATH="$FRJOCO_WS/src/frjoco/FrJoCo_YOLO/models/sam3_finetune_rot50_e20/weights/best.pt"

ros2 launch frjoco_bringup main.launch.py \
  enable_arm_hardware:=true \
  enable_mobile_hardware:=true \
  enable_gripper_hardware:=true \
  enable_realsense:=true \
  enable_yolo:=true \
  enable_web:=true \
  enable_move_executor:=true \
  enable_hardware_status:=true \
  model_path:="$YOLO_MODEL_PATH" \
  rviz:=true
```

Add keyboard teleop for the mobile base:

```bash
ros2 launch frjoco_bringup main.launch.py \
  enable_mobile_hardware:=true \
  enable_mobile_teleop:=true
```

The teleop node publishes to `/diff_drive_controller/cmd_vel_unstamped`.

## Single-System Launches

```bash
ros2 launch frjoco_bringup arm.launch.py
ros2 launch frjoco_bringup gripper.launch.py
ros2 launch frjoco_bringup mobile.launch.py
ros2 launch frjoco_bringup sensors.launch.py
```

## Simulation

```bash
ros2 launch frjoco_bringup main_sim.launch.py
ros2 launch frjoco_bringup arm_sim.launch.py
ros2 launch frjoco_bringup mobile_sim.launch.py
```

## Web UI

The main launch starts the web UI when `enable_web:=true`.

```text
http://localhost:8000
```

## More Detail

Operational notes live in:

```text
src/frjoco/FRJOCO_GUIDE.md
```
