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

## Build

```bash
cd /home/frlab/ing_ws/src/fullpackage
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
```

## Source

Run this in every new terminal:

```bash
cd /home/frlab/ing_ws/src/fullpackage
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

Confirm that the correct workspace is active:

```bash
ros2 pkg prefix frjoco_bringup
```

Expected prefix:

```text
/home/frlab/ing_ws/src/fullpackage/install/frjoco_bringup
```

## Main Launch

Arm, D405/D455, YOLO, MoveIt executor, web UI, status monitor, and RViz:

```bash
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
  rviz:=true
```

Add mobile base hardware:

```bash
ros2 launch frjoco_bringup main.launch.py \
  enable_arm_hardware:=true \
  enable_mobile_hardware:=true \
  enable_gripper_hardware:=true \
  enable_realsense:=true \
  enable_yolo:=true \
  enable_web:=true \
  enable_move_executor:=true \
  enable_hardware_status:=true \
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
