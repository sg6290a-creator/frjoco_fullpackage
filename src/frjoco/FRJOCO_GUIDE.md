# FrJoCo Fullpackage Guide

이 문서는 clone 위치를 `FRJOCO_WS`로 둡니다. 예시는 `~/frjoco_ws`입니다.

```bash
export FRJOCO_WS="$HOME/frjoco_ws"
```

`build/`, `install/`, `log/`는 git에 포함하지 않습니다.

## 1. Workspace Setup

처음 받는 경우:

```bash
export FRJOCO_WS="$HOME/frjoco_ws"
mkdir -p "$(dirname "$FRJOCO_WS")"
git clone https://github.com/sg6290a-creator/frjoco_fullpackage.git "$FRJOCO_WS"
cd "$FRJOCO_WS"
```

이미 다른 위치에 clone한 경우:

```bash
cd /path/to/frjoco_fullpackage
export FRJOCO_WS="$(pwd)"
```

## 2. Source

새 터미널마다:

```bash
export FRJOCO_WS="$HOME/frjoco_ws"
cd "$FRJOCO_WS"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

현재 source 확인:

```bash
ros2 pkg prefix frjoco_bringup
```

기대값:

```text
$FRJOCO_WS/install/frjoco_bringup
```

`dynamixel_sdk`는 fullpackage 내부 `src/frjoco/FrJoCo_Hardware/third_party/dynamixel_sdk`에 포함되어 있습니다.

## 3. Hardware Prep

### Arm CAN

CANable/USB-CAN 확인:

```bash
lsusb
lsusb -t
ip -details link show
```

`gs_usb`로 잡히는 경우:

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe gs_usb
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

`cdc_acm`/CANable slcan으로 잡히는 경우:

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe slcan
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
ip -details link show can0
```

수신 확인:

```bash
candump can0
```

### U2D2 Gripper

기본값:

| 항목 | 값 |
| --- | --- |
| Port | `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4ZHT-if00-port0` |
| Baudrate | `1000000` |
| Dynamixel ID | `15` |
| ROS position open | `0.029` |
| ROS position close | `0.000` |

포트 확인:

```bash
ls -l /dev/serial/by-id/
```

단독 실행:

```bash
ros2 launch frjoco_bringup gripper.launch.py \
  gripper_port:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4ZHT-if00-port0 \
  gripper_id:=15 \
  gripper_baudrate:=1000000
```

명령 테스트:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.000, max_effort: 50.0}}"
```

### Mobile Base

기본 포트:

```text
port_front:=/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.1:1.0-port0
port_rear:=/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.2:1.0-port0
front_driver_id:=1
rear_driver_id:=2
mobile_baudrate:=57600
```

확인:

```bash
ls -l /dev/serial/by-path/
```

### RealSense / YOLO

```bash
lsusb | grep -i -E 'intel|realsense'
```

workspace에 포함된 YOLO weight:

```text
$FRJOCO_WS/src/frjoco/FrJoCo_YOLO/models/sam3_finetune_rot50_e20/weights/best.pt
```

launch 전에 변수로 잡아두면 편합니다.

```bash
export YOLO_MODEL_PATH="$FRJOCO_WS/src/frjoco/FrJoCo_YOLO/models/sam3_finetune_rot50_e20/weights/best.pt"
```

다른 weight를 쓸 때는 원하는 파일로 바꿉니다.

```bash
export YOLO_MODEL_PATH="/path/to/best.pt"
```

## 4. Build

전체 빌드:

```bash
cd "$FRJOCO_WS"
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
```

bringup 문서/launch만 빠르게 확인할 때:

```bash
colcon build --symlink-install \
  --packages-select frjoco_bringup robot_web_interface \
  --cmake-args -DBUILD_TESTING=OFF
```

## 5. Main Launch

팔, 그리퍼, 카메라, YOLO, 웹 UI, MoveIt executor:

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

모바일까지 포함:

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

모바일 키보드 teleop 포함:

```bash
ros2 launch frjoco_bringup main.launch.py \
  enable_mobile_hardware:=true \
  enable_mobile_teleop:=true
```

teleop topic:

```text
/diff_drive_controller/cmd_vel_unstamped
```

Nav2/Livox까지 포함:

```bash
ros2 launch frjoco_bringup main.launch.py \
  enable_mobile_hardware:=true \
  enable_nav_sensors:=true \
  enable_rtab_livox:=true \
  enable_nav2:=true
```

## 6. Single Launches

```bash
ros2 launch frjoco_bringup arm.launch.py
ros2 launch frjoco_bringup gripper.launch.py
ros2 launch frjoco_bringup mobile.launch.py
ros2 launch frjoco_bringup sensors.launch.py
```

## 7. Simulation

```bash
ros2 launch frjoco_bringup main_sim.launch.py
ros2 launch frjoco_bringup arm_sim.launch.py
ros2 launch frjoco_bringup mobile_sim.launch.py
```

시뮬레이션은 하드웨어 없이 `/joint_states`, MoveIt, RViz, web UI, Nav2 흐름 확인용입니다.

## 8. Web UI

`enable_web:=true`면 자동 실행됩니다.

```text
http://localhost:8000
```

별도 실행:

```bash
ros2 launch robot_web_interface web_interface.launch.py
```

## 9. Runtime Flow

```text
Arm CAN
  -> frlab_manipulator_hardware
  -> ros2_control joint_trajectory_controller
  -> MoveIt execute

U2D2 gripper
  -> gripper_hardware
  -> gripper_controller
  -> UI / vision_move GripperCommand

Mobile base
  -> mobile_sdk
  -> diff_drive_controller
  -> UI / teleop / Nav2 cmd_vel

D405/D455 + YOLO
  -> yolo_realsense
  -> thin_part_grasp_node
  -> yolo_grasp_pose_bridge.py
  -> /target_pose
  -> vision_move.py
  -> MoveIt plan/execute
```

## 10. Pick And Place

현재 `vision_move.py` 기준 흐름:

```text
YOLO object detection
  -> /thin_part/estimate
  -> /execute_pick_place
  -> /target_pose
  -> workspace gate
  -> pre-grasp align at target x -0.10 m
  -> final target pose
  -> optional wrist_3 grasp roll
  -> gripper close
  -> carry pose
  -> place pose
  -> post-place shoulder_pan turn
  -> delay
  -> gripper open
  -> place pose return
  -> ready pose
```

기본 자세:

| pose | J1 | J2 | J3 | J4 | J5 | J6 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| home | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| place | 0.0 | -0.0700 | -2.4870 | 1.3538 | 0.0 | 0.0 |

주요 launch 인자:

```text
planning_mode:=manual_ik
workspace_gate_enabled:=true
pre_grasp_align_enabled:=true
pre_grasp_align_x_offset:=-0.10
grasp_roll_enabled:=true
post_place_turn_enabled:=true
post_place_turn_angle:=-3.1416
pre_open_after_place_delay_sec:=1.5
carry_elbow_extra_bend_deg:=6.0
```

## 11. Quick Checks

컨트롤러:

```bash
ros2 control list_controllers
```

조인트:

```bash
ros2 topic echo /joint_states --once
```

그리퍼:

```bash
ros2 action list -t | grep gripper
```

카메라/YOLO:

```bash
ros2 topic hz /d405/color/image_raw
ros2 topic hz /yolo_seg/detection_image
ros2 topic echo /yolo_seg/target_info --once
```

Nav2:

```bash
ros2 topic list | grep -E 'map|scan|odom|goal_pose|cmd_vel'
```

## 12. Package Roles

| package | role |
| --- | --- |
| `frjoco_bringup` | top-level launch, RViz, status tools |
| `mobile_manipulator_moveit_config` | MoveIt config, `vision_move.py`, pose publishers |
| `manipulator_description` | integrated URDF/XACRO |
| `frjoco_model` | mobile/manipulator model assets |
| `frlab_manipulator_hardware` | arm ros2_control hardware |
| `gripper_hardware` | U2D2 Dynamixel gripper hardware |
| `mobile_sdk` | mobile base hardware |
| `robot_nav2` | Nav2 params and map |
| `robot_slam` | Cartographer/RTAB mapping config |
| `robot_web_interface` | browser UI |
| `yolo_realsense` | YOLO segmentation and grasp estimation |

## 13. Troubleshooting

### Wrong workspace is active

Symptom:

```text
file '...' was not found in the share directory
```

Fix:

```bash
cd "$FRJOCO_WS"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 pkg prefix frjoco_bringup
```

### CAN is not up

Check:

```bash
ip -details link show can0
candump can0
```

If `can0` does not exist, check whether the adapter is `gs_usb` or `cdc_acm` with:

```bash
lsusb -t
```

### controller_manager disappears

Because arm, mobile, and gripper are loaded into the same ros2_control stack in the integrated launch, one failed hardware configure can bring the controller manager down.

Common causes:

| log | likely cause |
| --- | --- |
| `FrlabManipulator::init() failed` | CAN, arm power, motor response |
| `Failed to open U2D2 gripper port` | gripper power/port |
| `MobileSystemInterface ... Failed to open serial ports` | mobile USB port order/path |

Run one subsystem first:

```bash
ros2 launch frjoco_bringup arm.launch.py
ros2 launch frjoco_bringup gripper.launch.py
ros2 launch frjoco_bringup mobile.launch.py
```

### RealSense or YOLO is not publishing

Check devices and topics:

```bash
lsusb | grep -i -E 'intel|realsense'
ros2 topic list | grep -E 'd405|d455|yolo'
```

For arm-only testing, disable vision:

```bash
enable_realsense:=false enable_yolo:=false
```

### RViz shows stale or repeated poses

Close old RViz/launch sessions and start one launch only. Then check publishers:

```bash
ros2 topic info -v /joint_states
```

There should be one intended `/joint_states` source for the active mode.
