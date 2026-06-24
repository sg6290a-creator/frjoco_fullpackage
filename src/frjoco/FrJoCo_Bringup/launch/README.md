# FrJoCo Launch Files

Top-level launch files live here. Implementation-only launch files live in
`internal/`.

## Hardware

```bash
ros2 launch frjoco_bringup main.launch.py
ros2 launch frjoco_bringup arm.launch.py
ros2 launch frjoco_bringup gripper.launch.py
ros2 launch frjoco_bringup mobile.launch.py
ros2 launch frjoco_bringup sensors.launch.py
```

| launch | purpose |
| --- | --- |
| `main.launch.py` | integrated arm/mobile/gripper/vision/MoveIt/Nav2/web entrypoint |
| `arm.launch.py` | CAN arm hardware, MoveIt, RViz |
| `gripper.launch.py` | U2D2 Dynamixel gripper only |
| `mobile.launch.py` | MD200T mobile base, RViz, optional web |
| `sensors.launch.py` | RealSense cameras and Livox |

## Simulation

```bash
ros2 launch frjoco_bringup main_sim.launch.py
ros2 launch frjoco_bringup arm_sim.launch.py
ros2 launch frjoco_bringup mobile_sim.launch.py
```

| launch | purpose |
| --- | --- |
| `main_sim.launch.py` | no-hardware integrated UI/MoveIt/Nav2 check |
| `arm_sim.launch.py` | arm planning/RViz check |
| `mobile_sim.launch.py` | map/Nav2/mobile-base check |

## Common Main Options

```text
enable_arm_hardware:=true
enable_mobile_hardware:=false
enable_gripper_hardware:=false
enable_realsense:=true
enable_yolo:=true
enable_web:=true
enable_mobile_teleop:=false
enable_move_executor:=true
enable_hardware_status:=true
enable_nav_sensors:=false
enable_rtab_livox:=false
enable_nav2:=false
rviz:=true
```

Mobile keyboard teleop publishes to:

```text
/diff_drive_controller/cmd_vel_unstamped
```
