# FrJoCo Web UI

Browser UI for the fullpackage runtime.

The normal path is to start it from `main.launch.py`:

```bash
ros2 launch frjoco_bringup main.launch.py enable_web:=true
```

Open:

```text
http://localhost:8000
```

Run only the web layer:

```bash
ros2 launch robot_web_interface web_interface.launch.py
```

## Ports

| port | purpose |
| --- | --- |
| `8000` | static web UI |
| `9090` | rosbridge websocket |
| `8080` | web video server |

## Main ROS Interfaces

| UI feature | ROS interface |
| --- | --- |
| mobile joystick | `/diff_drive_controller/cmd_vel_unstamped` |
| arm joint buttons | `/joint_trajectory_controller/joint_trajectory` |
| gripper buttons | `/web/gripper_command` bridge to `/gripper_controller/gripper_cmd` |
| target pose | `/target_pose` |
| pick/place | `/execute_pick_place` |
| status log | `/pick_place_status` |
| gripper center XYZ | `/gripper_center_pose` |
| D405 image | `/d405/color/image_raw` |
| D455 image | `/d455/color/image_raw` |
| YOLO preview | `/yolo_seg/detection_image` |

## Quick Checks

```bash
ros2 topic info -v /joint_states
ros2 topic list | grep -E 'd405|d455|yolo|pick_place|gripper'
ros2 action list -t | grep gripper
```
