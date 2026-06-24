# Gripper SDK

ROS 2 hardware package for the parallel gripper.

Current implementation:

- Hardware path: U2D2 direct Dynamixel communication
- ROS package: `gripper_hardware`
- Plugin: `gripper_hardware/GripperHardwareInterface`
- Controller interface: `control_msgs/action/GripperCommand`
- Main action: `/gripper_controller/gripper_cmd`

Default fullpackage values:

| item | value |
| --- | --- |
| port | `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4ZHT-if00-port0` |
| baudrate | `1000000` |
| Dynamixel ID | `15` |
| open position | `0.029` |
| close position | `0.000` |

The integrated URDF maps the ROS gripper position range `0.000..0.030` to
Dynamixel ticks using the current calibration parameters.

Run only the gripper:

```bash
ros2 launch frjoco_bringup gripper.launch.py \
  gripper_port:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4ZHT-if00-port0 \
  gripper_id:=15 \
  gripper_baudrate:=1000000
```

Send a command:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.000, max_effort: 50.0}}"
```
