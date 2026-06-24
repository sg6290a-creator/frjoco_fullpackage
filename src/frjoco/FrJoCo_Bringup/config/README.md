# FrJoCo Bringup Config

Runtime configuration used by `frjoco_bringup`.

| file | purpose |
| --- | --- |
| `mobile_manipulator_full_controllers.yaml` | integrated arm/mobile/gripper controllers |
| `mobile_manipulator_nav_controllers.yaml` | controller setup for Nav2 mode |
| `diff_drive_controller.yaml` | mobile base hardware controller |
| `diff_drive_controller_sim.yaml` | simulated mobile base controller |
| `gripper_only.urdf.xacro` | gripper-only ros2_control model |
| `gripper_only_controllers.yaml` | gripper-only controller config |
| `nav2_params.yaml` | hardware Nav2 parameters |
| `sim_nav2_params.yaml` | simulation Nav2 parameters |
| `ekf.yaml` | wheel/IMU odometry fusion |
| `ekf_icp_d455.yaml` | ICP/D455 odometry fusion |
| `pick_place_config.yaml` | legacy topic pick/place defaults |

The active integrated pick/place flow is launched from
`launch/internal/main_hardware_impl.launch.py` and executed by
`mobile_manipulator_moveit_config/scripts/vision_move.py`.
