# Robot Nav2

Nav2 configuration package used by `frjoco_bringup`.

Current usage is through the fullpackage launch files, not separate legacy
launch entrypoints.

## Hardware Nav2

```bash
ros2 launch frjoco_bringup main.launch.py \
  enable_mobile_hardware:=true \
  enable_nav_sensors:=true \
  enable_rtab_livox:=true \
  enable_nav2:=true
```

## Simulation Nav2

```bash
ros2 launch frjoco_bringup mobile_sim.launch.py
```

or as part of the integrated no-hardware stack:

```bash
ros2 launch frjoco_bringup main_sim.launch.py
```

## Frames

```text
map
  -> odom
    -> base_footprint
      -> base_link
```

Nav2 uses `base_footprint` as the robot base frame.

## Main Files

| file | purpose |
| --- | --- |
| `config/nav2_params.yaml` | hardware Nav2 parameters |
| `config/nav2_params_safe.yaml` | conservative Nav2 parameters |
| `maps/my_map.yaml` | default static map |
| `maps/my_map.pgm` | default map image |

## Quick Checks

```bash
ros2 topic list | grep -E 'map|scan|odom|goal_pose|cmd_vel'
ros2 run tf2_ros tf2_echo map base_footprint
```
