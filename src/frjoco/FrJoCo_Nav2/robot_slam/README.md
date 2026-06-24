# Robot SLAM

SLAM and mapping configuration used by the fullpackage navigation stack.

The normal Livox/RTAB mapping path is launched from `frjoco_bringup`:

```bash
ros2 launch frjoco_bringup main.launch.py \
  enable_mobile_hardware:=true \
  enable_nav_sensors:=true \
  enable_rtab_livox:=true \
  enable_nav2:=false
```

To run Nav2 with the mapping/localization stack:

```bash
ros2 launch frjoco_bringup main.launch.py \
  enable_mobile_hardware:=true \
  enable_nav_sensors:=true \
  enable_rtab_livox:=true \
  enable_nav2:=true
```

## Main Files

| file | purpose |
| --- | --- |
| `config/cartographer.lua` | Cartographer scan/odom config |
| `config/rtabmap_livox.ini` | RTAB/Livox mapping config |
| `launch/cartographer.launch.py` | standalone Cartographer helper |
| `rviz/slam.rviz` | SLAM RViz view |

## Expected Topics

```text
/livox/lidar
/livox/imu
/odom
/odom_info
/map
```

Quick check:

```bash
ros2 topic list | grep -E 'livox|scan|odom|map'
```
