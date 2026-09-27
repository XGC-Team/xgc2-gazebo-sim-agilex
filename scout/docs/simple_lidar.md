# Optional shared simple lidar

Scout can attach the shared `xgc2_simple_lidar` GPU ray sensor to `base_link`.
It is disabled by default. The existing `enable_lidar` and `enable_rslidar`
switches keep their current sensors and settings.

Enable it on one Scout with the default mount pose (`0 0 0.35 0 0 0`):

```bash
roslaunch gazebo_sim_scout accurate.launch enable_simple_lidar:=true
```

Set another pose with six space separated values, or enable both robots in the
two Scout launch:

```bash
roslaunch gazebo_sim_scout accurate.launch enable_simple_lidar:=true \
  simple_lidar_pose:="0.12 -0.03 0.41 0 0 1.57"
roslaunch gazebo_sim_scout multi_accurate.launch \
  ugv1_enable_simple_lidar:=true ugv2_enable_simple_lidar:=true
```

The sensor uses each model's `ns` for its ROS namespace, so its point cloud is
published under `/<ns>/simple_lidar/points` (for example,
`/ugv1/simple_lidar/points`). The two-robot launch also accepts per-robot pose
arguments `ugv1_simple_lidar_pose` and `ugv2_simple_lidar_pose`.
