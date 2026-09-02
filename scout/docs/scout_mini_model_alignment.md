# Scout Mini Gazebo defaults

Accurate/process candidate (not a digital twin):

```text
wheelbase              0.451 m
wheel_track            0.490 m
wheel_pid              P=2 I=0 D=0
max_step_size          0.004 s
real_time_update_rate  250 Hz
wheel_contact_fdir1    0 0 1
command_gain           1.10
angular_command_gain   1.15
command_delay_s        0.15
command_time_constant  0.15
IMU topic              /ugv1/imu/data_raw at 100 Hz
```

NMPC input bounds match the chassis clip: `v ∈ [-1.5, 1.5] m/s`,
`ω ∈ [-0.5235, 0.5235] rad/s`. Default circle is a direct circle, not
`CIRCLE_ENTRY`.

Debug chain: `memory/now/scout-gazebo-control.md` (`xgc2-dev-memory`).
