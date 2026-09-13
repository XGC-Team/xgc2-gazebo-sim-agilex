# Scout Mini Gazebo defaults

Accurate/process candidate (not a digital twin):

```text
wheelbase              0.451 m
wheel_track            0.416503 m
wheel_pid              P=2 I=8 D=0
max_step_size          0.004 s
real_time_update_rate  250 Hz
wheel_contact_fdir1    0 0 1
command_gain           1.04
angular_command_gain   0.80
command_delay_s        0.005
IMU topic              /ugv1/imu/data_raw (configured 100 Hz)
```

The command path applies transport delay and a zero-order hold. Wheel PI and
rigid-body/contact dynamics determine the response; no extra first-order
input model is present. Chassis limits are `v ∈ [-1.5, 1.5] m/s` and
`ω ∈ [-0.5235, 0.5235] rad/s`.

Debug chain: `memory/now/scout-gazebo-control.md` (`xgc2-dev-memory`).
