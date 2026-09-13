# Scout Mini Gazebo defaults

Accurate/process candidate (not a digital twin):

```text
wheelbase              0.451 m
wheel_track            0.416503 m (mounting joint separation)
collision_midpoints    0.495779688 m apart
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

The retained wheel mesh has its tire midpoint at local axial z=0.039638344 m.
The collision uses the same axial offset; the visible assembly and joint frames
remain fixed. The cylinder approximates the tire envelope but omits its crown.
`wheel_separation` belongs to command allocation, whose differential-to-common
wheel-speed ratio uses `angular_command_gain * wheel_separation`. It is not a
measurement of the tire midpoint separation or the effective rolling track.
The retained allocation gains require validation with the corrected contacts.

Debug chain: `memory/now/scout-gazebo-control.md` (`xgc2-dev-memory`).
