# Scout Mini Gazebo defaults

Accurate/process candidate (not a digital twin):

```text
wheelbase              0.451 m
wheel_track            0.416503 m (mounting joint separation)
collision_midpoints    0.495779688 m apart
wheel_controller       incremental I-P, P=1.8 I=10
max_step_size          0.004 s
real_time_update_rate  250 Hz
wheel_contact_fdir1    0 0 1
wheel_contact_mu1/mu2  0.16 / 1.0
wheel_effort_limit     6 N m
wheel_velocity_limit   26 rad/s
command_gain           1.01
angular_command_gain   1.46
command_delay_s        0.005
IMU topic              /ugv1/imu/data_raw (configured 100 Hz)
```

The command path applies transport delay and a zero-order hold. Wheel I-P and
rigid-body/contact dynamics determine the response; no extra first-order
input model is present. Chassis limits are `v ∈ [-1.5, 1.5] m/s` and
`ω ∈ [-1.0, 1.0] rad/s`, matching the verified Scout SDK command interface.

These are fixed effective-model parameters selected from the full 409 s
physical command replay. They do not claim that the unobserved physical motor
controller uses I-P or that the effort and friction values are uniquely
identified hardware constants.

The retained wheel mesh has its tire midpoint at local axial z=0.039638344 m.
The collision uses the same axial offset; the visible assembly and joint frames
remain fixed. The cylinder approximates the tire envelope but omits its crown.
`wheel_separation` belongs to command allocation, whose differential-to-common
wheel-speed ratio uses `angular_command_gain * wheel_separation`. It is not a
measurement of the tire midpoint separation or the effective rolling track.
The retained allocation and contact gains are validated together for the
recorded maneuver range; they are not separate hardware measurements.

Debug chain: `memory/now/scout-gazebo-control.md` (`xgc2-dev-memory`).
