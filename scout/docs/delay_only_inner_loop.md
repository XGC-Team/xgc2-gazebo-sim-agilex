# Scout command dynamics

`command_delay_s` configures the transport delay in seconds. The node holds the last delayed command between updates.

The command path is a transport delay followed by a zero-order hold. Wheel I-P control, wheel inertia, and contact physics determine the dynamic response. There is no additional first-order filter or time-constant parameter.

Wheel I-P control, torque saturation, joint damping, inertia and wheel-ground contact remain active. Normal zero commands pass through the transport delay; emergency hold uses its separate queue-reset path.

Numerical command-path test:

```sh
python3 scout/test/test_inner_loop_delay.py
```

The test requires Python 3 and a C++17 compiler. It checks the command-dynamics header and the node's configuration call, not Gazebo contact dynamics.

Wheel commands use `scout_gazebo/WheelVelocityController` over the wheel transmissions' `hardware_interface/EffortJointInterface`. For each controller update, it applies

```text
effort[k] = clamp(effort[k-1]
                  + i * (target[k] - velocity[k]) * dt
                  - p * (velocity[k] - velocity[k-1]),
                  -wheel_effort_limit, wheel_effort_limit)
```

This is an incremental I-P controller: the integral term acts on velocity error and the proportional term acts on measured velocity. `p` and `i` are controller parameters. `wheel_effort_limit` and `wheel_velocity_limit` come from the URDF joint limits; the former bounds output effort and the latter bounds the accepted target velocity. There is no D gain, independent integral clamp, anti-windup flag, setpoint slew limiter, or extra actuator pole.

The fixed validated defaults are `p=1.8`, `i=10`, `wheel_effort_limit=6 N m`, and `wheel_velocity_limit=26 rad/s`. These are effective simulation parameters for the recorded Scout response, not claimed physical firmware gains.
