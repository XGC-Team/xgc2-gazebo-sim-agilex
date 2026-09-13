# Scout command dynamics

`command_delay_s` configures the transport delay in seconds. The node holds the last delayed command between updates.

The command path is a transport delay followed by a zero-order hold. Wheel PI, wheel inertia, and contact physics determine the dynamic response. There is no additional first-order filter or time-constant parameter.

Wheel PI, torque saturation, joint damping, inertia and wheel-ground contact remain active. Normal zero commands pass through the transport delay; emergency hold uses its separate queue-reset path.

Numerical command-path test:

```sh
python3 scout/test/test_inner_loop_delay.py
```

The test requires Python 3 and a C++17 compiler. It checks the command-dynamics header and the node's configuration call, not Gazebo contact dynamics.

Wheel commands use `velocity_controllers/JointVelocityController`, a forwarding controller. The wheel PI gains are loaded only under `gazebo_ros_control/pid_gains/<joint>`; the forwarding controller has no PID gain parameters.
