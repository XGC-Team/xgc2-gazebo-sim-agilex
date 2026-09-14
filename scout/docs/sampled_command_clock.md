# Sampled command-clock contract

Related task: XGC-Team/xgc2-harness#93. This change is not a model-calibration or whole-chain acceptance claim.

## Time and units

`TwistCmdCallback` observes receipt `t_r` in ROS simulation seconds. `CommandDelay::Push` stores deadline `t_d=t_r+0.005 s` for the #93 frozen configuration. On observed control clock `t_k`, `Advance(t_k)` consumes all commands with `t_d<=t_k`, preserving the newest due command. The wheel target remains zero-order held between dispatches. There is no input time constant, first-order actuator lag, future PVA queue or new control mode.

Previously the deadline used simulation time but the dispatcher used a 0.010 s **wall** timer. At constant real-time factor rho, its ideal simulation sampling interval was `rho*0.010 s`. Ignoring ROS scheduling, a command received on a poll at zero with a 5 ms deadline would first dispatch at 10 ms for rho=1 and 40 ms for rho=4. This clock-domain dependence is separate from wheel PI or contact dynamics.

The dispatcher now uses a 0.001 s ROS-time timer. With regularly delivered simulation clock steps h in {0.001,0.002,0.004} s, the arithmetic release is the first observation at or after the deadline. Its ideal quantization lies in `[0,h]`, not in `[0,rho*0.010]`. Floating-point deadline comparison can place an exactly coincident boundary on the next observation; tests include that endpoint. Callback queuing, `/clock` publication rate and `gazebo_ros_control` update order can add delay. **Do not label the body response or wheel actuation exactly 5 ms delayed.** Log receipt, wheel target publication, PI update and physics step separately.

When paused, ROS time and plant dynamics do not advance. The existing UDP HOLD invokes `HoldZeroThunk -> PublishZeroMotors` directly and clears the delay queue without relying on the timer. The existing queue rewind behavior is retained; this is not a new Reset feature.

## Verification

Run `python3 scout/test/test_inner_loop_delay.py`. It compiles the actual `CommandDelay` header with warnings as errors and checks FIFO/stop/reverse/pause/rewind behavior, sampled 1/2/4 ms grids, phase offsets and wall dilations. The source guard checks ROS-time timer wiring and the direct HOLD thunk. This does **not** execute roscpp or Gazebo.

Before adoption, on the frozen installed-image comparison: record `/clock`, incoming `cmd_vel`, all four wheel target topics and step-level physics records at h=1/2/4 ms; repeat at different real-time factors. Compare simulation-time receipt-to-target distributions (including tails, not only means), check a final single zero command and HOLD during pause, and run the unchanged fixed-PVA/four-vehicle/full Experiment gates. Reject missing target messages, hidden queue accumulation, heartbeat-dependent stop, or unexplained real-time-factor-dependent shifts. Measure the increased ROS publication load; this PR does not claim performance neutrality.

No PI, torque limit, allocation gain, friction setting, production steering weight, feasible set, low-speed tracking law or SDK command clamp changes are made here. A frozen 5 ms configuration remains mandatory for #93; this reusable class still permits other explicitly configured experiments.
