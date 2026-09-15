# Scout selectable dynamics

## Contract

The installed Scout package contains both plants. `dynamics_mode` is a **spawn-time** enum:

| Value | Horizontal plant | Input shaping |
| --- | --- | --- |
| `wheel_physics` (default) | Existing four wheel effort/I-P loops and Gazebo contact | Existing `command_delay_s=0.005`; **no additional input lag** |
| `unicycle` | Prescribed planar rigid-body velocity field | One shared `command_delay_s=0.005`, followed by separate `unicycle_linear_time_constant_s=0.005` and `unicycle_angular_time_constant_s=0.005` |

For unsaturated simplified commands, both channels have unity DC gain:

```text
v(s)/v_cmd(s) = exp(-d*s)/(1 + tau_v*s)
w(s)/w_cmd(s) = exp(-d*s)/(1 + tau_w*s)
x_dot = v*cos(yaw); y_dot = v*sin(yaw); yaw_dot = w
```

A time constant is not a second pure delay. At 10 ms after a received unit step,
with d=tau=5 ms, the mathematical state is 1-exp(-1)=0.6321205588, not 1.
`xgc2_math::DelayedPlanarVelocity` splits at command release events and uses exact
ZOH exponential propagation. It is not an explicit-Euler filter. Physics outputs
are still applied on the Gazebo update grid, and ROS reception is timestamped by
the latest observed simulation time (up to one physics-step quantization). This
is not a claim of continuous-time 5 ms physical actuation between physics ticks.
Both simulation and hybrid runs use **Gazebo simulation time** for this plant;
ROS message headers continue to follow the existing measurement clock contract.

The simplified mode never consumes the high-fidelity command/steering calibration
gains. It never starts ros_control, wheel effort controllers or the skid-steer
allocator. Wheel tangential friction/slip and wheel-joint damping are zero in this
mode only, so no second wheel-ground plant competes with the prescribed velocity.
Each link CoG receives `v_O + omega cross (p_CoG-p_O)`, not the same linear velocity
for every link. No pose is teleported. Geometry, sensor creation, measured Gazebo
ground truth, status and source robot-state-publisher interfaces are retained.

**Scope:** simplified means flat-ground/free-motion algorithm testing, not a
validated tire or collision-force simulator. Collision geometry and the solver
remain present, but prescribed velocity can overwrite impulse-induced velocity
on the next tick. Do not use this mode to validate impact, traction, slopes,
rollover or wheel-slip estimation. `/simulation/drive/velocity` is the mathematical
drive state; `/simulation/ground_truth/*` remains the actual Gazebo state.
`joint_states` in simplified mode has `frame_id=unicycle_virtual_wheels`, virtual
wheel angles/velocities and no effort values. These states support existing
status/RViz consumers, not measured wheel dynamics; physical wheel animation is
not used to produce motion.

Normal zero `cmd_vel` traverses the delay and lag and needs no repeat heartbeat.
UDP Hold clears the delay queue and both lag states immediately, including while
paused. It does not call Gazebo physics APIs from the UDP thread: the first
physics update after resume applies zero under the same admission gate. World
reset/time rollback clears old-epoch state. Resume does not resurrect old commands.

## Selection and installation

```bash
roslaunch gazebo_sim_scout accurate.launch dynamics_mode:=unicycle
roslaunch gazebo_sim_scout accurate.launch dynamics_mode:=wheel_physics wheel_diagnostics:=true
roslaunch gazebo_sim_scout multi_accurate.launch ugv1_dynamics_mode:=unicycle ugv2_dynamics_mode:=wheel_physics
```

`simple.launch` now defaults to `unicycle`; all other existing entry defaults stay
wheel physics. Unknown enum values fail rather than silently selecting a plant.
Stop/delete and respawn the robot to switch; mid-motion hot switching is not
implemented. There is no mode-specific build or release. The **first** deployment
needs math >=0.5.8 and a Scout build containing both new shared libraries on the
Gazebo server; subsequent selection only changes parameters and respawns.
Do not claim that an older installed package already contains these plugins.
XGC's process-catalog change exposes the same enum and time constants; saved
ProcessDefinition snapshots/overrides must be reviewed when consuming it.

## Wheel-physics audit and diagnostic contract

The physical I-P law and contact coefficients are not retuned by this patch.
The independent XGC catalog had old PI-era gains, calibration, limits and
undeclared D/I-clamp/antiwindup arguments. That source wiring is repaired in the
companion catalog PR. It is a confirmed configuration defect, **not proof that
the live station used those values or that real/sim response is now aligned**.

For a planar reference with velocity `(u,v,r)`, wheel position `(x_i,y_i)`, and
signed rolling speed `R*Omega_i`, use:

```text
s_xi = R*Omega_i - (u-r*y_i)
s_yi = v+r*x_i
P_contact_i = -F_xi*s_xi + F_yi*s_yi
```

`P_contact_i<=0` is the dissipative-contact sign check, not a stability theorem
for a sampled saturated controller. The formulas contain no division by forward
speed and stay finite at standstill. Fixed front/rear wheels cannot both satisfy
`s_y=0` when `r!=0`: subtraction gives `r*wheelbase=0`. Thus accurate wheel-speed
tracking alone cannot establish accurate chassis yaw/translation.

Under equal wheels, weak mean longitudinal-slip change and weak `r*v`, the common
mode approximately obeys `J_eff*dOmega_c/dt=tau_c-b*Omega_c`, with
`J_eff=J+m*R^2/4`. Unsaturated continuous I-P gives

```text
H_common(s) = Ki / (J_eff*s^2 + (b+Kp)*s + Ki)
low-frequency lag scale = (b+Kp)/Ki = (0.1+1.8)/10 = 0.19 seconds
```

This explains how a nearly fixed forward response can survive scalar wheel-loop
tuning. It is an approximation, not identification of real firmware or proof of
Gazebo's discrete stability. Steering must influence the common mode through
slip/contact/load dynamics; multiplying a fixed command gain cannot supply an
absent state-dependent mechanism. For left/right mean slips:

```text
u = R*(Omega_R+Omega_L)/2 - (s_R+s_L)/2
r = [R*(Omega_R-Omega_L)-(s_R-s_L)]/B_contact
```

The current mounting-joint track is 0.416503 m. The collision cylinders are each
offset outwards by 0.039638344 m, giving nominal center track 0.495779688 m on flat
ground. Do not confuse mounting track, contact-center track, and an identified
effective skid-steer track. The allocator multiplies BOTH modes by `command_gain`
and additionally the yaw mode by `angular_command_gain`; their product matters.
This patch does not erase intentional calibration by replacing one track number.

Gazebo's `pyramid_model` solves independent tangential friction directions; a cone
uses coupled dynamic friction. Neither naming choice establishes a complete tire
model. `slip1/slip2` are force-dependent slip (m/s/N), not input time constants.
See the [official physics parameter description](https://classic.gazebosim.org/tutorials?tut=physics_params).
No static-friction/combined-slip law has been silently replaced, and no global
world setting affecting other robot types is changed.

With `wheel_diagnostics:=true`, the read-only plugin publishes
`/<ns>/simulation/wheel_kinematics`: four rows (FL, FR, RL, RR), twelve columns:

```text
sim_time_s,target_rad_s,measured_rad_s,joint_effort_nm,x_m,y_m,
rolling_m_s,ground_x_m_s,ground_y_m_s,slip_x_m_s,slip_y_m_s,axis_body_y
```

It uses collision-center positions relative to the base CoG and actual joint
axis signs, not mounting positions. It is a nominal flat-ground planar diagnostic,
not an ODE contact-patch/normal-force sensor. `GetForce(0)` reports joint actuation,
not measured tire-ground force. Targets are NaN until observed. Compare target vs
measured wheel speed separately from measured rolling speed vs chassis motion;
do not label either residual with a single unexplained gain. Physical mode is
published latched even when detailed diagnostics are disabled.

## Validation and open gates

Locally runnable checks:

```bash
# In the companion math source tree:
g++ -std=c++17 -Wall -Wextra -Werror -pedantic -Iinclude test/delayed_planar_velocity_test.cpp -o /tmp/scout-math-test
/tmp/scout-math-test
# In this source tree:
python3 scout/test/test_dynamics_modes.py
```

The math tests check analytic delay/lag/Stop, independent channels, event splitting,
step-size invariance, pause coalescing, reset/rewind, invalid inputs and contact
kinematic/power identities. The Python suite checks source wiring and ownership,
not a ROS or Gazebo experiment. Existing wheel defaults and controller law stay
unchanged. This submission environment has no ROS/Gazebo SDK/runtime; full plugin
compilation, xacro/SDF expansion, shared-world mixed modes, paused-Hold/resume,
physics reset, step-size replay and joint-input field comparison remain acceptance
gates. No live station, physical robot, estimator, DMPC or APT release is changed.
High-fidelity response alignment remains open.

A real, separately marked Gazebo regression is registered as
`test/unicycle_runtime.test` (port 11369, isolated test container only). It checks
measured joint-input DC response, single-message Stop, and paused UDP Hold/resume.
Its Python syntax was checked here, but the test itself was **not executed** in
this environment. Run `rostest gazebo_sim_scout unicycle_runtime.test` after
installing/building the dependencies. Do not run it against a live station.
