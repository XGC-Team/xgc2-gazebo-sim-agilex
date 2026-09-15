# Experimental implicit four-wheel force backend

This PR changes physical forces AND the wheel-speed controller. It is not just
instrumentation. It is stacked on the selectable-model PR and requires the new
`implicit_wheel_contact.hpp` from XGC-Team/xgc2-math#5 (commit 4789e075).
The version 0.5.8 prerequisite alone is insufficient: use the companion commit.
Neither PR has been promoted to APT or deployed to a live station.

## Selection

`implicit_brush.launch` starts an isolated world and the new backend. For an
already running Gazebo world, use the installed entry:

```
roslaunch gazebo_sim_scout spawn_selectable.launch ns:=ugv1 dynamics_mode:=wheel_physics wheel_physics_backend:=implicit_brush
```

`wheel_physics_backend=ode` delegates to the existing `spawn_accurate.launch`
byte-for-byte, so the existing I-P and the simplified unicycle remain available.
`unicycle` with an `implicit_brush` wheel backend is rejected, not silently
accepted. Selection is at spawn time; stop/delete/respawn, not hot switching.
No rebuild is needed per selection once all libraries are installed.

The new dispatch entry deliberately leaves existing launch entry behavior and
its tests untouched. **The XGC process catalog PR #59 still invokes the old spawn
entry. Exposing this new backend in that UI requires pointing its command to
spawn_selectable.launch and adding wheel_physics_backend/brush parameters. This
PR does not claim that the existing XGC UI already exposes it.**

## Physical changes

The new backend owns motor AND tangential contact forces. It does not start
ros_control, the old four wheel controllers, the skid-steer allocator, or the
unicycle plugin. Wheel ODE tangential/torsional friction is disabled locally;
normal collision support and obstacle normal impulses remain in ODE. No global
world friction/solver setting changes. Optional passive drag is applied once by
the coupled kernel, not also by joint damping.

Each step obtains whole-assembly COM, mass and yaw inertia from actual link
inertias, including parallel-axis terms. Wheel spin inertias are projected on
actual joint axes. Collision cylinder radii, contact positions and wheel axis
signs are read from Gazebo. The kernel uses the exact planar spin moment arm at
the selected contact centroid, not mounting-joint track or an extra -R*Fx torque.

The new nominal allocator uses contact-center geometry with explicit unity
`brush_allocation_gain` and `brush_yaw_calibration`. The old 1.01/1.46 calibration
is NOT transplanted onto a new tire law. Those new scalars are calibration
parameters, not evidence that physical wheel-rate differences equal chassis yaw.
Changing the tire law requires new joint-response comparison before calibration.

The high-fidelity path keeps ONE 5 ms pure command delay and NO extra command
lag (`DelayedPlanarVelocity({delay,0,0})`). The new wheel impedance and real wheel
inertia themselves create dynamic response. See math PR #5's
`docs/implicit_wheel_contact.md` for constitutive equations, convex solve,
anti-windup, discrete work/energy proof and executed numerical tests.

## Normal feedback and co-simulation boundary

Contacts are collected on WorldUpdateEnd. Gazebo's raw Contact.hh documents
wrenches in world coordinates; the appropriate body1/body2 force is selected.
For each wheel, positive world-Z force is summed and a force-weighted contact
centroid retained. The next Begin uses this completed-step normal load, rejecting
stale samples and detached geometry. No constant mg/4 fallback is supplied.
Support changes reset tread state. Contact filtering/subscription is local to
this model; SetNeverDropContacts is not changed globally.

The kernel predicts velocity while solving forces; those velocities are never
written to Gazebo. Only `wheel->AddForceAtWorldPosition` and `joint->SetForce`
are used. ODE integrates actual motion. Thus obstacle impulses are not erased
by a following velocity command, unlike the intentionally simplified mode.

However, this is a **partitioned co-simulation**. Previous-step normal load,
ODE normal-contact smoothing, full 3D constraints and the reduced tangential
inertia can cause differences from the standalone kernel. Its discrete energy
proof does NOT establish the whole ODE/plugin system's stability. This must be
checked with real SDK compilation and joint-response/step-size replays.

## Validity, stops and diagnostics

Initial scope is static, near-horizontal hard support and near-planar motion.
Roll/pitch above 5 degrees, moving support, invalid parameters, nonfinite feedback,
unsupported step (>20 ms), bad wheel geometry or solver nonconvergence latch a
fault. A fault stops drive writes rather than silently changing model. This is
not a terrain/rollover/deformable-ground simulator, and the isotropic brush is
not a complete pneumatic tire or independently identified Scout firmware.

Normal Stop uses the delay then physical braking. Hold clears delayed commands
and motor tracking state even while paused; force writes remain on the physics
thread, and a held robot uses zero wheel references (not an instantaneous
kinematic halt). Reset clears prior-epoch contact/motor states. Fault/Hold are
not a replacement for real-robot safety systems. No real robot is touched.

Published `joint_states` are actual joints. `simulation/implicit_wheels` reports
four rows: physics time, target/measured wheel speed, normal load, applied Fx/Fy,
motor torque, **predicted** surface slip, tread deformation, motor error, KKT
residual and iteration count. `simulation/dynamics_fault` is latched and must be
checked alongside ground truth; a model existing in Gazebo is not proof it moves.

## Verification status

Executed locally: companion math 12,478 assertions including 1,200 randomized
energy/work cases, optimized and ASan/UBSan; 9 new source wiring tests; XML/Python
syntax and child-argument forwarding. Math/plugin uploaded blobs were checked
against local Git blob hashes. No full repository suite was run here.

`test/implicit_wheel_runtime.test` registers a real isolated Gazebo test for
loaded combined v/yaw motion, force and torque bounds, wheel tracking and single
message Stop. It is **authored but not run**: the current environment has no
ROS/Gazebo SDK/runtime. No real-vehicle bag was replayed. Do not claim improved
field RMS or production readiness; the simulator PR stays Draft.

References checked for adapter semantics: Gazebo Classic gazebo11 Contact.hh,
ContactManager.hh, Link.hh, SurfaceParams.hh and WheelSlipPlugin.cc. The official
physics tutorial distinguishes independent pyramid friction from coupled cone
friction: https://classic.gazebosim.org/tutorials?tut=physics_params . This backend
implements its own compliant disk-constrained tangential law rather than merely
renaming or switching ODE's global friction model.
