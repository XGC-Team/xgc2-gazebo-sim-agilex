# Native simulator HOLD

Scout ModelPlugins bind `xgc2_gazebo_scene::ChassisBinding` to their native world
and explicit `chassisRobotId`. That ID must equal the public simulation-v1
entity ID and occur in the prepared world's `chassis_robot_ids` grant. The ROS
namespace routes user data. Missing authority, unknown identity, duplicate
binding or failed native initialization fails creation. `Ready()` runs only
when the complete native owner is usable.

One world provider/listener owns the HOLD domain, with at most 16 chassis IDs.
Robots open no control socket or control thread. `GET /v1/chassis/hold` reads the
applied revision; `POST` supplies `expected_revision` and explicit
`{robot_id, held}` changes. Applied HOLD includes completed zero-sink status.
Release clears old state too. The old local UDP control entry is removed;
physical firmware is outside this product.

Command admission, delayed targets, I/P state and native joint/velocity writes
share one `with_command` transaction. Admission tries the world domain mutex
once and skips a contended sample. It never waits or copies an old command for
later execution. The domain always precedes vehicle state. HOLD clears pending
commands, filtered state and native effort/velocity outputs even when physics
is paused. Zero effort does not remove physical inertia or establish an
instantaneous physical stop.

User cmd_vel, joints, truth, status and sensors retain ROS data interfaces.
One fixed shared world dispatcher serves model subscriptions; models create no
ROS queue/spinner threads. Owner-specific queue entries carry an epoch. HOLD,
release and reset invalidate it in constant time. Old entries still drain the
underlying SubscriptionQueue but skip their action, including when dispatch
resumes only after release. Command admission rechecks the epoch inside the
world transaction. A fresh command after draining remains usable.

Delete retires/zeros before Gazebo link Fini. Models then disconnect updates,
close subscriptions, drain admitted callbacks and unregister bindings. Binding
destruction never dereferences finalized joints. Gazebo start/stop remains an
explicit process workflow; native plugins load with the prepared world. The
provider does not start Gazebo. See `native-validation.md` for evidence and gates.
