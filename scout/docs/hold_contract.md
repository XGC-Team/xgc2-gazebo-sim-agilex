# Chassis HOLD

The Scout simulation takes part in chassis HOLD. The contract is the one of `xgc2-chassis-hold`
(`contracts/chassis-hold.md` of `libxgc2-chassis-hold-dev`); the Gazebo binding (the `xgc2.chassis.hold` capability of the
world host, dynamic roster, command admission, native tick, feedback) is described in
`xgc2_gazebo_scene/docs/chassis-hold.md` of `ros-noetic-xgc2-gazebo-scene`. This page lists only what the Scout
plugins contribute.

* The plugin element `chassisRobotId` is the entity ID. Each plugin (`libscout_gazebo.so`,
  `libscout_unicycle_plugin.so`, `libscout_implicit_wheel_plugin.so`) reserves it with
  `xgc2_gazebo_scene::ChassisHold` when it loads and joins the HOLD roster when its native setup is complete; it
  leaves when it is destroyed. A model that is created again under the same ID starts held if it was held.
* `cmd_vel` is subscribed on the world's command queue, which records when a message was queued. A command that
  is not admitted (the robot is held, or the command was queued before the last release) never reaches the
  plugin's delay line, however late it is dispatched.
* The control step of a plugin runs under the lock of its HOLD seat, so it never overlaps the zero output of the
  native tick. While the world is paused the tick runs on its own thread.
* Zero output, written on every tick while held:
  * skid steer: clears the delay line, wheel targets and I/P state and writes zero effort to the four wheel joints
    (the vehicle coasts);
  * unicycle: resets the drive state, writes zero velocity to every link (a rigid stop) and publishes zero velocity;
  * implicit wheel: clears the delayed command and the drive-error memory and writes zero effort; the wheel
    model keeps running with a zero command, so the motors brake.
* Feedback for `stopped` is the horizontal speed and yaw rate of the model.

Zero output is not a physical stop: inertia, contact and slope are separate facts, and `stopped` exists only
because the model's velocity is observed. See `native-validation.md` for the test that exercises all of this.
