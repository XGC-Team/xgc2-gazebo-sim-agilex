# Simulation ground-truth telemetry contract

`gazebo_model_ground_truth` selects `model_name` from `gazebo_msgs/ModelStates`
and publishes `simulation/ground_truth/pose` and `simulation/ground_truth/twist`
by default. Canonical robot pose/twist remain owned by the Robot Adapter; this
node does not publish chassis odometry or replace an estimator.

## Time and reset semantics

`ModelStates` has no source header. Both outputs use the same **callback
reception-time ROS stamp**. This is simulation time when `/use_sim_time` is set,
not a reconstructed physics-step timestamp. A queued message can predate that
stamp, including around a reset; this bridge cannot prove source-time alignment
or distinguish an old queued sample from a newly generated one.

A zero ROS clock produces no measurements. A positive clock stamp is published
at most once within an observed clock epoch. A backwards clock observation,
including a return to zero, clears the prior publication watermark. The next
valid positive-time sample can be published immediately, without waiting for
the previous experiment's final time. Clock observations are tracked even when
the selected model is absent or the sample is rejected.

Pose and twist from one accepted callback share a stamp and frame, but the two
ROS topics are not an atomic transport. Consumers must pair them and separate
experiment/clock epochs; equal numeric stamps across resets are not the same
measurement. Consumers must also clear their own old-epoch buffers.

## Data and frame semantics

The name, pose and twist arrays must have equal lengths. Every selected pose
and twist component must be finite, and the quaternion must have a finite,
nonzero squared norm. Invalid samples publish neither output and do not consume
a publication stamp; a subsequent valid sample at that time remains eligible.
No invalid value is replaced with a plausible zero or a fabricated measurement.

The node copies the selected Gazebo pose and twist without a coordinate or
reference-point conversion. `world_frame` labels that existing world basis; it
does not perform a transform. In particular, twist is **not** signed body-forward
velocity. Comparing it with a tracker state requires checking the Gazebo model's
reported reference point against the tracker's point and applying the appropriate
frame/point transform. Wheel-derived `scout_status` is not interchangeable with
this telemetry when wheel slip is present.

## Regression and acceptance boundary

Run `python3 scout/test/test_ground_truth.py -v`. The test compiles the actual
production C++ translation unit against small ROS API doubles, then replays
clock resets, pauses, missing models, malformed arrays and invalid numbers.
`CXX`/`CXXFLAGS` can select another compiler or enable sanitizers, for example:

```sh
CXXFLAGS='-g -fsanitize=address,undefined -fno-omit-frame-pointer' \
  python3 scout/test/test_ground_truth.py -v
```

The test is registered with CTest and discovered by the existing control
regression workflow. It proves callback/data behavior, not ROS transport timing,
Gazebo contact dynamics, simulator-versus-hardware calibration, or station
installation. No plant parameter, wheel allocation, controller law, DMPC weight,
or robot command is changed by this telemetry contract.
