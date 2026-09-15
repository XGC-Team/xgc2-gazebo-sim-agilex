# Gazebo 11 ODE contact-load adapter repair

Related: [harness #101](https://github.com/XGC-Team/xgc2-harness/issues/101).
This change repairs contact conversion and feedback acquisition only. It does
not change any default plant, APT configuration, controller, constitutive
parameter, motor gain, allocation gain or command delay. The live/default plant
remains unicycle. Do not close the high-fidelity gate from these source tests.

## Why the wheel spun without traction

Gazebo Classic `ODEPhysics::UpdatePhysics` writes each body's contact force in
that body's **link frame**, using `WorldPose().Rot().RotateVectorReverse(f)`.
The old adapter used the selected force's local Z component as world support.
On the Scout, wheel-link local Z is the axle, so an upward contact force can
have essentially zero local Z. This gave the tire kernel zero normal load and
therefore zero allowable tangential traction.

`World::Update` synchronizes ODE dirty poses only after `UpdatePhysics`, before
`worldUpdateEnd`. Restoring the force using a newly read pose at UpdateEnd is
also wrong. The adapter now snapshots the actual wheel **link** rotation at
`beforePhysicsUpdate` (after model updates and collision detection, before ODE
integration), associates it with simulation time, and consumes it once at End.
It selects body1/body2 by collision identity, checks Contact.time against both
the snapshot and collection time, rotates to world, then projects on gravity-up.
Raw force, rotation, world force and projection are checked before clipping.
Downward or zero support stays zero; no absolute-value/norm/mg fallback exists.
Pause transitions, model reset, time reset and rewind invalidate old feedback.
The existing completed-step age and separating-wheel geometry checks remain.

Source contracts inspected for this repair:

- [ODEPhysics.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ode/ODEPhysics.cc), blob `7067861457a3ea2292b3659251ca76485045e302`.
- [World.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/World.cc), blob `6d36124fb0be00f76dbad475cf40ff129b76a174`.
- [ContactManager.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ContactManager.cc), blob `e0fd0ee252a6471f286acb7e8f0a95730c9ab6b3`.

## Feedback ownership and diagnostics

The plugin registers its four actual `CollisionPtr`s through the map overload
of `CreateFilter`, avoiding deferred name resolution. It checks filter and
subscription creation, retains the local subscriber and removes only its own
successfully created filter on destruction. `ContactManager::NewContact` in
Gazebo 11 allocates feedback for a matching custom publisher; it does not need
an extra probe's global `SetNeverDropContacts`. Losing the owned filter faults
rather than silently continuing free-spin.

The existing `simulation/implicit_wheels` 4 x 14 layout is unchanged. A separate
`simulation/implicit_contacts` Float64MultiArray has four rows (FL, FR, RL, RR)
and 23 columns named in `layout.dim[1].label`:

```
sim_time,frame_time,contact_first_time,contact_last_time,contacts,
matched_pairs,samples,static_horizontal,accepted,
discard_no_frame,discard_time,discard_nonfinite,discard_nonhorizontal,
discard_moving,discard_nonpositive,
raw_Fx,raw_Fy,raw_Fz,world_Fx,world_Fy,world_Fz,normal,filter_registered
```

Counts are per completed physics step, not cumulative. `contacts` is the whole
manager count, repeated in each row. Raw/world forces are signed sums over the
wheel's eligible finite, time-matched support samples, including rejected
nonpositive samples; `normal` sums only positive upward support. Timestamps
are physics time, never ROS wall-clock time. `-1` means no timestamp. Rejected
geometry is counted before force conversion. Collection `normal` is not a
promise of next-step use: the existing age/separation checks may still reject
that patch before actuation; compare against `implicit_wheels.normal`.

## Tests and evidence boundaries

Portable production-adapter test:

```bash
python3 -m unittest discover -s scout/test -p 'test_ode_contact_feedback.py' -v
g++ -std=c++11 -Wall -Wextra -Werror -pedantic \
  -fsanitize=address,undefined -fno-omit-frame-pointer -Iscout/include \
  scout/test/ode_contact_feedback_test.cpp -o /tmp/contact-feedback-test
/tmp/contact-feedback-test
```

Executed in the source-only repair environment: four Python tests passed;
627 numerical checks passed both normally and with ASan/UBSan. They cover both
collision owners, both wheel installation sides, wheel angle/chassis yaw,
pre-step versus latest rotation, signed/zero support, NaN/infinity/overflow,
1/2/4 ms timestamp mismatches, reset epochs, and source wiring. The original
plugin was reconstructed from the connector and verified against its exact
Git blob `9ea98026f19cc2b66541e0e9cc0961bad8c1abaf` before editing.

Isolated physical regression (requires a built Noetic workspace):

```bash
rostest gazebo_sim_scout implicit_contact_feedback.test step_size:=0.004
# Repeat in fresh isolated masters with step_size:=0.001 and :=0.002.
```

The new CI job builds the actual Scout plugin against source math
`4789e075acd54e8df565763124b59ce4b604f352`, not an assumed unreleased APT math
package. It runs math self-tests separately, then the no-probe physical test.
It retains source/binary hashes, Gazebo version, ROS/test logs and the runtime
summary as artifacts. This file does **not** assert that those CI jobs passed;
consult the actual PR check results and artifacts.

The physical test is a **synthetic** 20 s sequence: 2 s rest, a single 0.3 m/s
command for 8 s, then a single Stop for 10 s. It compares summed stationary
support to the assembly's queried weight (not equal per-wheel allocation),
requires actual XY displacement and nonzero bounded tire force, and checks
stop, pause/lift-off and reset. The latter lifecycle checks run after the 20 s
sequence. It does not gate on precise wheel-speed tracking, which belongs to
the separate predicted-memory P1.

The repair environment has no ROS/Gazebo SDK or Docker, so the physical test is
not claimed as locally executed. Discovery explicitly skips it outside its
rostest entrypoint, including source-only CI with ROS Python installed.

Still required before high-fidelity acceptance: the original **520-command,
20 s prefix of `hybrid-dmpc-20260912`**, with the same isolated Noetic image and
4 ms physics as the reported failure; first without any extra probe, then
with the independent probe for load comparison. Preserve raw diagnostics,
source/binary/image identities, loads, forces, body velocities/displacements
and stop/lift/reset results. The reported ~272 N total and ~60-70 N per wheel
are order-of-magnitude references, not four equal-load constraints.

`memory_=result.memory` is deliberately unchanged. Restoring normal feedback
does NOT resolve the separate P1 about committing unobserved predicted memory.
No field-accuracy, full-run RMS, rollout or live-station approval is implied.
