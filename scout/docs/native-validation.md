# Native validation of chassis HOLD

`scout/test/native/verify.py` runs the real model plugins of this product (Scout skid steer, unicycle and
implicit wheel) and of `gazebo-sim-mecanum` (ideal and high fidelity), the world plugin with its HOLD host and the
ROS data owner in one headless `gzserver`: a private ROS master, a private Gazebo master, a temporary Unix
socket, no `gzclient`, no `DISPLAY`. The test is the HOLD verification of the Gazebo side of the platform
(`xgc2_gazebo_scene/docs/chassis-hold.md` of `ros-noetic-xgc2-gazebo-scene` describes what is verified).

## Run

```sh
. /opt/ros/noetic/setup.bash   # plus the workspace that holds xgc2_gazebo_scene, gazebo_sim_scout and gazebo_sim_mecanum
# the test plugin that blocks the chassis command dispatcher
cmake -S scout/test/native -B build/native -G Ninja \
      -DSCENE_INCLUDE=<xgc2_gazebo_scene>/include -DSIMULATION_LIBRARY=<libxgc2_simulation_service.so>
cmake --build build/native
python3 scout/test/native/verify.py --plugins <devel or install lib dir> build/native \
        --mecanum-source <gazebo-sim-mecanum checkout> [--gzserver /usr/bin/gzserver] [--only cycle,no_replay,...]
```

The test needs `roscore`, `xacro` (it builds the Mecanum assets from `models/xgc2_mecanum_ugv/model.sdf.xacro`
without their visuals) and the Python 3 `rospy`. It uses its own ROS master port, Gazebo master port and ROS home,
stops `gzserver` with SIGINT (the exit code must be 0) and checks that none of its processes survives.

## Fixture

* The world: ODE at 1 ms with gravity and a ground plane (friction 1), the world plugin with a required
  `dispatcher-test` component, and `libxgc2_dispatcher_fixture.so`, which blocks the single command dispatcher of
  the world on a ROS message and opens it through a world extension route. It holds a seat that never joins the
  HOLD roster.
* Five robots created through `POST /v1/entities` as SDF: `scout-unicycle` (kinematic, floating), `scout-skid` and
  `scout-implicit` (a body with four wheels on revolute joints and tires on the ground), `mecanum-ideal` (kinematic)
  and `mecanum-hifi` (the wheels of the model of the checkout on the ground). All are driven at 0.3 m/s by a
  controller thread publishing `cmd_vel` at 20 Hz. The Scout assets are minimal bodies with the Scout's joint names,
  not the scout_description meshes; nothing here is calibrated ground contact.

## Checks

1. Five chassis models are created and follow `cmd_vel`; creating them adds no thread to `gzserver`.
2. The HOLD methods `Describe` and `State` (`POST /v1/call/xgc2.chassis.hold/<Method>`): shared instance, roster,
   feedback flags, the capability and its entities in the describe facts, closed request schemas, method and route
   errors, unknown robots, and the retired `/v1/chassis/*` and `/v1/runtime-policy` routes.
3. For each of the five models: engage (reply is `engaged`, repeated engage is `already_held`), the stages
   `zero_written` and `stopped` with ordered timestamps, no movement of the held robot while commands keep
   arriving, the other four robots keep moving, release with a wrong revision and with a stale instance is a
   `conflict` and leaves the robot held, release by revision succeeds, a replayed release is `not_held`, and new
   commands move the robot again. The implicit wheel model is judged by the speed of its wheels, not by its body
   (see "Findings"); it is `zero_written`, and `stopped` only if its motion shows rest.
4. Group engage of three robots and a partial release (one released, two conflicts).
5. Commands queued before a release: the dispatcher is blocked, `cmd_vel` is published, the robot is engaged and
   released, then the dispatcher is opened; the robot does not move, and a command received after the release
   moves it. Run for every model with the release before the dispatch, and for the skid steer and the ideal Mecanum
   with the dispatch while held.
6. A paused world: engage writes zero within milliseconds; the kinematic models reach `stopped` at once, the
   dynamic models stay `zero_written` (their velocity is frozen, which is not rest); a velocity pushed into a
   kinematic model is zeroed again; after the world runs again every robot reaches `stopped`; a release while paused
   succeeds and the robots move after the world resumes.
7. Deleting a held model removes it from the roster and from the entities of the capability (`unknown_robot`,
   `404` on `State`), creating it again restores
   the held state with the same revision and the new model does not move, for each model, and for a model that was
   not held; release by revision moves it.
8. Restart of the world host: a new instance, an empty roster, no persisted HOLD, a release for the old instance is
   a `conflict` and a transport-level `409`, a stale release does not release a held robot; `gzserver` exits 0 on
   SIGINT and removes the socket.

The test was also run against two deliberate defects, and failed as it should: admitting commands by their
dispatch time instead of their receipt time (check 5), and no periodic tick while the world is paused (check 6).

## Result

2026-10-10, Ubuntu 20.04, g++ 9.4, ROS Noetic, Gazebo Classic 11.15.1 (ODE), XRPC C++ 0.2.0 at `d33afa8`,
chassis-hold 0.2.0 at `fcfe8f4`: all 24 checks pass in ten consecutive runs (240 checks total), taking
59.2 to 72.0 seconds per run. Each run uses private master ports in 20000-40000 and its own ROS home;
none left a surviving process or collected an orphan. The dependencies were built into
`/home/node/workspace/build/gazebo-hold/prefix`, and all plugins were built against that prefix. This is a
source-tree result on the development machine: the Focal controlled-toolchain ABI, installed packages, the
real Scout and Mecanum description meshes, calibrated contact and deployment are not covered here.

## Findings

* **Implicit wheel contact load.** The plugin reads the normal load of a tire from `Contact::wrench`. With
  Gazebo 11.15.1 that wrench follows the rotating wheel link: the load seen by the plugin is a half sine of the
  wheel angle, so the tires grip for half of the wheel angles and a vehicle commanded to stop may slide on. This is
  independent of HOLD (it happens with the HOLD tick disabled); see `implicit_wheel_backend.md`.
* **Gazebo transport abort.** At SIGINT `gzserver` sometimes (twice in twenty shutdowns of this series) aborts
  inside Gazebo's own transport library (`boost thread: trying joining itself`, every frame in
  `libgazebo_transport`). The test reports it and does not count it as a failure; any other exit status fails.
The full-vehicle rostests were then run against an isolated prepared native world, its discovered ServiceRef
and the Python `xgc2_scene_runtime`. `unicycle_runtime.test` passes, including HOLD while paused.
`implicit_wheel_runtime.test` fails in setup waiting for all four normal loads to exceed 1 N;
`scout_runtime.test` fails before its HOLD cycles because positive yaw input produces about -0.64 rad/s
against the required positive yaw. Both failures reproduce with the preceding plugin build and foundation
libraries, and the drive physics was left unchanged. The two Mecanum rostests and all four scene rostests pass.
`test/test_source_rsp_clock.py` still needs the `simulation_service_ref_json` launch argument and was not
part of this native HOLD validation.
