# Scout simulation command ownership

User cmd_vel is ROS data. Every drive mode owns acceptance and actuation inside
its Gazebo ModelPlugin. Wheel physics owns four native joints and applies the
existing differential allocation and xgc2_math::wheelVelocityIPStep law, with
unchanged default P=1.8, I=10, gains, URDF effort/velocity limits and 5 ms input
delay. The independent skid controller and ROS motor target pipeline are retired.
Unicycle and experimental implicit-brush retain their existing plants.

Physics updates advance the plant in native simulation time. A final zero input
needs no second publication. Paused time does not advance the plant; reset and
time rewind discard old queued/filtered commands. The skid delay is a fixed
array retaining at most the newest 2048 samples, with no native deque allocation.
Full-vehicle contact response and field alignment remain scientific acceptance
gates.

HOLD is described in `hold_contract.md`: a command is admitted by the time it was queued, the control
step and the zero output of the HOLD tick share one lock, and release never restores a cached command.
ROS namespaces may differ from explicit public chassis identities.

Spawning requires an explicit simulation-v1 ServiceRef and target. It submits one native creation
operation and waits for its receipt.
Spawning does not unpause the world. Gazebo start/stop remains explicit process
management; domain control starts after the process and native plugin exist.
