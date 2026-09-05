# Scout simulation control contract

`cmd_vel` reception queues timestamped input. A 100 Hz wall-clock scheduler
advances the delayed, first-order plant using ROS simulation time and publishes
wheel targets even when no new input arrives. A final zero command therefore
completes its configured delay and response without a second publication.
Paused simulation time does not advance the filter; time rewind discards old
queued and filtered commands. Gains and command limits retain their existing
values. The pending history retains at most the newest 2048 samples.

HOLD transitions, input admission and wheel-target publication share one gate
transaction. HOLD clears pending and filtered commands; release does not replay
them. The acknowledgement concerns command admission/zero targets, not an
instantaneous physical stop. Shutdown stops ROS callbacks, then unregisters and
drains the UDP callback before destroying its owner.

Robot spawning does not unpause the shared world. World/experiment orchestration
owns the decision to start its clock.

The existing UDP endpoint and multi-process routing are unchanged by these
control/lifecycle fixes. They require separate validation; single-process gate
tests do not establish that multi-process requests reach the intended robot.
