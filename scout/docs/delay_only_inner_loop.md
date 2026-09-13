# Delay-only command path for the wheel-PI plant

The Scout node retains `command_delay_s` and the existing delayed zero-order hold. It configures the command dynamics with zero additional first-order speed lag. Wheel PI, torque saturation, joint damping, inertia and wheel-ground contact remain responsible for the plant response.

`command_time_constant_s` is accepted only for migration of old launch/snapshot files: a nonzero value produces an explicit warning, is not applied, and the startup summary reports the effective value zero. Existing stored experiments are not rewritten. The generic CommandDynamics class is unchanged and may still be used in tests with nonzero lag; that is not the active Scout policy. Network delay is not changed or calibrated by this patch.

This is not a claim that the physical vehicle has no lag, and does not remove contact forces, collision, wheel torque limits, emergency hold, or the delayed execution of a normal zero command. Emergency hold retains its separate existing immediate queue-reset path.

## Local tests

`python3 scout/test/test_inner_loop_delay.py` compiles the actual CommandDynamics header and checks transition time, no added filtering, one-shot stop, reverse motion, repeated/rewound clocks and reset. A source guard checks the production Configure call. This is not a Gazebo dynamics or ROS callback integration test.

## Structural audit before numerical fitting

Check the final generated SDF, not only launch parameters. In particular, slip belongs under collision/surface/friction/ode. Preserve the historical distinction between configured and effective slip: the September 8 surface fix and September 9 zero-slip-baseline restoration changed this boundary. Zero ODE force-dependent slip does not mean zero physical skid under finite Coulomb friction.

Verify common-frame wheel-axis signs, wheel radius, geometric track, transmission mapping, number/location of contacts, normal loads, body/wheel mass and inertia, joint damping, wheel torque saturation and PI integral clamp. Reduce physics step size and check response convergence before identifying solver artifacts as plant dynamics. Do not fit yaw by moving physical wheel geometry or forcing zero lateral contact velocity at all four wheels.

Only after these checks compare common-point body longitudinal/lateral/yaw responses under pure and joint inputs against independent physical data. A closed-loop formation run alone is not an open-loop identification experiment.

## Merge/release gates

Full ROS/Gazebo build and existing surface/control tests, identical-input replay, stopping/turning and multi-vehicle clearance acceptance remain required. Do not promote a passing numerical test to real-vehicle or station acceptance. Package revision, runtime locks and live experiments are intentionally not changed here.
