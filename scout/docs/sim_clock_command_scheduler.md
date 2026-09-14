# Command release clock and wheel allocation

Related task: XGC-Team/xgc2-harness#93. This change is not a declaration of
sim-to-real calibration or four-vehicle motion acceptance.

## Source finding and minimal change

At baseline ce694ff4c83295b3e0132e652b4f7eed7cb9be5b,
`ScoutSkidSteer::SetupSubscription` used a 0.010 wall-second timer while
`CommandDelay::Push/Advance` used ROS/simulation seconds. With locally constant
real-time factor rho, the ideal release lattice spacing was rho*0.010 simulation
seconds. For receipt t_k and fixed transport delay d=0.005 s, a due command was
released at the first tick t_m >= t_k+d. Thus the extra wait was in
[0, rho*0.010), before ROS callback and hardware sampling latency. The configured
5 ms queue was not the entire measured input delay.

Normal release now uses a ROS timer (`control_period_s=0.001` simulation s).
The existing wall-time safety hold has its own timer and can only publish zero;
it never advances the delay queue or releases normal motion. The UDP hold path
is retained. No first-order input response, wheel PI change, controller mode,
contact coefficient or production gain change is introduced.

The ideal scheduling bound becomes [0, control_period_s). This is NOT an exact
end-to-end 5 ms guarantee: /clock publication cadence, callback scheduling,
transport to velocity controllers, and gazebo_ros_control sampling remain
observable contributors. Freeze and record them in local experiments.

## Allocation, units and signs

Body +x is forward and +z yaw is counterclockwise. With incoming clipped v [m/s],
w [rad/s], radius r [m], allocation separation b_a [m], common gain g [1] and
angular gain g_w [1], the retained mapping is

    Omega_L = g*(v - g_w*b_a*w/2)/r
    Omega_R = g*(v + g_w*b_a*w/2)/r.

`AllocateWheelSpeeds` returns FR,FL,RL,RR joint angular velocities [rad/s].
The left/right URDF joint transforms give the same positive rolling direction.
For motor-side RPM and gear ratio G, RPM=60*G*Omega/(2*pi); G cannot be silently
assumed to be one when comparing CAN feedback. The effective allocation span
b_a*g_w is not the geometric wheel-contact track. In particular,

    r*(Omega_R+Omega_L)/2 = g*v
    r*(Omega_R-Omega_L) = g*g_w*b_a*w.

These identities are now exercised through the same header called by production,
not a separate Python implementation. The mapping is algebraically unchanged.

## Executed validation

`python3 scout/test/test_sim_clock_scheduler.py` compiles the production delay
and allocation headers with C++17, -Wall -Wextra -Werror. It checks 1,953 signed
allocation cases, 1,000 receipt phases at each of rho=0.1/1/10, invalid inputs,
pause, a single final zero, reversal, clock rewind and hold reset. Ideal maximum
extra waits were 0.99983/9.98783/99.9838 ms before and 0.99983 ms in all three
new-lattice cases. These are mathematical scheduler inputs, not ROS/Gazebo runs.

## Local acceptance still required

Use the existing isolated model-validation runner, not the shared lifecycle.
Freeze command_delay_s=0.005, control_period_s=0.001, all physical parameters,
input sequence, /clock settings and source/binary identities in the trial YAML
and receipt. Run identical straight, pure-yaw, signed combined, start and stop
inputs at physics steps 4/2/1 ms and at multiple wall-time loads. Record receipt,
wheel target publication, wheel measured velocity/torque and body velocity/yaw.
Do not align outputs by an optimized time shift. Check that observed release
lateness is explained by measured /clock/callback opportunities, that no queued
motion escapes hold, and that the final zero is executed without a heartbeat.
Check CPU/message load as the nominal release frequency increased. Full
open-loop matching and four-vehicle acceptance remain separate, unpassed gates.
