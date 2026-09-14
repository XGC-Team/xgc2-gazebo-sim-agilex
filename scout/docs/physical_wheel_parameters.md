# Physical wheel parameter contract — harness #93

`wheel_radius` [m] now reaches the wheel-speed allocator AND all four
collision cylinders. Previously the launch argument changed only the
inverse wheel-speed allocation while geometry remained radius 0.08 m.
A radius identification then acted partly as an unexplained command gain.

`wheel_mass` [kg], `wheel_width` [m] and `wheel_joint_damping`
[N m s/rad] are exposed through the same description chain. The existing
cylinder approximation uses J_axle = m r^2/2 and
J_transverse = m(3 r^2 + width^2)/12, about its configured centre.
`wheel_inertial_axial_offset` [m] is explicit. Its default stays 0;
neither the 0.039638344 m visual/collision tire offset nor a real
motor/wheel mass centre is silently substituted for it.

Defaults reproduce the previous collision geometry, mass, J_axle=0.0096,
J_transverse=0.006564, damping=0.1 and mass centre. No PI, friction,
torque limit, allocation gain, 5 ms communication delay or controller
law is tuned by this change. There is no added input inertia.

Run `python3 scout/test/test_physical_wheel_contract.py` with xacro
installed. Tests expand the real product xacro, including non-default
radius/mass/width/damping, rather than matching source text alone.
On the target also capture `gz sdf -p` output and verify its contacts.
The visual mesh is intentionally not resized: it is not a calibrated
tire geometry measurement; a non-default radius requires visual review.

This repair makes physically consistent identification possible; it
does NOT certify agreement with hardware or four-vehicle acceptance.
Fit one parameter vector on the training input, freeze its SHA, then
evaluate the independent bag without refitting. Record mass-centre
assumptions separately; the measured marker translation is uncalibrated.
