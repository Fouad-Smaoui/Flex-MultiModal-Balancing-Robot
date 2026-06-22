# Control Architecture

## What's actually deployed today (unchanged by V3)

`firmware/src/main.cpp` runs a cascaded PID:
- **Outer loop:** PD on wheel position/velocity (from encoder odometry) → produces a small tilt
  setpoint (`AngleOffset`, clamped to ±5.5°).
- **Inner loop:** PID on IMU pitch (from the MPU6050's on-chip DMP, not a Kalman/complementary
  filter written in this repo) tracking that tilt setpoint → motor PWM duty (clamped to ±0.8).
- **Fall safety:** `|pitch| > 30°` triggers a stop + state reset (not a true e-stop).
- In the shipped firmware, `Ki = Kd = 0` by default — effectively P-only, gains live-tunable via
  Bluetooth or onboard potentiometers.

## LQR — designed, not deployed

A 4-state LQR (`[x, ẋ, θ, θ̇]`) is fully derived offline in `matlab_archive/LQR.m` via MATLAB's
`lqr(A, B, Q, R)` (Q=100·I₄, R=1) from a linearized cart-pole model. **The resulting gains were
never transferred to firmware** — `firmware/src/main.cpp`'s `K1lqr..K4lqr` are hardcoded to `0`,
making `calculateLQRControl()` a dead code path whose output is immediately overwritten by the
PID dispatch. V3 does not fix this in firmware. Instead, the dormant LQR design is given a second
life as a **parallel ROS2/Simulink demonstration** (see "Simulink master controller" below),
running against live `/imu` and `/joint_states` topics from the Gazebo digital twin — a legitimate
MBSE showcase that does not touch the real robot's control loop.

## Simulink master controller: `PendulumCartSim.slx`

Of the three MATLAB/Simulink assets, `flex_simulink/models/PendulumCartSim.slx` is the chosen
candidate for MATLAB ROS Toolbox code generation:

- `matlab_archive/2D_Simulator/Script_matlab_cte_lqr.m` is a script + 2D animation, not a
  block-diagram controller — not codegen material.
- `matlab_archive/Multi-Body_Simulator/simulateur_FLEX_multicorps_V2.slx` is a multibody
  **physics validation** model (paired with the SolidWorks part) — deploying a physics sim as a
  "controller" would be architecturally backwards.
- `PendulumCartSim.slx` is structured as a control-law block diagram, and a Simulink-Coder build
  was already attempted (`flex_simulink/build_diagnostics/PendulumCartSim_grt_rtw/`) — it failed,
  leaving only `build_exception.mat` (no generated code). The first concrete step in reviving this
  is loading `build_exception.mat` in MATLAB to read the captured exception, then retargeting the
  build from generic GRT to a ROS2 node via Robotics System Toolbox + ROS Toolbox.

## ros2_control framing: `diff_drive_controller` is not a second balance loop

`flex_control`'s `diff_drive_controller` consumes `/cmd_vel` and produces `/odom`/`/tf`. This
mirrors the *existing* outer velocity/position control surface in firmware (the PD loop that
turns wheel position/velocity error into a tilt setpoint) — it does not add a new control
authority or compete with the inner balance PID.

## ODrive control mode: torque control

The existing `odrive/ODrive_Configuration_Script.txt` already configures
`CONTROL_MODE_TORQUE_CONTROL`. V3 respects this rather than re-deciding it:
- Torque mode treats the ODrive as a pure actuator (current/torque follows command), matching how
  the firmware's inner PID already treats the MC33926 H-bridge as a pure PWM-to-torque actuator.
- Velocity or position mode would impose the ODrive's own internal velocity loop *underneath* the
  existing balance PID's torque-equivalent output — two stacked loops fighting each other, a real
  control-architecture hazard, not just a style mismatch.

This is a designed-only decision (the V2 ODrive motor stage is unbuilt) — see
`docs/architecture/deployment_architecture.md` for status.
