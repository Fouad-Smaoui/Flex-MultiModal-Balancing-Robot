# Deployment Architecture

## Status: simulation-only today; real-hardware integration is designed but deferred

No SolidWorks access and no physical-robot access are currently available, so the items below are
**designed, not implemented or hardware-validated**. They're documented now so the architecture
is ready to execute as soon as access is available, and so the design rationale isn't lost.

## STM32 integration strategy: serial bridge (chosen), micro-ROS and CAN rejected

| Option | Verdict | Why |
|---|---|---|
| micro-ROS | Rejected | Requires an RTOS (FreeRTOS/Zephyr/NuttX) or the micro-ROS bare-metal executor port. The current firmware is bare-metal mbed with Ticker/Timer-driven timing for the balance loop — porting risks the timing-critical control loop, which V3 must not touch. |
| CAN bridge | Rejected | No CAN transceiver exists on the current hardware (only Bluetooth UART). ODrive's CAN interface belongs to the unbuilt V2 motor stage, not the STM32. Solves a multi-node bandwidth problem FLEX doesn't have. |
| **Custom serial bridge** | **Chosen** | Extends the firmware's *existing* UART tuning protocol additively. Zero RTOS port, zero new hardware. The only firmware change in this entire plan is new parse cases in the existing UART handler — purely additive telemetry/command framing, not a change to control logic. |

### Topic mapping (future work)

| Direction | Topic | Type | Firmware source/sink |
|---|---|---|---|
| STM32 → ROS2 | `/imu_data` | `sensor_msgs/Imu` | MPU6050 DMP read, already happening in the main loop |
| STM32 → ROS2 | `/encoder_ticks` | `flex_msgs/EncoderTicks` | Raw QEI tick counters (pre-CPR-scaling — intentionally raw so the known left/right CPR inconsistency, see Known Issues, is visible rather than silently baked in) |
| STM32 → ROS2 | `/battery` | `sensor_msgs/BatteryState` | Existing current-sense ADC averaging, repurposed as telemetry only |
| ROS2 → STM32 | `/motor_commands` | `flex_msgs/MotorCommand` | Feeds the existing PWM output stage |
| ROS2 → STM32 | `/leg_commands` | `flex_msgs/LegCommand` | Feeds existing servo control code |

**Non-goal:** the bridge does not intercept or relay the balance loop's internal PID computation
— it only taps the loop's existing natural boundaries (sensor in, PWM out).

## ODrive integration: torque-control ROS2 node (designed, V2 stage unbuilt)

See `control_architecture.md` for the torque-vs-velocity-vs-position rationale. Implementation
plan: a Python `ament_python` node (`flex_odrive_node`, using the `python-odrive` client library
rather than reimplementing ODrive's USB protocol) subscribing to `/odrive_torque_cmd` and calling
`odrv.axis0.controller.input_torque`. Explicitly lower priority than the STM32 bridge — the ODrive
V2 motor stage was never built, so this demonstrates forward design, not a working deployment.

## Known Issues Carried Forward From the Original Firmware Audit

These are pre-existing firmware issues, **not introduced by V3** and **not fixed by V3** (per the
constraint that the balancing control algorithm is not to be modified). Recorded here so they're
not silently inherited by future hardware-integration work:

- `firmware/src/main.cpp`'s left encoder is constructed with `64.30` (a float) vs. the right
  encoder's `64*30` (int) — a likely typo that desynchronizes left/right encoder CPR scaling.
- `calculateLQRControl()` runs every control tick but its output is discarded.
- `balanceControl()` is invoked both from a 100 Hz Ticker and from the main loop — a latent
  double-invocation race.

## Recruiter Demo Workflow (the part that works today)

1. `git clone` the repo.
2. `./scripts/demo.sh` — builds/pulls the Docker image (ROS2 Jazzy + Gazebo Harmonic), builds the
   `flex_*` packages with `colcon`, launches Gazebo + RViz.
3. Robot model spawns and settles under gravity in Gazebo Harmonic.
4. `ros2 topic list` / `rqt_graph` shows the live ROS2 graph (`/joint_states`, `/imu`, `/odom`,
   `/tf`, `/cmd_vel`).
5. RViz2 shows the robot model with live joint states.
6. (Future) Compare a recorded real-hardware log against the simulated topics once the STM32
   bridge exists.
