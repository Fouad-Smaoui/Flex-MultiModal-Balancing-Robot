# Simulation Architecture

## Runtime

ROS2 Jazzy + Gazebo Harmonic, run inside a Docker container (chosen over WSL2/bare install for
portability — `docker compose up` or `./scripts/demo.sh` is the entire setup needed on a fresh
machine with Docker installed).

## Topic Parity (sim ↔ real)

The hardware-abstraction-layer design goal is that `flex_control`'s controllers, and everything
above them, are unaware of whether they're talking to Gazebo or the real robot.

| Topic | Type | Sim source | Real-robot equivalent (future work) |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` via `gz_ros2_control` | Derived from encoder ticks by `flex_hardware`'s STM32 interface |
| `/imu` | `sensor_msgs/Imu` | Gazebo IMU sensor plugin | MPU6050 DMP quaternion + gyro, relayed by the same interface |
| `/odom` | `nav_msgs/Odometry` | `diff_drive_controller` | Same node, fed by real `/joint_states` — no duplicated logic |
| `/cmd_vel` | `geometry_msgs/Twist` | teleop / demo script / generated Simulink node | Same topic, mapped to motor PWM duty by `flex_hardware` |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` + `diff_drive_controller` | Identical — this is the actual point of the HAL |

## Fidelity Caveats (be upfront about these)

- **Meshes/collision geometry are box/cylinder primitives**, not CAD-derived — see
  `hardware_architecture.md`'s CAD-to-Sim Pipeline. Visual fidelity is intentionally deferred.
- **Inertias are analytically-computed placeholders** from primitive geometry and an estimated
  total mass, not measured from the real robot. Dynamics behavior in sim will not exactly match
  the real robot until Phase "CAD export" lands.
- **The balance controller is not running in Gazebo today.** `flex_control`'s controllers expose
  kinematic-level topics (`/cmd_vel`, `/odom`, `/joint_states`, `/imu`); the actual balancing
  control law (cascaded PID, or the dormant LQR via the Simulink-generated node) is a separate,
  not-yet-wired demonstration layer — see `control_architecture.md`.

## Gazebo Plugin Choice

`gz_ros2_control`'s `GazeboSimSystem` hardware-interface plugin is used instead of a bespoke
Gazebo diff-drive plugin (e.g. the older `libgazebo_ros_diff_drive` pattern), so that
`flex_control`'s `ros2_control` controllers — not Gazebo-specific code — own all control logic.
This is what makes the same controller stack portable to real hardware later.
