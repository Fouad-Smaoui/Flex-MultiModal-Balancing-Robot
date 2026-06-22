# Software Architecture

## ROS2 Package Graph

```mermaid
flowchart TB
    subgraph MBSE["MBSE / Model Layer (offline, MATLAB)"]
        LQR[matlab_archive/LQR.m\ncart-pole linearization]
        SIMULINK[flex_simulink/PendulumCartSim.slx\nmaster controller candidate]
    end

    subgraph ROS2["ROS2 Jazzy Graph (Docker container)"]
        DESC[flex_description\nURDF/Xacro]
        CTRL[flex_control\nros2_control]
        HW[flex_hardware\nHAL interfaces]
        BRIDGENODE[flex_simulink generated node\nC++ ROS2 package]
    end

    subgraph SIMTARGET["Simulation Target (active now)"]
        GZ[flex_gazebo\nGazebo Harmonic world + plugins]
    end

    subgraph REALTARGET["Real Hardware Target (future work, deferred)"]
        STM32[STM32 Nucleo mbed firmware\nUNCHANGED]
        ODRIVE[ODrive\ntorque control, unbuilt V2]
        IMU[MPU6050]
        ENC[Wheel Encoders]
    end

    LQR --> SIMULINK
    SIMULINK -- "MATLAB ROS Toolbox codegen" --> BRIDGENODE
    DESC --> GZ
    DESC --> CTRL
    CTRL --> HW
    HW -- "gz_ros2_control plugin" --> GZ
    HW -. "serial bridge (future)" .-> STM32
    HW -. "torque cmd (future)" .-> ODRIVE
    STM32 --- IMU
    STM32 --- ENC
    BRIDGENODE -. "/cmd_vel /wheel_torque_cmd" .-> CTRL
    GZ -- "/joint_states /imu /odom /tf" --> CTRL
```

## Runtime Data Flow (simulation path — the active demo path today)

```mermaid
flowchart LR
    A[joint_state_broadcaster] --> B[/joint_states/]
    C[imu_sensor_broadcaster] --> D[/imu/]
    E[diff_drive_controller] --> F[/odom/, /tf/]
    G[/cmd_vel/] --> E
    H[Gazebo Harmonic\nphysics + sensors] --> A
    H --> C
    H --> E
    I[RViz2] --- B
    I --- D
    I --- F
```

## Package Responsibilities

| Package | Build type | Responsibility |
|---|---|---|
| `flex_description` | `ament_cmake` | URDF/Xacro robot model: base, wheels, legs, IMU. Source of truth for kinematics/visuals. |
| `flex_gazebo` | `ament_cmake` | Gazebo Harmonic world, spawn/demo launch files, `ros_gz_bridge` topic config. |
| `flex_control` | `ament_cmake` (pluginlib) | `controller_manager` config: `joint_state_broadcaster`, `imu_sensor_broadcaster`, `diff_drive_controller`, `leg_position_controller`. |
| `flex_hardware` | `ament_cmake` | Hardware abstraction layer — same ROS2 graph drives sim or real robot via a launch-time `hardware_type` param. |
| `flex_simulink` | `ament_cmake` | Wraps `PendulumCartSim.slx`; future home of MATLAB ROS Toolbox-generated C++ controller node. |
| `flex_msgs` | `ament_cmake`/`rosidl` | Custom messages: `WheelTorque`, `EncoderTicks`, `MotorCommand`, `LegCommand`. |

## Execution Model Note

This layer does **not** change the balancing control algorithm in `firmware/src/main.cpp`. It
adds a parallel, ROS2-native graph that (a) simulates the robot's kinematics/dynamics in Gazebo
and (b) is designed to eventually observe/command the real firmware at its existing sensor-in/
PWM-out boundary, without touching the cascaded PID or the (currently dormant) LQR code paths.
