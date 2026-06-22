# Human Tracking System — Standalone Prototype (Not Integrated)

This is a standalone OpenCV computer-vision prototype for a **separate Raspberry Pi + L298N
rover chassis**. It is **not connected to FLEX's STM32 balance firmware** — there is no
serial/bridge protocol anywhere in this repo linking the two.

## What's here

- `vision.py` / `vision2.py` — HSV color-blob tracking and ArUco fiducial-marker pose estimation
  via OpenCV (`cv2.aruco`).
- `control.py` — a `Driver` class wrapping `RPi.GPIO` to drive an L298N dual H-bridge.
- `Calibration_Camera/` — chessboard-based camera intrinsic calibration (`cameraCalibration.py`,
  `takePictureChess.py`) with saved `cameraMatrix.txt` / `cameraDistortion.txt`.

## Why it's scoped out of FLEX V3

FLEX V3's ROS2/Gazebo digital-twin work models and simulates the actual STM32-based balancing
robot. Wrapping this human-tracking prototype in a ROS2 node would imply a perception-integrated
robot that does not exist on real hardware today. It's kept here as a reference for a possible
perception layer on a future FLEX hardware revision, not as part of the active architecture.

## Reviving this later

To integrate, this would need: a defined message protocol to FLEX's STM32 (or its ROS2 hardware
interface, once built), a `requirements.txt` (none exists currently — depends on `opencv-python`,
`numpy`, `RPi.GPIO`), and a decision on whether tracking runs on the robot's own compute or a
companion board.
