#!/bin/bash
# ONE-COMMAND recruiter demo: launches Gazebo Harmonic + the FLEX robot + RViz2 inside the
# flex-v3 Docker image (see ./setup.sh to build it first).
#
# GUI passthrough: run this from inside WSL2 (Ubuntu) so WSLg forwards the X11 display
# automatically. On native Linux, the X11 mount below is enough; on a Windows host without
# WSL2/WSLg, install an X server (e.g. VcXsrv) and set DISPLAY accordingly before running.
set -e
cd "$(dirname "$0")/.."

docker run --rm -it \
  --name flex-v3-demo \
  -e DISPLAY="${DISPLAY:-:0}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  flex-v3:latest
