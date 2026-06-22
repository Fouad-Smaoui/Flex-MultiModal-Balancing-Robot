#!/bin/bash
# One-time setup: build the FLEX V3 Docker image (ROS2 Jazzy + Gazebo Harmonic + flex_* packages).
#
# Requires Docker Desktop running (Windows/Mac) or the Docker daemon running (Linux).
set -e
cd "$(dirname "$0")/.."
docker build -t flex-v3:latest .
echo "Image built. Run ./scripts/demo.sh to launch the recruiter demo."
