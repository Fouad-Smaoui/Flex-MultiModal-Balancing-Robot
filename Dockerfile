# FLEX V3 — ROS2 Jazzy + Gazebo Harmonic runtime.
#
# This image builds and runs the flex_* ROS2 packages (flex_description, flex_gazebo, and
# future flex_control/flex_hardware/flex_simulink/flex_msgs once they exist). Chosen over a
# bare WSL2/host install for portability — "clone, build the image, run one command" is the
# whole setup story (see docs/architecture/deployment_architecture.md, Recruiter Demo Workflow).

FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_WS=/ws
WORKDIR ${ROS_WS}

# Only the ROS2 packages are copied into the colcon workspace — firmware/, matlab_archive/,
# odrive/, future_work/, docs/ etc. are not part of the build.
COPY flex_description ${ROS_WS}/src/flex_description
COPY flex_gazebo ${ROS_WS}/src/flex_gazebo

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep update --rosdistro jazzy || true && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    colcon build --symlink-install"

COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "flex_gazebo", "flex_demo.launch.py"]
