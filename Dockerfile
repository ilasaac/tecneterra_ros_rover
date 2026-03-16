# AgriRover — Isaac ROS Docker image (Jetson aarch64)
#
# Base: NVIDIA Isaac ROS (JetPack 5.x/6.x, aarch64) — Jazzy detected at runtime
# Adds: project Python deps, GStreamer RTSP server libs, project workspace
#
# Build:
#   docker build -t agri_rover:latest .
#
# Run (see docker-compose.yml for device mounts):
#   docker run --rm --privileged --network host --runtime nvidia \
#     -e NVIDIA_VISIBLE_DEVICES=all agri_rover:latest \
#     ros2 launch agri_rover_bringup rover1.launch.py

ARG BASE_IMAGE=nvcr.io/nvidia/isaac/ros:isaac_ros_740c8500df2685ab1f4a4e53852601df-arm64-jetpack
FROM ${BASE_IMAGE}

# ── System packages ───────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # GStreamer RTSP server (used by video_streamer fallback + appsrc server)
    gstreamer1.0-rtsp \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    python3-gi \
    gir1.2-gst-rtsp-server-1.0 \
    # Serial ports
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ── Python packages ───────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir --break-system-packages pyserial pymavlink

# ── Isaac ROS binary packages (apt) ──────────────────────────────────────────
# isaac_ros_argus_camera / nitros / h264_encoder are exec_depends only —
# colcon build succeeds without them.  The new hash-based base image
# (isaac_ros_*-arm64-jetpack) ships these pre-installed; this step is kept
# as a best-effort install for older base images and is non-fatal.
RUN apt-get update && \
    ROS_DISTRO=$(ls /opt/ros/ | head -1) && \
    (apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-isaac-ros-common \
        ros-${ROS_DISTRO}-isaac-ros-nitros \
        ros-${ROS_DISTRO}-isaac-ros-argus-camera \
        ros-${ROS_DISTRO}-isaac-ros-h264-encoder \
     || echo "Isaac ROS apt packages not found — assuming pre-installed in base image") \
    && rm -rf /var/lib/apt/lists/*

# ── Workspace ─────────────────────────────────────────────────────────────────
ENV ISAAC_ROS_WS=/workspaces/isaac_ros-dev
WORKDIR ${ISAAC_ROS_WS}

# Copy project packages into the colcon workspace src tree
COPY ros2_ws/src src/

# Build — source whichever ROS distro the base image provides (humble or jazzy)
RUN /bin/bash -c "\
    ROS_SETUP=\$(ls /opt/ros/*/setup.bash 2>/dev/null | head -1) && \
    echo \"Sourcing \${ROS_SETUP}\" && \
    source \${ROS_SETUP} && \
    colcon build --symlink-install \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release \
    "

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
