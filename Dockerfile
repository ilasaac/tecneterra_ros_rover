# AgriRover — Isaac ROS Docker image (Jetson aarch64)
#
# Base: NVIDIA Isaac ROS Humble (JetPack 5.x, aarch64)
# Adds: project Python deps, GStreamer RTSP server libs, project workspace
#
# Build:
#   docker build -t agri_rover:latest .
#
# Run (see docker-compose.yml for device mounts):
#   docker run --rm --privileged --network host --runtime nvidia \
#     -e NVIDIA_VISIBLE_DEVICES=all agri_rover:latest \
#     ros2 launch agri_rover_bringup rover1.launch.py

ARG BASE_IMAGE=nvcr.io/nvidia/isaac/ros:aarch64-humble-ros_base_3.2.3
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
RUN pip3 install --no-cache-dir pyserial pymavlink

# ── Isaac ROS binary packages (apt) ──────────────────────────────────────────
# These ship as pre-built Debian packages in the Isaac ROS apt repo.
# The base image already sources /opt/ros/humble/setup.bash.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-isaac-ros-common \
    ros-humble-isaac-ros-nitros \
    ros-humble-isaac-ros-argus-camera \
    ros-humble-isaac-ros-h264-encoder \
    && rm -rf /var/lib/apt/lists/*

# ── Workspace ─────────────────────────────────────────────────────────────────
ENV ISAAC_ROS_WS=/workspaces/isaac_ros-dev
WORKDIR ${ISAAC_ROS_WS}

# Copy project packages into the colcon workspace src tree
COPY ros2_ws/src src/

# Build (source Isaac ROS Humble overlay first)
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release \
    "

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
