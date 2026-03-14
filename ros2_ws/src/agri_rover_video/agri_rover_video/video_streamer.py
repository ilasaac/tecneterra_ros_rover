"""
agri_rover_video — video_streamer node

Launches a GStreamer RTSP server using gst-rtsp-server.
GQC connects to:  rtsp://<rover-ip>:<rtsp_port>/stream

Pipeline (Jetson CSI camera, H.264 HW encode):
  nvarguscamerasrc → nvvidconv → nvv4l2h264enc → rtph264pay → RTSP

For USB camera, substitute:
  v4l2src device=/dev/video0 → videoconvert → x264enc → rtph264pay → RTSP

Requires on Jetson:
  gstreamer1.0-rtsp  gstreamer1.0-plugins-{bad,good,ugly}
  gstreamer1.0-libav  python3-gi  gir1.2-gst-rtsp-server-1.0
"""

from __future__ import annotations

import subprocess
import shlex
import signal
import sys

import rclpy
from rclpy.node import Node


PIPELINES = {
    'csi': (
        'nvarguscamerasrc sensor-id={sensor_id} ! '
        'video/x-raw(memory:NVMM),width={w},height={h},framerate={fps}/1 ! '
        'nvvidconv flip-method={flip} ! '
        'nvv4l2h264enc bitrate={bps} ! '
        'rtph264pay name=pay0 pt=96'
    ),
    'usb': (
        'v4l2src device=/dev/video0 ! '
        'video/x-raw,width={w},height={h},framerate={fps}/1 ! '
        'videoconvert ! '
        'x264enc bitrate={kbps} tune=zerolatency ! '
        'rtph264pay name=pay0 pt=96'
    ),
    'test': (
        'videotestsrc ! '
        'video/x-raw,width={w},height={h},framerate={fps}/1 ! '
        'videoconvert ! '
        'x264enc bitrate={kbps} tune=zerolatency ! '
        'rtph264pay name=pay0 pt=96'
    ),
}


class VideoStreamerNode(Node):

    def __init__(self):
        super().__init__('video_streamer')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('rtsp_port',      8554)
        self.declare_parameter('mount_point',    '/stream')
        self.declare_parameter('camera_source',  'csi')
        self.declare_parameter('csi_sensor_id',  0)
        self.declare_parameter('width',          1280)
        self.declare_parameter('height',         720)
        self.declare_parameter('framerate',      30)
        self.declare_parameter('bitrate',        2000)
        self.declare_parameter('flip_method',    0)

        port    = self.get_parameter('rtsp_port').value
        mount   = self.get_parameter('mount_point').value
        src     = self.get_parameter('camera_source').value
        w       = self.get_parameter('width').value
        h       = self.get_parameter('height').value
        fps     = self.get_parameter('framerate').value
        bps     = self.get_parameter('bitrate').value * 1000
        flip    = self.get_parameter('flip_method').value
        sensor  = self.get_parameter('csi_sensor_id').value

        pipeline_tmpl = PIPELINES.get(src, PIPELINES['test'])
        pipeline = pipeline_tmpl.format(
            sensor_id=sensor, w=w, h=h, fps=fps,
            bps=bps, kbps=bps // 1000, flip=flip)

        # Launch gst-rtsp-server via test-launch helper
        # For production, replace with python-gi GstRtspServer binding
        cmd = (
            f'gst-rtsp-server '
            f'--port {port} '
            f'--mount-point {mount} '
            f'"{pipeline}"'
        )
        self.get_logger().info(f'Starting RTSP server on :{port}{mount}')
        self.get_logger().info(f'Pipeline: {pipeline}')

        self._proc = subprocess.Popen(
            shlex.split(cmd),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def destroy_node(self):
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
