"""
agri_rover_video — video_streamer node

CSI camera (camera_source: csi)  — Isaac ROS path:
  Subscribes to H264-encoded frames published by isaac_ros_h264_encoder
  (sensor_msgs/CompressedImage, format="h264") and serves them as RTSP via a
  GstRtspServer appsrc pipeline.  The camera capture and hardware encoding run
  as NITROS composable nodes launched separately by video_pipeline.launch.py.

USB / test (camera_source: usb | test)  — legacy GStreamer path:
  Spawns a gst-rtsp-server subprocess directly (no Isaac ROS acceleration).

GQC connects to:  rtsp://<rover-ip>:<rtsp_port>/stream

Requires on Jetson (all modes):
  gstreamer1.0-rtsp  gstreamer1.0-plugins-{bad,good,ugly}
  gstreamer1.0-libav  python3-gi  gir1.2-gst-rtsp-server-1.0
"""

from __future__ import annotations

import subprocess
import shlex
import threading

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


# ── USB / test fallback pipelines (subprocess gst-rtsp-server) ───────────────

_FALLBACK_PIPELINES: dict[str, str] = {
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


# ── Isaac ROS RTSP factory (appsrc → h264parse → rtph264pay) ─────────────────

class _H264AppsrcFactory(GstRtspServer.RTSPMediaFactory):
    """
    RTSP media factory backed by an appsrc element.
    Clients connect and receive H264 frames pushed via push_frame().
    The factory is shared so all clients receive the same stream.
    """

    def __init__(self) -> None:
        super().__init__()
        self.set_shared(True)
        self.set_latency(0)
        self._appsrc: Gst.Element | None = None
        self._lock = threading.Lock()

    def do_create_element(self, url: GstRtspServer.RTSPUrl) -> Gst.Element:
        # appsrc receives raw H264 byte-stream NAL units from the ROS2 callback.
        # h264parse converts to RTP-friendly format; rtph264pay packetises.
        pipeline = Gst.parse_launch(
            'appsrc name=src is-live=true format=time do-timestamp=true '
            'caps=video/x-h264,stream-format=byte-stream,alignment=au '
            '! h264parse ! rtph264pay name=pay0 pt=96 config-interval=1'
        )
        with self._lock:
            self._appsrc = pipeline.get_by_name('src')
        return pipeline

    def push_frame(self, data: bytes) -> None:
        with self._lock:
            if self._appsrc is None:
                return
            buf = Gst.Buffer.new_wrapped(data)
            self._appsrc.emit('push-buffer', buf)


# ── Node ──────────────────────────────────────────────────────────────────────

class VideoStreamerNode(Node):

    def __init__(self) -> None:
        super().__init__('video_streamer')

        self.declare_parameter('rtsp_port',     8554)
        self.declare_parameter('mount_point',   '/stream')
        self.declare_parameter('camera_source', 'csi')
        self.declare_parameter('csi_sensor_id', 0)
        self.declare_parameter('width',         1280)
        self.declare_parameter('height',        720)
        self.declare_parameter('framerate',     30)
        self.declare_parameter('bitrate',       2000)   # kbps
        self.declare_parameter('flip_method',   0)

        self._port  = self.get_parameter('rtsp_port').value
        self._mount = self.get_parameter('mount_point').value
        self._src   = self.get_parameter('camera_source').value
        w           = self.get_parameter('width').value
        h           = self.get_parameter('height').value
        fps         = self.get_parameter('framerate').value
        kbps        = self.get_parameter('bitrate').value

        self._proc    = None
        self._factory = None
        self._glib    = None

        if self._src == 'csi':
            self._start_isaac_rtsp_server()
        else:
            self._start_fallback_rtsp(w, h, fps, kbps)

    # ── Isaac ROS CSI path ────────────────────────────────────────────────────

    def _start_isaac_rtsp_server(self) -> None:
        """
        Start GstRtspServer fed by compressed H264 frames from isaac_ros_h264_encoder.
        The camera and encoder composable nodes are launched by video_pipeline.launch.py.
        """
        Gst.init(None)

        server = GstRtspServer.RTSPServer()
        server.set_service(str(self._port))

        self._factory = _H264AppsrcFactory()
        server.get_mount_points().add_factory(self._mount, self._factory)
        server.attach(None)

        # isaac_ros_h264_encoder publishes sensor_msgs/CompressedImage (format="h264")
        self.create_subscription(
            CompressedImage,
            'image_compressed',
            self._on_compressed_image,
            10,
        )

        # GstRtspServer requires a GLib main loop
        self._glib = GLib.MainLoop()
        threading.Thread(target=self._glib.run, daemon=True).start()

        self.get_logger().info(
            f'[Isaac ROS] RTSP server: rtsp://<ip>:{self._port}{self._mount}'
        )

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        # msg.format == "h264"; msg.data contains raw NAL units
        if self._factory:
            self._factory.push_frame(bytes(msg.data))

    # ── USB / test fallback path ──────────────────────────────────────────────

    def _start_fallback_rtsp(self, w: int, h: int, fps: int, kbps: int) -> None:
        tmpl = _FALLBACK_PIPELINES.get(self._src, _FALLBACK_PIPELINES['test'])
        pipeline = tmpl.format(w=w, h=h, fps=fps, kbps=kbps)

        Gst.init(None)

        server = GstRtspServer.RTSPServer()
        server.set_service(str(self._port))

        factory = GstRtspServer.RTSPMediaFactory()
        factory.set_launch(f'( {pipeline} )')
        factory.set_shared(True)
        server.get_mount_points().add_factory(self._mount, factory)
        server.attach(None)

        self._glib = GLib.MainLoop()
        threading.Thread(target=self._glib.run, daemon=True).start()

        self.get_logger().info(
            f'[GStreamer] RTSP server: rtsp://<ip>:{self._port}{self._mount}'
        )

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        if self._glib:
            self._glib.quit()
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
