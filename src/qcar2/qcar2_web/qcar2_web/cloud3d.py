"""Live 3D point cloud from the V-SLAM camera images, for the web page's 3D view.

While mapping with the camera (V-SLAM), rgbd_bridge publishes a colour image and a depth image of
the same pixels (/vslam/rgb/image, /vslam/depth/image, /vslam/rgb/camera_info). This turns the
newest pair into a coloured point cloud, places it on the map with the car's pose from TF, and
offers it as binary messages for the page, which draws it with WebGL and lets the mouse move the
view (like RViz). The page accumulates the clouds into a map.

Only works while someone watches (it subscribes then), and costs nothing otherwise.

Message (little endian), one per cloud:
    float32 x, y, yaw   car pose in the map frame (0 if unknown)
    uint32  count       number of points
    uint32  flags       bit 0: points are in the map frame (else relative to the car)
    int16   x, y, z     per point, millimetres, x forward / y left / z up (map axes)
    uint8   r, g, b     per point
"""

import math
import struct
import threading
import time

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from qcar2_web import tuning

RATE_HZ = 4.0           # clouds per second at most
STRIDE = 2              # use every 2nd pixel in both directions
MIN_RANGE_M = 0.15
MAX_RANGE_M = 6.0
HEADER = struct.Struct('<fffII')


def _rotation(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


class CloudStream:

    def __init__(self, node, get_obstacle_config, get_pose):
        self.node = node
        self._get_cfg = get_obstacle_config
        self._get_pose = get_pose        # () -> (x, y, yaw) in the map frame, or None
        self._lock = threading.Lock()
        self._viewers = 0
        self._subs = []
        self._rgb = None                 # (stamp key, array)
        self._info = None
        self._last = 0.0
        self._frame = None               # newest message (bytes)
        self.seq = 0
        self._seen = 0.0                 # when the newest input arrived
        self._cam_tf = None
        self._cam_cfg = None

    # ------------------------------------------------------------ viewers
    @property
    def viewers(self):
        return self._viewers

    @property
    def receiving(self):
        return time.monotonic() - self._seen < 2.0

    def add_viewer(self):
        with self._lock:
            self._viewers += 1
            if self._subs:
                return
            reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
            n = self.node
            self._subs = [
                n.create_subscription(CameraInfo, '/vslam/rgb/camera_info', self._on_info, reliable),
                n.create_subscription(Image, '/vslam/rgb/image', self._on_rgb, reliable),
                n.create_subscription(Image, '/vslam/depth/image', self._on_depth, reliable),
            ]

    def remove_viewer(self):
        with self._lock:
            self._viewers = max(0, self._viewers - 1)
            if self._viewers or not self._subs:
                return
            for s in self._subs:
                self.node.destroy_subscription(s)
            self._subs = []
            self._rgb = self._info = self._frame = None

    def frame(self):
        with self._lock:
            return self.seq, self._frame

    # ------------------------------------------------------------ input
    def _on_info(self, msg):
        self._info = (msg.k[0], msg.k[2], msg.k[5])

    @staticmethod
    def _key(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def _on_rgb(self, msg):
        if msg.encoding != 'bgr8':
            return
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        self._rgb = (self._key(msg), a)

    def _on_depth(self, msg):
        now = time.monotonic()
        self._seen = now
        rgb, info = self._rgb, self._info
        if rgb is None or info is None or rgb[0] != self._key(msg) or now - self._last < 1.0 / RATE_HZ:
            return
        if msg.encoding != '16UC1':
            return
        self._last = now
        depth = np.frombuffer(msg.data, np.uint8).view('<u2').reshape(msg.height, msg.width)
        try:
            data = self._build(depth, rgb[1], info)
        except Exception as e:   # never let a display feature disturb the robot
            self.node.get_logger().warn(f'3D view: {e}', throttle_duration_sec=10.0)
            return
        with self._lock:
            if self._subs:
                self._frame = data
                self.seq += 1

    # ------------------------------------------------------------ cloud
    def _camera_tf(self):
        cfg = self._get_cfg()
        key = (cfg['cam_x'], cfg['cam_y'], cfg['cam_z'], cfg['cam_pitch'], cfg['cam_roll'], cfg['wheelbase'])
        if self._cam_cfg != key:
            x, y, z, qx, qy, qz, qw = tuning.camera_transform(cfg)
            self._cam_tf = (_rotation(qx, qy, qz, qw), np.array([x, y, z]))
            self._cam_cfg = key
        return self._cam_tf

    def _build(self, depth, bgr, info):
        f, cx, cy = info
        d = depth[::STRIDE, ::STRIDE].astype(np.float32) * 0.001
        h, w = d.shape
        v, u = np.mgrid[0:h, 0:w].astype(np.float32) * STRIDE
        ok = (d > MIN_RANGE_M) & (d < MAX_RANGE_M)
        z = d[ok]
        pts = np.stack([(u[ok] - cx) * z / f, (v[ok] - cy) * z / f, z], axis=1)
        col = bgr[::STRIDE, ::STRIDE][ok][:, ::-1]
        rot, trans = self._camera_tf()
        pts = pts @ rot.T + trans          # optical frame -> base_link (x forward, y left, z up)
        pose = self._get_pose()
        flags = 0
        px = py = pyaw = 0.0
        if pose is not None:
            px, py, pyaw = pose
            c, s = math.cos(pyaw), math.sin(pyaw)
            x = pts[:, 0] * c - pts[:, 1] * s + px
            y = pts[:, 0] * s + pts[:, 1] * c + py
            pts = np.stack([x, y, pts[:, 2]], axis=1)
            flags = 1
        mm = np.clip(np.rint(pts * 1000.0), -32767, 32767).astype('<i2')
        return (HEADER.pack(px, py, pyaw, len(mm), flags) + mm.tobytes()
                + np.ascontiguousarray(col).tobytes())
