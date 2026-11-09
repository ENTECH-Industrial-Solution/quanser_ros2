"""Prepares the RealSense images for RTAB-Map (V-SLAM).

qcar2_nodes' rgbd node publishes a 1280x720 colour image and a 1280x720 depth image (16-bit,
millimetres) without camera_info, each with its own time stamp, and with no alignment between
them: the depth camera of a D435 sees about 87 degrees, the colour camera about 69. RTAB-Map
needs a colour image, a depth image of the same pixels, a camera_info and equal time stamps.

This node makes those from the newest colour/depth pair, at a lower resolution and rate
(Jetson-friendly), and publishes

    /vslam/rgb/image          bgr8
    /vslam/depth/image        16UC1, mm, registered to the colour image
    /vslam/rgb/camera_info    pinhole, no distortion

all with the colour image's time stamp and frame `realsense_optical`.

Registration: the colour and depth cameras are about 15 mm apart, which is nothing at the
distances that matter, so only the different field of view is corrected: every colour pixel
looks up the depth pixel in the same direction (a pinhole model for each camera).
"""

import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from qcar2_web import tuning

FRAME_ID = 'realsense_optical'
DEPTH_MAX_AGE_S = 0.3   # use a depth frame at most this much older than the colour one


class RgbdBridge(Node):

    def __init__(self):
        super().__init__('rgbd_bridge')
        values = tuning.load_values('vslam')
        get = lambda name, default: self.declare_parameter(name, default).value   # noqa: E731
        self.width = int(get('image_width', values['image_width']))
        self.rate = float(get('rate', values['rate']))
        self.depth_range = float(get('depth_range', values['depth_range']))
        self.color_hfov = float(get('color_hfov', values['color_hfov']))
        self.depth_hfov = float(get('depth_hfov', values['depth_hfov']))
        self.register = bool(get('register_depth', values['register_depth']))
        color_topic = get('color_topic', '/qcar2/realsense/camera/color_image')
        depth_topic = get('depth_topic', '/qcar2/realsense/camera/depth_image')

        self._lock = threading.Lock()
        self._depth = None          # (stamp seconds, uint16 array, received monotonic)
        self._last_stamp = -1.0
        self._last_pub = 0.0
        self._maps = {}             # (color size, depth size) -> (map_x, map_y)
        self._warned = False
        self._count = 0
        self._t0 = time.monotonic()

        reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.rgb_pub = self.create_publisher(Image, '/vslam/rgb/image', reliable)
        self.depth_pub = self.create_publisher(Image, '/vslam/depth/image', reliable)
        self.info_pub = self.create_publisher(CameraInfo, '/vslam/rgb/camera_info', reliable)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(Image, color_topic, self._on_color, qos_profile_sensor_data)
        self.create_timer(10.0, self._report)
        self.get_logger().info(
            f'RTAB-Map input: {self.width} px wide, {self.rate:.0f} Hz, depth to {self.depth_range} m, '
            f'hfov colour {self.color_hfov}° / depth {self.depth_hfov}°, '
            f'depth registration {"on" if self.register else "off"}')

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_depth(self, msg):
        if msg.width == 0 or msg.height == 0 or msg.encoding.lower() not in ('mono16', '16uc1'):
            return
        stamp = self._stamp(msg)
        if stamp <= 0.0:   # rgbd re-sends the previous frame with an empty header when it has no new one
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8).view('<u2').reshape(msg.height, msg.step // 2)
        with self._lock:
            self._depth = (stamp, raw[:, :msg.width], time.monotonic())

    def _remap(self, color_size, depth_size):
        """Per output pixel, where to read the depth image."""
        key = (color_size, depth_size)
        if key not in self._maps:
            cw, ch = color_size
            dw, dh = depth_size
            if self.register:
                fc = (cw / 2.0) / math.tan(math.radians(self.color_hfov) / 2.0)
                fd = (dw / 2.0) / math.tan(math.radians(self.depth_hfov) / 2.0)
            else:   # the same view: only scale
                fc, fd = cw, dw
            us = (np.arange(cw, dtype=np.float32) - (cw - 1) / 2.0) * (fd / fc) + (dw - 1) / 2.0
            vs = (np.arange(ch, dtype=np.float32) - (ch - 1) / 2.0) * (fd / fc) + (dh - 1) / 2.0
            self._maps[key] = (np.tile(us, (ch, 1)), np.tile(vs[:, None], (1, cw)))
        return self._maps[key]

    def _on_color(self, msg):
        now = time.monotonic()
        if now - self._last_pub < 1.0 / self.rate or msg.width == 0 or msg.height == 0:
            return
        stamp = self._stamp(msg)
        if stamp <= 0.0 or stamp == self._last_stamp:
            return
        with self._lock:
            depth = self._depth
        if depth is None or now - depth[2] > DEPTH_MAX_AGE_S:
            if not self._warned:
                self.get_logger().warn('Waiting for depth images from the RealSense')
                self._warned = True
            return
        self._warned = False
        enc = msg.encoding.lower()
        if enc not in ('bgr8', 'rgb8'):
            self.get_logger().error(f'Unsupported colour encoding {msg.encoding}',
                                    throttle_duration_sec=10.0)
            return
        self._last_stamp, self._last_pub = stamp, now

        full = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        full = full.reshape(msg.height, msg.width, 3)
        out_w = min(self.width, msg.width)
        out_h = max(2, int(round(msg.height * out_w / msg.width / 2.0)) * 2)
        rgb = cv2.resize(full, (out_w, out_h), interpolation=cv2.INTER_AREA)
        if enc == 'rgb8':
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        raw = np.ascontiguousarray(depth[1])
        map_x, map_y = self._remap((out_w, out_h), (raw.shape[1], raw.shape[0]))
        depth_mm = cv2.remap(raw, map_x, map_y, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT,
                             borderValue=0)
        depth_mm[depth_mm > self.depth_range * 1000.0] = 0

        header = msg.header
        header.frame_id = FRAME_ID
        rgb_msg = Image(header=header, height=out_h, width=out_w, encoding='bgr8', step=out_w * 3,
                        data=rgb.tobytes())
        depth_msg = Image(header=header, height=out_h, width=out_w, encoding='16UC1', step=out_w * 2,
                          data=np.ascontiguousarray(depth_mm).tobytes())
        info = CameraInfo(header=header, height=out_h, width=out_w, distortion_model='plumb_bob')
        f = (out_w / 2.0) / math.tan(math.radians(self.color_hfov) / 2.0)
        cx, cy = (out_w - 1) / 2.0, (out_h - 1) / 2.0
        info.d = [0.0] * 5
        info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(info)
        self._count += 1

    def _report(self):
        elapsed = time.monotonic() - self._t0
        self.get_logger().info(f'{self._count / max(elapsed, 1e-6):.1f} frames/s to RTAB-Map')
        self._count, self._t0 = 0, time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = RgbdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
