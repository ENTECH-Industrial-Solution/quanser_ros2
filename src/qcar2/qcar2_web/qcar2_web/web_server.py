"""Local web interface for the QCar2.

- Streams camera topics to the browser as MJPEG (subscribes only while someone is watching).
- Builds a bird's-eye surround view from the four CSI cameras (see surround.py).
- Lets one browser at a time drive the car by publishing qcar2_interfaces/MotorCommands.
- Uses the RealSense depth image to brake for obstacles while driving forward, and offers an
  auto mode that drives forward and steers around them (see obstacle.py).
- Offers a follow mode that finds people in the RealSense colour image and follows one of
  them, keeping a set distance measured with the depth image (see person.py).
- Shows the SLAM map with the car, lidar scan and planned path, sends goals clicked on it to
  Nav2, and drives the car with Nav2's commands in 'nav' mode (see nav.py).
- PCB buttons (S2 held: Wi-Fi hotspot/office, S3 held: ROS on/off, i.e. start the ROS
  nodes or hand the hardware to MATLAB, S4 held: eco mode), the same switches in the page,
  and a status line on the PCB LCD (see panel.py). The car has no E-STOP button: use the
  page's.
- Eco mode: the CSI cameras and the RealSense (qcar2-csi / qcar2-realsense services) run
  only while something uses them (panel.EcoSensors).

Safety: qcar2_hardware keeps applying the last command it received and has no timeout of
its own, so this node runs a watchdog. While a browser holds control, a command is published
at a fixed rate; if the browser stops sending (closed tab, Wi-Fi drop) for longer than
`command_timeout` seconds, zero speed is published instead.
"""

import asyncio
import base64
import errno
import fcntl
import hmac
import itertools
import json
import math
import os
import signal
import socket
import struct
import threading
import time

# Before numpy: the small matrix products here gain nothing from OpenBLAS's thread pool, and
# that pool has hung (spinning a core at 100%) when used from the executor's threads.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import cv2  # noqa: E402
import numpy as np
import rclpy
from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
from qcar2_interfaces.msg import MotorCommands
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage, Image
from std_msgs.msg import ColorRGBA, UInt8

from qcar2_web import cloud3d, nav, obstacle, panel, person, surround, tuning

# name, label, default topic, kind
CAMERA_DEFAULTS = [
    ('front', 'Front (CSI)', '/qcar2/csi_front/camera/csi_image', 'color'),
    ('left', 'Left (CSI)', '/qcar2/csi_left/camera/csi_image', 'color'),
    ('right', 'Right (CSI)', '/qcar2/csi_right/camera/csi_image', 'color'),
    ('back', 'Back (CSI)', '/qcar2/csi_back/camera/csi_image', 'color'),
    ('realsense', 'RealSense RGB', '/qcar2/realsense/camera/color_image', 'color'),
    ('depth', 'RealSense Depth', '/qcar2/realsense/camera/depth_image', 'depth'),
]
SURROUND_NAME = 'surround'
SURROUND_PREVIEW_PX = 240  # output size while a calibration slider is being dragged
OBSTACLE_NAME = 'obstacle'
OBSTACLE_RATE_HZ = 15.0
PERSON_NAME = 'person'
PERSON_RATE_HZ = 15.0
MAP_NAME = 'map'
NAV_RATE_HZ = 5.0
AUTO_MODES = ('avoid', 'follow', 'nav')

# WebSocket camera streams: at most this many frames sent but not yet acknowledged by the
# page. Without a limit, frames pile up in the TCP send buffer on a slow Wi-Fi link and the
# view lags seconds behind; with it, a slow link gets a lower frame rate instead of delay.
STREAM_WINDOW = 2
STREAM_ACK_TIMEOUT_S = 1.0  # assume the acks were lost (or an older page) and send anyway

PUBLISH_RATE_HZ = 20.0
BUTTON_HOLD_S = 2.0    # S2 (Wi-Fi), S3 (ROS on/off) and S4 (eco) must be held this long
HIL_BUTTONS_S = 0.4    # button poll period in MATLAB mode (HIL card held ~25 ms each time)
PANEL_REFRESH_S = 2.0
STOP_REPEAT_S = 1.0  # keep sending zeros this long after control is released
BATTERY_SAMPLE_S = 1.0


def _finite_or_none(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _clamp_unit(value):
    value = _finite_or_none(value)
    if value is None:
        return 0.0
    return max(-1.0, min(1.0, value))


def _parameter_value(value):
    """rcl_interfaces/ParameterValue for a bool, int or float."""
    if isinstance(value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    if isinstance(value, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))


def image_to_bgr(msg, depth_max_mm):
    """Convert a sensor_msgs/Image to a BGR (or grey) numpy image, or None if unsupported."""
    h, w, step = msg.height, msg.width, msg.step
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('bgr8', 'rgb8', '8uc3'):
        img = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else img
    if enc in ('bgra8', 'rgba8', '8uc4'):
        img = buf.reshape(h, step)[:, :w * 4].reshape(h, w, 4)
        code = cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(img, code)
    if enc in ('mono8', '8uc1'):
        return buf.reshape(h, step)[:, :w]
    if enc in ('mono16', '16uc1'):
        dtype = '>u2' if msg.is_bigendian else '<u2'
        depth = buf.view(dtype).reshape(h, step // 2)[:, :w]
        scaled = np.clip(depth.astype(np.float32) * (255.0 / depth_max_mm), 0, 255).astype(np.uint8)
        color = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
        color[depth == 0] = 0
        return color
    return None


class CameraStream:
    """One camera topic, converted to JPEG frames for any number of HTTP viewers."""

    def __init__(self, node, name, label, topic, kind):
        self.node = node
        self.name = name
        self.label = label
        self.topic = topic
        self.kind = kind
        self.loop = None
        self._lock = threading.Lock()
        self._sub = None
        self._viewers = 0
        self._jpeg = None
        self._seq = 0
        self._last_frame_time = 0.0
        self._waiters = set()

    def info(self):
        return {'name': self.name, 'label': self.label, 'topic': self.topic, 'kind': self.kind}

    def add_viewer(self):
        with self._lock:
            self._viewers += 1
            need_sub = self._sub is None
        if need_sub:
            sub = self._subscribe()
            with self._lock:
                self._sub = sub
            self.node.get_logger().info(f'Camera "{self.name}": subscribed to {self.topic}')

    def remove_viewer(self):
        sub = None
        with self._lock:
            self._viewers -= 1
            if self._viewers <= 0:
                self._viewers = 0
                sub, self._sub = self._sub, None
                self._jpeg = None
        if sub is not None:
            self._unsubscribe(sub)
            self.node.get_logger().info(f'Camera "{self.name}": no viewers, unsubscribed')

    def _unsubscribe(self, sub):
        self.node.destroy_subscription(sub)

    def _subscribe(self):
        group = self.node.camera_group
        if self.kind == 'color' and self.node.use_compressed:
            return self.node.create_subscription(
                CompressedImage, self.topic + '/compressed', self._on_compressed,
                qos_profile_sensor_data, callback_group=group)
        return self.node.create_subscription(
            Image, self.topic, self._on_raw, qos_profile_sensor_data, callback_group=group)

    def _rate_limited(self):
        now = time.monotonic()
        if now - self._last_frame_time < 1.0 / self.node.max_fps:
            return True
        self._last_frame_time = now
        return False

    def _on_compressed(self, msg):
        if self._rate_limited():
            return
        fmt = msg.format.lower()
        if not self.node.recompress and ('jpeg' in fmt or 'jpg' in fmt):
            self._set_frame(bytes(msg.data))
            return
        img = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if img is not None:
            self._encode_and_set(img)

    def _on_raw(self, msg):
        if msg.width == 0 or msg.height == 0:  # rgbd used to republish an empty cv::Mat
            return
        if self._rate_limited():
            return
        img = image_to_bgr(msg, self.node.depth_max_mm)
        if img is None:
            self.node.get_logger().warn(
                f'Camera "{self.name}": unsupported encoding {msg.encoding}',
                throttle_duration_sec=10.0)
            return
        self._encode_and_set(img)

    def _encode_and_set(self, img):
        max_w = self.node.stream_max_width
        # Only real cameras: the drawn views (map, obstacles, ...) carry small text and lines.
        if max_w and self.kind in ('color', 'depth') and img.shape[1] > max_w:
            h = max(1, round(img.shape[0] * max_w / img.shape[1]))
            img = cv2.resize(img, (max_w, h), interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.node.jpeg_quality])
        if ok:
            self._set_frame(jpeg.tobytes())

    def _set_frame(self, jpeg):
        with self._lock:
            self._jpeg = jpeg
            self._seq += 1
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self._wake_waiters)

    def _wake_waiters(self):
        for fut in self._waiters:
            if not fut.done():
                fut.set_result(None)
        self._waiters.clear()

    async def wait_frame(self, last_seq, timeout):
        """Return (jpeg, seq) once a frame newer than last_seq exists, or the current one on timeout."""
        with self._lock:
            # _jpeg is None before the first frame (and after the last viewer left, which keeps
            # _seq): returning then would make the caller spin without ever awaiting.
            if self._jpeg is not None and self._seq != last_seq:
                return self._jpeg, self._seq
        fut = asyncio.get_running_loop().create_future()
        self._waiters.add(fut)
        try:
            await asyncio.wait_for(fut, timeout)
        except asyncio.TimeoutError:
            pass
        finally:
            self._waiters.discard(fut)
        with self._lock:
            return self._jpeg, self._seq


def decode_image(msg, depth_max_mm, half=False):
    """BGR image from a CompressedImage or Image message, or None.

    half: a compressed image may be decoded at half its width and height (several times
    faster for JPEG); raw images are always returned at full size.
    """
    if isinstance(msg, CompressedImage):
        flag = cv2.IMREAD_REDUCED_COLOR_2 if half else cv2.IMREAD_COLOR
        return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), flag)
    if msg.width == 0 or msg.height == 0:
        return None
    img = image_to_bgr(msg, depth_max_mm)
    if img is not None and img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


class SurroundStream(CameraStream):
    """Virtual camera: the four CSI cameras stitched into a bird's-eye view.

    While someone watches, it subscribes to the four CSI topics, keeps only the newest message
    of each, and composes at max_fps from a timer, so frames that arrive between two
    compositions are never decoded.
    """

    def __init__(self, node, sources, config_path):
        super().__init__(node, SURROUND_NAME, 'Surround view (CSI x4)',
                         ', '.join(cam.topic for cam in sources.values()), SURROUND_NAME)
        self.sources = sources  # side -> CameraStream, for the topic names
        self.config_path = config_path
        self.config = surround.load_config(config_path)
        self._model = None
        self._model_lock = threading.Lock()
        self._latest = {}   # side -> newest message
        self._frames = {}   # side -> decoded image
        self._dirty = False
        self.highlight = None  # side outlined in the view while calibrating

    def model(self):
        """The lookup tables, or None while another thread is still building them.

        Never waits for the lock: each re-subscription (page reload) adds a _compose timer,
        and timers blocked here used to take every executor thread, so no other callback of
        the node (buttons, drive watchdog, camera streams) ran any more.
        """
        if not self._model_lock.acquire(blocking=False):
            return None
        try:
            if self._model is None:
                self._model = surround.SurroundModel(self.config)
            return self._model
        finally:
            self._model_lock.release()

    def set_config(self, config, save, preview=False):
        """Apply a full config (from surround.make_config); rebuilds the lookup tables.

        Takes a few hundred ms (preview: a lower resolution, under 100 ms), so call it off
        the event loop.
        """
        size = min(config['size_px'], SURROUND_PREVIEW_PX) if preview else config['size_px']
        model = surround.SurroundModel(surround.merge_config(config, {'size_px': size}))
        with self._model_lock:
            self.config, self._model = config, model
        if save:
            surround.save_config(self.config_path, config)
        return config

    def _subscribe(self):
        node = self.node
        self._frames.clear()
        subs = []
        for side, cam in self.sources.items():
            def on_msg(msg, side=side):
                with self._lock:
                    self._latest[side] = msg
                    self._dirty = True
            if node.use_compressed:
                subs.append(node.create_subscription(
                    CompressedImage, cam.topic + '/compressed', on_msg,
                    qos_profile_sensor_data, callback_group=node.camera_group))
            else:
                subs.append(node.create_subscription(
                    Image, cam.topic, on_msg, qos_profile_sensor_data,
                    callback_group=node.camera_group))
        timer = node.create_timer(1.0 / node.max_fps, self._compose,
                                  callback_group=MutuallyExclusiveCallbackGroup())
        return subs, timer

    def _unsubscribe(self, handles):
        subs, timer = handles
        self.node.destroy_timer(timer)
        for sub in subs:
            self.node.destroy_subscription(sub)
        with self._lock:
            self._latest.clear()
            self._dirty = False
        self.highlight = None  # nobody is calibrating any more

    def _compose(self):
        with self._lock:
            if not self._dirty:
                return
            latest, self._latest = self._latest, {}
            self._dirty = False
        for side, msg in latest.items():
            img = decode_image(msg, self.node.depth_max_mm, half=self.node.surround_half_res)
            if img is not None:
                self._frames[side] = img
        model = self.model()
        if model is None:
            return
        try:
            self._encode_and_set(model.compose(self._frames, self.highlight))
        except cv2.error as e:
            self.node.get_logger().error(f'Surround view: {e}', throttle_duration_sec=10.0)


class ObstacleStream(CameraStream):
    """Virtual camera: top view of the obstacle points and the path being checked.

    The frames are drawn by QCar2WebNode._process_depth, only while someone watches.
    """

    def __init__(self, node, depth_topic):
        super().__init__(node, OBSTACLE_NAME, 'Obstacles (RealSense depth)', depth_topic,
                         OBSTACLE_NAME)

    @property
    def watched(self):
        return self._viewers > 0

    def _subscribe(self):
        return True  # QCar2WebNode subscribes to the depth topic

    def _unsubscribe(self, handle):
        pass


class PersonStream(ObstacleStream):
    """Virtual camera: the RealSense colour image with the detected people and the target.

    The frames are drawn by QCar2WebNode._process_person, only while someone watches.
    """

    def __init__(self, node, color_topic):
        CameraStream.__init__(self, node, PERSON_NAME, 'Person following (RealSense)',
                              color_topic, PERSON_NAME)


class MapStream(ObstacleStream):
    """Virtual camera: the SLAM map with the car, scan and plan.

    The frames are drawn by QCar2WebNode._process_nav, only while someone watches.
    """

    def __init__(self, node, map_topic):
        CameraStream.__init__(self, node, MAP_NAME, 'Map (SLAM + Nav2)', map_topic, MAP_NAME)


class DriveController:
    """Who may drive, the latest command, and the watchdog/e-stop logic.

    Commands are normalized: steer and throttle in [-1, 1]. Positive steer = left.
    """

    def __init__(self, command_timeout):
        self.command_timeout = command_timeout
        self._lock = threading.Lock()
        self._owner = None
        self._steer = 0.0
        self._throttle = 0.0
        self._last_cmd = 0.0
        self._estopped = False
        self._stop_until = 0.0
        self._auto = None   # None or one of AUTO_MODES

    def take(self, client_id):
        with self._lock:
            if self._owner is None and not self._estopped:
                self._owner = client_id
                self._steer = self._throttle = 0.0
                self._last_cmd = 0.0
            return self._owner == client_id

    def release(self, client_id):
        with self._lock:
            if self._owner == client_id:
                self._owner = None
                self._auto = None
                self._steer = self._throttle = 0.0
                self._stop_until = time.monotonic() + STOP_REPEAT_S

    def command(self, client_id, steer, throttle):
        """Returns True if this manual input switched auto mode off."""
        with self._lock:
            if self._owner != client_id or self._estopped:
                return False
            self._steer = _clamp_unit(steer)
            self._throttle = _clamp_unit(throttle)
            self._last_cmd = time.monotonic()
            # Any real joystick/keyboard input takes the car back from auto mode.
            if self._auto and max(abs(self._steer), abs(self._throttle)) > 0.05:
                self._auto = None
                return True
            return False

    def set_auto(self, client_id, mode):
        """mode: one of AUTO_MODES, or None for manual driving."""
        with self._lock:
            if self._owner != client_id or self._estopped:
                return False
            self._auto = mode if mode in AUTO_MODES else None
            return True

    def clear_auto(self, mode):
        """Leave auto mode `mode` (if still in it) without changing who drives."""
        with self._lock:
            if self._auto == mode:
                self._auto = None

    def estop(self):
        with self._lock:
            self._estopped = True
            self._owner = None
            self._auto = None
            self._steer = self._throttle = 0.0

    def reset_estop(self):
        with self._lock:
            self._estopped = False
            self._stop_until = time.monotonic() + STOP_REPEAT_S

    def output(self):
        """Normalized (steer, throttle, auto) to publish now, or None to stay silent.

        Staying silent when nobody drives lets other nodes (Nav2, autonomy) use the car.
        auto: the mode (one of AUTO_MODES) the browser holding control asked for, while it is
        still sending heartbeats, else None; steer and throttle are then 0 and the caller drives.
        """
        now = time.monotonic()
        with self._lock:
            if self._estopped:
                return 0.0, 0.0, None
            if self._owner is None:
                return (0.0, 0.0, None) if now < self._stop_until else None
            self._stop_until = now + STOP_REPEAT_S
            if now - self._last_cmd <= self.command_timeout:
                return self._steer, self._throttle, self._auto
            self._auto = None  # lost the browser: do not resume driving on reconnect
            return 0.0, 0.0, None

    def state_for(self, client_id):
        now = time.monotonic()
        with self._lock:
            if self._owner is None:
                control = 'none'
            elif self._owner == client_id:
                control = 'me'
            else:
                control = 'other'
            return {
                'control': control,
                'estop': self._estopped,
                'auto': self._auto,
                'stale': self._owner is not None and now - self._last_cmd > self.command_timeout,
            }


class QCar2WebNode(Node):

    def __init__(self):
        super().__init__('qcar2_web')
        self.host = self.declare_parameter('host', '0.0.0.0').value
        self.port = int(self.declare_parameter('port', 8080).value)
        self.password = str(self.declare_parameter('password', '').value or '')
        self.max_speed = float(self.declare_parameter('max_speed', 0.5).value)
        self.max_steer = min(float(self.declare_parameter('max_steer', 0.5).value), 0.6)
        self.invert_steering = bool(self.declare_parameter('invert_steering', False).value)
        command_timeout = float(self.declare_parameter('command_timeout', 0.5).value)
        self.use_compressed = bool(self.declare_parameter('use_compressed', True).value)
        self.max_fps = max(1.0, float(self.declare_parameter('max_fps', 15.0).value))
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 70).value)
        # Camera JPEGs come from image_transport at quality 95 and full size, far more than
        # Wi-Fi needs: re-encode them at jpeg_quality and at most stream_max_width px wide.
        self.recompress = bool(self.declare_parameter('recompress', True).value)
        # Surround view from the CSI JPEGs decoded at half size: about 40% less CPU, and the
        # 480 px bird's-eye view hardly shows the difference. False for full-size frames.
        self.surround_half_res = bool(self.declare_parameter('surround_half_res', True).value)
        self.stream_max_width = max(0, int(self.declare_parameter('stream_max_width', 640).value))
        self.depth_max_mm = max(1.0, float(self.declare_parameter('depth_max_mm', 4000.0).value))
        motor_topic = self.declare_parameter('motor_topic', '/qcar2_motor_speed_cmd').value
        battery_topic = self.declare_parameter('battery_topic', '/qcar2_battery').value

        surround_config = os.path.expanduser(self.declare_parameter(
            'surround_config', '~/.ros/qcar2_web_surround.json').value)
        self.obstacle_config_path = os.path.expanduser(self.declare_parameter(
            'obstacle_config', '~/.ros/qcar2_web_obstacle.json').value)
        self.person_config_path = os.path.expanduser(self.declare_parameter(
            'person_config', '~/.ros/qcar2_web_person.json').value)
        person_model = os.path.expanduser(self.declare_parameter(
            'person_model', '~/Documents/Quanser_demo/libraries/resources/pretrained_models/'
            'yolov8s-seg.engine').value)

        self.camera_group = ReentrantCallbackGroup()
        self.cameras = {}
        for name, label, topic, kind in CAMERA_DEFAULTS:
            topic = self.declare_parameter(f'topics.{name}', topic).value
            self.cameras[name] = CameraStream(self, name, label, topic, kind)
        self.surround = SurroundStream(
            self, {side: self.cameras[side] for side in surround.SIDES}, surround_config)
        self.obstacle_stream = ObstacleStream(self, self.cameras['depth'].topic)
        self.person_stream = PersonStream(self, self.cameras['realsense'].topic)
        self.cameras = {SURROUND_NAME: self.surround, **self.cameras,
                        OBSTACLE_NAME: self.obstacle_stream, PERSON_NAME: self.person_stream}

        self.drive = DriveController(command_timeout)
        self.last_output = None
        self.last_steer_rad = 0.0   # steering actually commanded, positive = left
        self.auto_plan = None       # (steer_rad, factor, free, candidates) while in auto mode

        self.obstacles = obstacle.ObstacleDetector(obstacle.load_config(self.obstacle_config_path))
        self.obstacle_status = None
        self._depth_lock = threading.Lock()
        self._depth_msg = None
        self._depth_sub = None
        self.create_timer(1.0 / OBSTACLE_RATE_HZ, self._process_depth,
                          callback_group=MutuallyExclusiveCallbackGroup())

        self.person = person.PersonFollower(
            person.load_config(self.person_config_path), person_model, self.get_logger())
        self._color_lock = threading.Lock()
        self._color_msg = None
        self._color_sub = None
        self._was_following = False
        # Own group: the first call loads the model, which takes a few seconds.
        self.create_timer(1.0 / PERSON_RATE_HZ, self._process_person,
                          callback_group=MutuallyExclusiveCallbackGroup())

        self.tuning = tuning.Store(os.path.expanduser(self.declare_parameter(
            'tuning_config', tuning.TUNING_PATH).value))
        self._param_clients = {}
        self._param_clients_lock = threading.Lock()

        self.nav = nav.NavBridge(
            self,
            map_topic=self.declare_parameter('map_topic', '/map').value,
            scan_topic=self.declare_parameter('scan_topic', '/scan').value,
            plan_topic=self.declare_parameter('plan_topic', '/plan').value,
            cmd_topic=self.declare_parameter('nav_cmd_topic', '/cmd_vel_nav').value,
            map_dir=self.declare_parameter('map_dir', tuning.MAPS_DIR).value,
            steer_mode=self.declare_parameter('nav_steer_mode', 'ackermann').value)
        self.nav.on_goal_done = lambda state: self.drive.clear_auto('nav')
        self.cloud = cloud3d.CloudStream(self, lambda: self.obstacles.config,
                                         lambda: self.nav.pose)
        self.map_stream = MapStream(self, self.nav.topics['map'])
        self.cameras[MAP_NAME] = self.map_stream
        self._was_nav = False
        self.create_timer(1.0 / NAV_RATE_HZ, self._process_nav,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.battery = {'voltage': None, 'percentage': None}

        self.motor_pub = self.create_publisher(MotorCommands, motor_topic, 1)
        # RGB LED strip colour (qcar2_hardware's qcar2_led_color), latched so the hardware
        # node gets the saved colour whenever it (re)starts.
        self.led_path = os.path.expanduser(self.declare_parameter(
            'led_config', '~/.ros/qcar2_web_led.json').value)
        self.led_pub = self.create_publisher(ColorRGBA, 'qcar2_led_color', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.led = self._load_led()
        if self.led is not None:
            self._publish_led()
        # qcar2_hardware publishes the battery at ~1 kHz; taking every message in Python ate a
        # whole core and starved the camera streams. Sample one message per second instead.
        self._battery_topic = battery_topic
        self._battery_sub = None
        self.create_timer(BATTERY_SAMPLE_S, self._sample_battery)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_drive,
                          callback_group=MutuallyExclusiveCallbackGroup())

        self.system = panel.SystemControl(self.get_logger())
        self._nav_mode_seen = None
        # lcd_rotate_180: our two LCD lines upside down (readable from the buttons' side).
        self.lcd = panel.LcdStatus(
            self.get_logger(), rotate_180=bool(self.declare_parameter('lcd_rotate_180', True).value)
        ) if bool(self.declare_parameter('lcd_status', True).value) else None
        self.buttons = panel.Buttons({
            0: (BUTTON_HOLD_S, lambda: self._button_action('S2', 'wifi_toggle')),
            1: (BUTTON_HOLD_S, self._button_mode),
            2: (BUTTON_HOLD_S, self._button_eco),
        })
        self.hil_buttons = panel.HilButtons(self.get_logger())
        # Own group, so the buttons never wait behind other callbacks.
        button_group = MutuallyExclusiveCallbackGroup()
        self._lcd_holding = None
        self.create_subscription(
            UInt8, self.declare_parameter('buttons_topic', '/qcar2_user_buttons').value,
            lambda msg: self.buttons.update(msg.data), 10, callback_group=button_group)
        self.create_timer(0.1, self._poll_buttons, callback_group=button_group)
        self.create_timer(HIL_BUTTONS_S, self._read_hil_buttons, callback_group=button_group)
        # Eco mode: the camera services run only while used (see panel.EcoSensors).
        self.eco = panel.EcoSensors(self.get_logger(), self.declare_parameter(
            'eco_config', '~/.ros/qcar2_web_eco.json').value)
        self.create_timer(1.0, self._update_sensors, callback_group=MutuallyExclusiveCallbackGroup())
        # Own group: refresh() runs nmcli and systemctl.
        self.create_timer(PANEL_REFRESH_S, self._refresh_panel,
                          callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info(
            f'max_speed={self.max_speed} m/s  max_steer={self.max_steer} rad  '
            f'timeout={command_timeout}s  password={"on" if self.password else "off"}')

    def _button_action(self, button, action):
        if self.system.start(action):
            self.get_logger().info(f'Button {button}: {panel.ACTIONS[action][0]}')
            self._update_lcd()   # show "Switching Wi-Fi..." now, not at the next refresh

    def _ros_mode(self):
        return self.system.status()['robot'] in ('active', 'activating', 'deactivating')

    def _button_mode(self):
        """S3: ROS nodes running -> stop them for MATLAB; stopped -> start them."""
        self._button_action('S3', 'matlab' if self._ros_mode() else 'ros')

    def _read_hil_buttons(self):
        """MATLAB mode: qcar2_hardware is stopped, so read the buttons from the card."""
        st = self.system.status()
        if st['robot'] is None or self._ros_mode() or st['busy']:
            return   # unknown yet, the hardware node owns the card, or a switch is running
        mask = self.hil_buttons.read()
        if mask is not None:
            self.buttons.update(mask)

    def _poll_buttons(self):
        self.buttons.poll()
        # While S2/S3 is held, count down on the LCD so it is clear the press registered.
        holding = self.buttons.holding()
        names = {0: 'S2 Wi-Fi', 1: 'S3 MATLAB' if self._ros_mode() else 'S3 start ROS',
                 2: 'S4 ECO off' if self.eco.eco else 'S4 ECO on'}
        text = None
        for bit, held in sorted(holding.items()):
            if bit in names:
                left = max(0.0, BUTTON_HOLD_S - held)
                text = f'Hold {names[bit]} {math.ceil(left)}s'
                break
        if text != self._lcd_holding:
            self._lcd_holding = text
            self._update_lcd()

    def _load_led(self):
        """Saved {'r','g','b' 0..255, 'brightness' 0..100}, or None (keep led_color_id)."""
        try:
            with open(self.led_path) as f:
                d = json.load(f)
            return {k: max(0, min(255 if k != 'brightness' else 100, int(d[k])))
                    for k in ('r', 'g', 'b', 'brightness')}
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def _publish_led(self):
        led = self.led
        self.led_pub.publish(ColorRGBA(r=led['r'] / 255.0, g=led['g'] / 255.0, b=led['b'] / 255.0,
                                       a=led['brightness'] / 100.0))

    def set_led(self, led):
        self.led = led
        self._publish_led()
        tmp = self.led_path + '.tmp'
        os.makedirs(os.path.dirname(self.led_path) or '.', exist_ok=True)
        with open(tmp, 'w') as f:
            json.dump(led, f)
        os.replace(tmp, self.led_path)

    def _button_eco(self):
        """S4: toggle eco mode (cameras only while used)."""
        self.eco.set_eco(not self.eco.eco)
        self.get_logger().info(f'Button S4: eco {"on" if self.eco.eco else "off"}')
        self._update_lcd()

    def _sensors_wanted(self):
        """Which camera services something uses right now (for eco mode)."""
        cams = self.cameras
        csi = self.surround._viewers > 0 or any(
            cams[side]._viewers > 0 for side in surround.SIDES)
        realsense = (cams['realsense']._viewers > 0 or cams['depth']._viewers > 0
                     or self._depth_wanted() or self._person_wanted()
                     or self.cloud.viewers > 0
                     or self.system.status()['nav_method'] == 'vslam')
        return {'csi': csi, 'realsense': realsense}

    def _update_sensors(self):
        self.eco.update(bool(self.system.status()['busy']), self._sensors_wanted())

    def _refresh_panel(self):
        self.system.refresh()
        st = self.system.status()
        mode = (st['nav_mode'], st['nav_method'], st['nav_map'])
        if mode != self._nav_mode_seen:
            if self._nav_mode_seen is not None:
                # SLAM <-> navigation <-> off: the old map and goal no longer apply.
                self.get_logger().info(f'Nav mode: {mode[0]} {mode[1] or ""} {mode[2] or ""}')
                self.nav.cancel()
                self.nav.forget_map()
            self._nav_mode_seen = mode
        self._update_lcd()

    def _update_lcd(self):
        if self.lcd is None:
            return
        st = self.system.status()
        if st['robot'] != 'active' and not st['busy']:
            # MATLAB mode: say so once, then leave the display to the model.
            lines = ('ROS off: hold S3 = start',
                     self._lcd_holding or f'Web {st["ip"] or "-"}:{self.port}')
        else:
            drive = self.drive.state_for(None)
            if drive['estop']:
                driving = 'E-STOP'
            elif drive['auto']:
                driving = 'AUTO ' + drive['auto']
            else:
                driving = 'DRIVING' if drive['control'] != 'none' else ''
            wifi = 'HOTSPOT' if st['wifi'] == 'demo' else 'WIFI'
            if st['busy']:
                status = panel.ACTIONS_LCD.get(st['busy'], 'Please wait...')
            elif self._lcd_holding:
                status = self._lcd_holding
            else:
                stack = {'slam': 'SLAM', 'nav': 'NAV'}.get(st['nav_mode'], 'ROS')
                if st['nav_method'] == 'vslam':
                    stack = 'V' + stack
                if self.eco.eco:
                    wifi = ('AP' if wifi == 'HOTSPOT' else wifi) + ' ECO'
                status = f'{wifi} {stack} {driving}'
            lines = (f'Web {st["ip"] or "-"}:{self.port}', status)
        self.lcd.show(lines)

    def _sample_battery(self):
        if self._battery_sub is None:
            self._battery_sub = self.create_subscription(
                BatteryState, self._battery_topic, self._on_battery, 1)

    def _on_battery(self, msg):
        self.battery = {
            'voltage': _finite_or_none(msg.voltage),
            'percentage': _finite_or_none(msg.percentage),
        }
        # Same (default) callback group as the timer, so this cannot race _sample_battery.
        sub, self._battery_sub = self._battery_sub, None
        if sub is not None:
            self.destroy_subscription(sub)

    def apply_nav_live(self, changed):
        """Send Nav2 values to the running nodes. Blocking: call it off the event loop.

        Returns {key: {'ok': bool|None, 'reason': str|None}}; ok None = not applied live
        (Nav2 is not running, or the value is only read at start). Either way it is saved.
        """
        results = {}
        for node_name, items in tuning.nav_live_updates(changed).items():
            with self._param_clients_lock:
                client = self._param_clients.get(node_name)
                if client is None:
                    client = self._param_clients[node_name] = self.create_client(
                        SetParameters, f'/{node_name}/set_parameters')
            outcome = [(False, 'Nav2 ยังไม่ทำงาน')] * len(items)
            if client.wait_for_service(timeout_sec=0.5):
                request = SetParameters.Request()
                request.parameters = [Parameter(name=name, value=_parameter_value(value))
                                      for name, value in items]
                done = threading.Event()
                future = client.call_async(request)
                future.add_done_callback(lambda f: done.set())
                if done.wait(5.0) and future.result() is not None:
                    outcome = [(r.successful, r.reason) for r in future.result().results]
                else:
                    outcome = [(False, 'ไม่ตอบสนอง')] * len(items)
            for key, value in changed.items():
                for target_node, name, fn in tuning.NAV_TARGETS.get(key, ()):
                    if target_node != node_name:
                        continue
                    for (item_name, _), (ok, reason) in zip(items, outcome):
                        if item_name == name:
                            entry = results.setdefault(key, {'ok': True, 'reason': None})
                            if not ok:
                                entry['ok'] = None if reason == 'Nav2 ยังไม่ทำงาน' else False
                                entry['reason'] = reason or 'Nav2 ไม่รับค่านี้ขณะทำงาน'
        return results

    def _depth_wanted(self):
        return (self.obstacles.config['enabled'] or self.obstacle_stream.watched
                or self.auto_plan is not None or self.drive.state_for(None)['auto']
                or self.person_stream.watched)

    def _person_wanted(self):
        return self.person_stream.watched or self.drive.state_for(None)['auto'] == 'follow'

    def _on_color(self, msg):
        with self._color_lock:
            self._color_msg = msg

    def _process_person(self):
        """Subscribe to the RealSense colour image while needed and run the person detector."""
        following = self.drive.state_for(None)['auto'] == 'follow'
        if following and not self._was_following:
            self.person.reset_target()   # follow whoever is in front now
        self._was_following = following
        topic = self.person_stream.topic
        if self._person_wanted():
            if self._color_sub is None:
                if self.use_compressed:
                    self._color_sub = self.create_subscription(
                        CompressedImage, topic + '/compressed', self._on_color,
                        qos_profile_sensor_data, callback_group=self.camera_group)
                else:
                    self._color_sub = self.create_subscription(
                        Image, topic, self._on_color, qos_profile_sensor_data,
                        callback_group=self.camera_group)
                self.get_logger().info(f'Person: subscribed to {topic}')
        elif self._color_sub is not None:
            self.destroy_subscription(self._color_sub)
            self._color_sub = None
            with self._color_lock:
                self._color_msg = None
            self.get_logger().info('Person: colour image not needed, unsubscribed')
            return
        else:
            return

        if not self.person.ensure_model():
            return
        with self._color_lock:
            msg, self._color_msg = self._color_msg, None
        if msg is None:
            return
        color = decode_image(msg, self.depth_max_mm)
        if color is None:
            return
        depth = self.obstacles.latest_depth_m()
        obs_cfg = self.obstacles.config
        if depth is not None:
            depth_hfov = obstacle._hfov(obs_cfg, depth.shape[1], depth.shape[0])
        else:
            depth_hfov = 87.0
        self.person.process(color, depth, obs_cfg, depth_hfov)
        if self.person_stream.watched:
            self.person_stream._encode_and_set(self.person.render(color))

    def _process_nav(self):
        """Subscribe to the map, scan, plan and TF while needed and draw the map view."""
        watched = self.map_stream.watched
        self.nav.set_wanted(watched or self.drive.state_for(None)['auto'] == 'nav'
                            or time.monotonic() < self.nav.keep_until or self.cloud.viewers > 0)
        if self.nav._subs is None:
            return
        cfg = self.obstacles.config
        img = self.nav.update(watched, cfg['car_length'], cfg['car_width'])
        if img is not None:
            self.map_stream._encode_and_set(img)

    def _on_depth(self, msg):
        with self._depth_lock:
            self._depth_msg = msg

    def _process_depth(self):
        """Subscribe to depth while it is needed, and turn the newest frame into obstacle points."""
        if self._depth_wanted():
            if self._depth_sub is None:
                self._depth_sub = self.create_subscription(
                    Image, self.obstacle_stream.topic, self._on_depth, qos_profile_sensor_data,
                    callback_group=self.camera_group)
                self.get_logger().info(f'Obstacles: subscribed to {self.obstacle_stream.topic}')
        elif self._depth_sub is not None:
            self.destroy_subscription(self._depth_sub)
            self._depth_sub = None
            self.obstacle_status = None
            self.get_logger().info('Obstacles: depth not needed, unsubscribed')
            return
        else:
            return

        with self._depth_lock:
            msg, self._depth_msg = self._depth_msg, None
        if msg is not None and msg.width and msg.height:
            if msg.encoding.lower() in ('mono16', '16uc1'):
                dtype = '>u2' if msg.is_bigendian else '<u2'
                raw = np.frombuffer(msg.data, dtype=np.uint8).view(dtype).reshape(
                    msg.height, msg.step // 2)[:, :msg.width]
                self.obstacles.process(raw, self.obstacles.config['depth_scale'])
            else:
                self.get_logger().warn(f'Obstacles: unsupported depth encoding {msg.encoding}',
                                       throttle_duration_sec=10.0)

        plan = self.auto_plan
        steer = plan[0] if plan else self.last_steer_rad
        status = self.obstacles.assess(steer)
        if plan:
            status['free'] = plan[2]
        status['enabled'] = self.obstacles.config['enabled']
        self.obstacle_status = status
        if self.obstacle_stream.watched:
            cfg, ox, oy, oz, blind, _ = self.obstacles.snapshot()
            self.obstacle_stream._encode_and_set(obstacle.render_topdown(
                cfg, ox, oy, oz, steer, status['free'], blind, status['stale'],
                plan[3] if plan else ()))

    def _publish_drive(self):
        out = self.drive.output()
        navigating = out is not None and out[2] == 'nav'
        if self._was_nav and not navigating:
            self.nav.cancel()   # manual input, release, E-STOP or lost browser
        self._was_nav = navigating
        if out is None:
            self.last_output = None
            self.auto_plan = None
            return
        steer, throttle, auto = out
        steer_rad = steer * self.max_steer
        speed = throttle * self.max_speed
        cfg = self.obstacles.config
        if auto == 'follow':
            self.auto_plan = None
            steer_rad, frac = self.person.command(cfg, self.max_steer)
            speed = frac * self.max_speed
            # The brake assist always applies here: it also stops for a blind or missing depth view.
            if speed > 0:
                speed *= self.obstacles.assess(steer_rad)['factor']
        elif auto == 'nav':
            self.auto_plan = None
            steer_rad, speed = self.nav.command(cfg['wheelbase'], self.max_steer, self.max_speed)
            if speed > 0 and cfg['enabled']:
                speed *= self.obstacles.assess(steer_rad)['factor']
        elif auto:
            prev = self.auto_plan[0] if self.auto_plan else 0.0
            self.auto_plan = self.obstacles.plan(self.max_steer, prev)
            steer_rad, factor = self.auto_plan[0], self.auto_plan[1]
            speed = cfg['auto_speed'] * self.max_speed * factor
        else:
            self.auto_plan = None
            if speed > 0 and cfg['enabled']:
                speed *= self.obstacles.assess(steer_rad)['factor']
        self.last_steer_rad = steer_rad
        self.publish_motor(-steer_rad if self.invert_steering else steer_rad, speed)

    def publish_motor(self, steer_rad, speed_mps):
        msg = MotorCommands()
        # qcar2_hardware reads values by position: [0] = steering, [1] = throttle
        msg.motor_names = ['steering_angle', 'motor_throttle']
        msg.values = [float(steer_rad), float(speed_mps)]
        self.motor_pub.publish(msg)
        self.last_output = {'steer_rad': steer_rad, 'speed_mps': speed_mps}

    def status_for(self, client_id):
        status = self.drive.state_for(client_id)
        status.update({
            'type': 'status',
            'battery': self.battery,
            'output': self.last_output,
            'obstacle': self.obstacle_status,
            'person': self.person.status() if self._color_sub is not None else None,
            'nav': self.nav.status() if self.nav._subs is not None else None,
            'system': self.system.status(),
            'eco': self.eco.status(),
        })
        return status


# ---------------------------------------------------------------- HTTP / WebSocket

@web.middleware
async def auth_middleware(request, handler):
    password = request.app['node'].password
    if password:
        ok = False
        header = request.headers.get('Authorization', '')
        if header.startswith('Basic '):
            try:
                given = base64.b64decode(header[6:]).decode('utf-8').partition(':')[2]
                ok = hmac.compare_digest(given, password)
            except (ValueError, UnicodeDecodeError):
                ok = False
        if not ok:
            raise web.HTTPUnauthorized(headers={'WWW-Authenticate': 'Basic realm="QCar2"'})
    return await handler(request)


async def index_handler(request):
    return web.FileResponse(os.path.join(request.app['static_dir'], 'index.html'),
                            headers={'Cache-Control': 'no-cache'})


async def info_handler(request):
    node = request.app['node']
    return web.json_response({
        'cameras': [cam.info() for cam in node.cameras.values()],
        'limits': {
            'max_speed': node.max_speed,
            'max_steer': node.max_steer,
            'command_timeout': node.drive.command_timeout,
        },
    })


async def surround_get_handler(request):
    return web.json_response(request.app['node'].surround.config)


async def surround_post_handler(request):
    """Body: {"config": {...partial config...}, "save": bool, "reset": bool,
    "preview": bool, "highlight": side or null}.

    preview builds a lower-resolution view (fast, for slider drags); send the last change
    again without it. A body with only "highlight" does not rebuild anything.
    """
    stream = request.app['node'].surround
    try:
        body = await request.json()
    except ValueError:
        raise web.HTTPBadRequest(text='invalid JSON')
    if not isinstance(body, dict):
        raise web.HTTPBadRequest(text='expected an object')
    async with request.app['surround_lock']:
        if 'highlight' in body:
            side = body['highlight']
            stream.highlight = side if side in surround.SIDES else None
        if body.get('reset') or body.get('save') or 'config' in body:
            if body.get('reset'):
                config = surround.make_config()
            else:
                config = surround.merge_config(stream.config, body.get('config'))
            try:
                await asyncio.get_running_loop().run_in_executor(
                    None, stream.set_config, config, bool(body.get('save')),
                    bool(body.get('preview')))
            except OSError as e:
                raise web.HTTPInternalServerError(
                    text=f'cannot save {stream.config_path}: {e.strerror}')
    return web.json_response(stream.config)


async def obstacle_get_handler(request):
    return web.json_response(request.app['node'].obstacles.config)


async def obstacle_post_handler(request):
    """Body: {"config": {...partial config...}, "save": bool, "reset": bool, "fit_floor": bool}.

    fit_floor estimates the camera height, pitch and roll from the floor in the current depth
    frame and applies them (not saved); the response then also has "fit".
    """
    node = request.app['node']
    try:
        body = await request.json()
    except ValueError:
        raise web.HTTPBadRequest(text='invalid JSON')
    if not isinstance(body, dict):
        raise web.HTTPBadRequest(text='expected an object')
    det = node.obstacles
    result = {}
    if body.get('reset'):
        config = obstacle.make_config()
    else:
        config = obstacle.merge_config(det.config, body.get('config'))
    if body.get('fit_floor'):
        depth = det.latest_depth_m()
        if depth is None:
            raise web.HTTPConflict(text='ยังไม่มีภาพ depth (node RealSense ทำงานอยู่ไหม?)')
        try:
            fit = await asyncio.get_running_loop().run_in_executor(
                None, obstacle.fit_floor, config, depth)
        except ValueError as e:
            raise web.HTTPConflict(text=str(e))
        config = obstacle.merge_config(config, fit)
        result['fit'] = fit
    det.set_config(config)
    if body.get('save'):
        try:
            surround.save_config(node.obstacle_config_path, config)
        except OSError as e:
            raise web.HTTPInternalServerError(
                text=f'cannot save {node.obstacle_config_path}: {e.strerror}')
    result['config'] = config
    return web.json_response(result)


async def person_get_handler(request):
    return web.json_response(request.app['node'].person.config)


async def person_post_handler(request):
    """Body: {"config": {...partial config...}, "save": bool, "reset": bool}."""
    node = request.app['node']
    try:
        body = await request.json()
    except ValueError:
        raise web.HTTPBadRequest(text='invalid JSON')
    if not isinstance(body, dict):
        raise web.HTTPBadRequest(text='expected an object')
    if body.get('reset'):
        config = person.make_config()
    else:
        config = person.merge_config(node.person.config, body.get('config'))
    node.person.set_config(config)
    if body.get('save'):
        try:
            surround.save_config(node.person_config_path, config)
        except OSError as e:
            raise web.HTTPInternalServerError(
                text=f'cannot save {node.person_config_path}: {e.strerror}')
    return web.json_response(config)


async def system_post_handler(request):
    """Body: {"action": one of panel.ACTIONS}. Starts it; progress shows in the status."""
    node = request.app['node']
    try:
        body = await request.json()
    except ValueError:
        raise web.HTTPBadRequest(text='invalid JSON')
    action = body.get('action') if isinstance(body, dict) else None
    if action not in panel.ACTIONS:
        raise web.HTTPBadRequest(text='unknown action')
    map_name = body.get('map') if action in panel.MAP_ACTIONS else None
    if action in panel.MAP_ACTIONS:
        info = {m['name']: m for m in panel.map_info(node.nav.map_dir)}
        if map_name not in info:
            raise web.HTTPBadRequest(text='ไม่พบแผนที่นี้')
        if action == 'nav_vmap' and not info[map_name]['vslam']:
            raise web.HTTPBadRequest(text='แผนที่นี้ทำด้วย Lidar ไม่มีข้อมูลกล้อง จึงนำทางแบบ V-SLAM ไม่ได้')
    if not node.system.start(action, map_name):
        raise web.HTTPConflict(text='กำลังทำคำสั่งก่อนหน้าอยู่')
    return web.json_response({'started': action})


async def led_handler(request):
    """GET: the LED strip colour. POST {"r","g","b": 0..255, "brightness": 0..100}: set it."""
    node = request.app['node']
    if request.method == 'POST':
        try:
            body = await request.json()
            led = {k: int(body[k]) for k in ('r', 'g', 'b', 'brightness')}
        except (ValueError, KeyError, TypeError):
            raise web.HTTPBadRequest(text='expected {"r","g","b": 0..255, "brightness": 0..100}')
        if not all(0 <= led[k] <= 255 for k in 'rgb') or not 0 <= led['brightness'] <= 100:
            raise web.HTTPBadRequest(text='out of range')
        try:
            node.set_led(led)
        except OSError as e:
            raise web.HTTPInternalServerError(text=f'cannot save: {e.strerror}')
    return web.json_response(node.led or {})


async def eco_handler(request):
    """Body: {"on": bool}. Eco mode: cameras run only while used."""
    node = request.app['node']
    try:
        body = await request.json()
    except ValueError:
        raise web.HTTPBadRequest(text='invalid JSON')
    if not isinstance(body, dict) or not isinstance(body.get('on'), bool):
        raise web.HTTPBadRequest(text='expected {"on": true|false}')
    node.eco.set_eco(body['on'])
    return web.json_response(node.eco.status())


async def maps_handler(request):
    """{"maps": [names, newest first], "info": [per-map details], "dir": map directory}."""
    node = request.app['node']
    return web.json_response({'maps': panel.list_maps(node.nav.map_dir),
                              'info': panel.map_info(node.nav.map_dir),
                              'dir': os.path.expanduser(node.nav.map_dir)})


async def map_save_handler(request):
    """Body: {"name": str, "overwrite": bool}. Saves the current SLAM map (lidar or camera);
    responds {"files": [...], "warning": str or null}."""
    node = request.app['node']
    try:
        body = await request.json()
    except ValueError:
        body = {}
    if not isinstance(body, dict):
        body = {}
    st = node.system.status()
    if st['nav_mode'] != 'slam':
        raise web.HTTPConflict(text='บันทึกแผนที่ได้เฉพาะตอนกำลังทำแผนที่')
    name = nav.clean_map_name(str(body.get('name') or ''))
    if name in panel.list_maps(node.nav.map_dir) and not body.get('overwrite'):
        raise web.HTTPConflict(text=f'EXISTS:{name}')
    try:
        files, warning = await asyncio.get_running_loop().run_in_executor(
            None, node.nav.save_map, name, st['nav_method'] or 'lidar')
    except ValueError as e:
        raise web.HTTPConflict(text=str(e))
    except OSError as e:
        raise web.HTTPInternalServerError(text=f'cannot save map: {e.strerror}')
    node.get_logger().info(f'Map saved: {", ".join(files)}')
    return web.json_response({'files': files, 'warning': warning})


async def map_delete_handler(request):
    """Body: {"name": str}. Deletes a saved map (all its files)."""
    node = request.app['node']
    try:
        body = await request.json()
    except ValueError:
        body = {}
    name = body.get('name') if isinstance(body, dict) else None
    st = node.system.status()
    if st['nav_mode'] == 'nav' and st['nav_map'] == name:
        raise web.HTTPConflict(text='แผนที่นี้กำลังใช้นำทางอยู่ ปิดการนำทางก่อน')
    try:
        removed = panel.delete_map(node.nav.map_dir, name)
    except ValueError as e:
        raise web.HTTPConflict(text=str(e))
    except OSError as e:
        raise web.HTTPInternalServerError(text=f'cannot delete: {e.strerror}')
    node.get_logger().info(f'Map deleted: {", ".join(removed)}')
    return web.json_response({'removed': removed})


async def tuning_get_handler(request):
    """{"schema": {group: {label, params, presets}}, "values": {group: {key: value}}}."""
    store = request.app['node'].tuning
    return web.json_response({'schema': tuning.schema_json(),
                              'values': {g: store.values(g) for g in tuning.GROUPS}})


async def tuning_post_handler(request):
    """Body: {"values": {...}} or {"preset": name} or {"reset": true}.

    Saves the values of the group. For 'nav' they are also sent to the running Nav2 nodes;
    'live' in the response says per value whether that worked. The mapping groups
    (cartographer, vslam) are read when mapping starts.
    """
    node = request.app['node']
    group = request.match_info['group']
    if group not in tuning.GROUPS:
        raise web.HTTPNotFound()
    try:
        body = await request.json()
    except ValueError:
        raise web.HTTPBadRequest(text='invalid JSON')
    if not isinstance(body, dict):
        raise web.HTTPBadRequest(text='expected an object')
    store = node.tuning
    everything = False
    try:
        if body.get('reset'):
            store.reset(group)
            everything = True
        elif body.get('preset') is not None:
            preset = tuning.PRESETS[group].get(body['preset'])
            if preset is None:
                raise web.HTTPBadRequest(text='unknown preset')
            store.update(group, preset['values'], replace=True)
            everything = True
        else:
            store.update(group, body.get('values'))
    except OSError as e:
        raise web.HTTPInternalServerError(text=f'cannot save tuning: {e.strerror}')
    values = store.values(group)
    live = {}
    if group == 'nav':
        if everything:
            changed = values
        else:
            changed = {k: v for k, v in values.items() if k in tuning.clean(group, body.get('values'))}
        live = await asyncio.get_running_loop().run_in_executor(None, node.apply_nav_live, changed)
    return web.json_response({'values': values, 'live': live})


async def stream_handler(request):
    node = request.app['node']
    cam = node.cameras.get(request.match_info['name'])
    if cam is None:
        raise web.HTTPNotFound()

    resp = web.StreamResponse(headers={
        'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
        'Cache-Control': 'no-cache, no-store, must-revalidate',
        'Pragma': 'no-cache',
    })
    await resp.prepare(request)
    cam.add_viewer()
    seq = -1
    try:
        while True:
            jpeg, new_seq = await cam.wait_frame(seq, timeout=1.0)
            if request.transport is None or request.transport.is_closing():
                break
            if jpeg is None or new_seq == seq:
                continue
            seq = new_seq
            await resp.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                             + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
    except (ConnectionResetError, ConnectionError):
        pass
    finally:
        cam.remove_viewer()
    return resp


async def _send_frames(ws, cam, flow):
    seq = -1
    try:
        while not ws.closed:
            # Wait for an ack before picking the frame, so the one sent is the newest.
            while flow['inflight'] >= STREAM_WINDOW:
                flow['acked'].clear()
                try:
                    await asyncio.wait_for(flow['acked'].wait(), STREAM_ACK_TIMEOUT_S)
                except asyncio.TimeoutError:
                    flow['inflight'] = 0
            jpeg, new_seq = await cam.wait_frame(seq, timeout=1.0)
            if jpeg is None or new_seq == seq:
                continue
            seq = new_seq
            flow['inflight'] += 1
            await ws.send_bytes(jpeg)
    except (ConnectionResetError, ConnectionError, RuntimeError):
        pass


async def ws_stream_handler(request):
    """Camera frames as binary WebSocket messages, one JPEG each.

    The page uses this instead of /stream: browsers allow only 6 HTTP connections per host
    and do not always close an MJPEG <img> when its src changes, so after switching cameras a
    few times new streams queued forever and the view froze. A WebSocket closes for certain
    and does not count towards that limit.

    The page sends a (any) message for each frame it has shown or dropped; see STREAM_WINDOW.
    """
    cam = request.app['node'].cameras.get(request.match_info['name'])
    if cam is None:
        raise web.HTTPNotFound()
    ws = web.WebSocketResponse(heartbeat=5.0)
    await ws.prepare(request)
    cam.add_viewer()
    flow = {'inflight': 0, 'acked': asyncio.Event()}
    sender = asyncio.ensure_future(_send_frames(ws, cam, flow))
    try:
        async for _ in ws:  # frame acks; also notices when the client goes away
            flow['inflight'] = max(0, flow['inflight'] - 1)
            flow['acked'].set()
    finally:
        sender.cancel()
        cam.remove_viewer()
    return ws


async def ws_cloud_handler(request):
    """3D point clouds (see cloud3d.py) as binary messages; the page acks each one."""
    cloud = request.app['node'].cloud
    ws = web.WebSocketResponse(heartbeat=5.0)
    await ws.prepare(request)
    cloud.add_viewer()
    flow = {'inflight': 0}

    async def sender():
        seq = -1
        waited = 0.0
        try:
            while not ws.closed:
                await asyncio.sleep(0.1)
                new_seq, data = cloud.frame()
                if data is None or new_seq == seq:
                    continue
                if flow['inflight'] >= 2:
                    waited += 0.1
                    if waited < 3.0:
                        continue
                    flow['inflight'] = 0
                waited = 0.0
                seq = new_seq
                flow['inflight'] += 1
                await ws.send_bytes(data)
        except (ConnectionResetError, ConnectionError, RuntimeError):
            pass

    task = asyncio.ensure_future(sender())
    try:
        async for _ in ws:
            flow['inflight'] = max(0, flow['inflight'] - 1)
    finally:
        task.cancel()
        cloud.remove_viewer()
    return ws


async def _push_status(ws, node, client_id):
    try:
        while not ws.closed:
            await ws.send_str(json.dumps(node.status_for(client_id)))
            await asyncio.sleep(0.2)
    except (ConnectionResetError, ConnectionError, RuntimeError):
        pass


async def _nav_goal(node, client_id, data, ws):
    """Goal picked on the map: u0/v0 = position, u1/v1 = heading, as fractions of the frame."""
    coords = [_finite_or_none(data.get(k)) for k in ('u0', 'v0', 'u1', 'v1')]
    error = None
    if None in coords:
        error = 'พิกัดไม่ถูกต้อง'
    elif node.drive.state_for(client_id)['control'] != 'me':
        error = 'ต้องรับสิทธิ์ควบคุมก่อน'
    else:
        goal = node.nav.goal_from_view(*coords)
        if goal is None:
            error = 'ยังไม่มีแผนที่'
        else:
            error = node.nav.send_goal(*goal)
            if error is None and not node.drive.set_auto(client_id, 'nav'):
                node.nav.cancel()
                error = 'ส่งเป้าหมายไม่ได้ (E-STOP?)'
    if error:
        await ws.send_str(json.dumps({'type': 'nav_error', 'text': error}))


async def _nav_initial_pose(node, data, ws):
    """Start pose for AMCL picked on the map, like _nav_goal."""
    coords = [_finite_or_none(data.get(k)) for k in ('u0', 'v0', 'u1', 'v1')]
    error = None
    if None in coords:
        error = 'พิกัดไม่ถูกต้อง'
    elif node.system.status()['nav_mode'] != 'nav':
        error = 'ตั้งตำแหน่งรถได้เฉพาะตอนนำทางบนแผนที่ที่โหลด'
    else:
        pose = node.nav.goal_from_view(*coords)
        if pose is None:
            error = 'ยังไม่มีแผนที่'
        else:
            node.nav.set_initial_pose(*pose)
    if error:
        await ws.send_str(json.dumps({'type': 'nav_error', 'text': error}))


async def ws_handler(request):
    node = request.app['node']
    ws = web.WebSocketResponse(heartbeat=5.0)
    await ws.prepare(request)
    client_id = next(request.app['client_ids'])
    pusher = asyncio.ensure_future(_push_status(ws, node, client_id))
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
            except ValueError:
                continue
            if not isinstance(data, dict):
                continue
            kind = data.get('type')
            if kind == 'drive':
                if node.drive.command(client_id, data.get('steer'), data.get('throttle')):
                    node.get_logger().info(f'Client {client_id}: manual input, auto mode off')
            elif kind in AUTO_MODES or kind == 'auto':   # 'auto' = 'avoid', older pages
                mode = ('avoid' if kind == 'auto' else kind) if data.get('on') else None
                if node.drive.set_auto(client_id, mode):
                    node.get_logger().info(f'Client {client_id}: auto mode {mode or "off"}')
            elif kind == 'nav_goal':
                await _nav_goal(node, client_id, data, ws)
            elif kind == 'nav_initial_pose':
                await _nav_initial_pose(node, data, ws)
            elif kind == 'take':
                if node.drive.take(client_id):
                    node.get_logger().info(f'Client {client_id} took control')
            elif kind == 'release':
                node.drive.release(client_id)
            elif kind == 'estop':
                node.drive.estop()
                node.get_logger().warn(f'E-STOP pressed by client {client_id}')
            elif kind == 'estop_reset':
                node.drive.reset_estop()
                node.get_logger().info(f'E-STOP reset by client {client_id}')
    finally:
        pusher.cancel()
        node.drive.release(client_id)
    return ws


def make_app(node):
    app = web.Application(middlewares=[auth_middleware])
    app['node'] = node
    app['static_dir'] = os.path.join(get_package_share_directory('qcar2_web'), 'static')
    app['client_ids'] = itertools.count(1)
    app['surround_lock'] = asyncio.Lock()
    app.router.add_get('/', index_handler)
    app.router.add_get('/api/info', info_handler)
    app.router.add_get('/stream/{name}', stream_handler)
    app.router.add_get('/api/surround', surround_get_handler)
    app.router.add_post('/api/surround', surround_post_handler)
    app.router.add_get('/api/obstacle', obstacle_get_handler)
    app.router.add_post('/api/obstacle', obstacle_post_handler)
    app.router.add_get('/api/person', person_get_handler)
    app.router.add_post('/api/person', person_post_handler)
    app.router.add_post('/api/map/save', map_save_handler)
    app.router.add_post('/api/map/delete', map_delete_handler)
    app.router.add_get('/api/tuning', tuning_get_handler)
    app.router.add_post('/api/tuning/{group}', tuning_post_handler)
    app.router.add_get('/api/maps', maps_handler)
    app.router.add_post('/api/eco', eco_handler)
    app.router.add_get('/api/led', led_handler)
    app.router.add_post('/api/led', led_handler)
    app.router.add_post('/api/system', system_post_handler)
    app.router.add_get('/ws', ws_handler)
    app.router.add_get('/ws/stream/{name}', ws_stream_handler)
    app.router.add_get('/ws/cloud', ws_cloud_handler)
    app.router.add_static('/static/', app['static_dir'])
    return app


def local_ipv4_addresses():
    """IPv4 address of every non-loopback interface that has one."""
    addrs = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        for _, ifname in socket.if_nameindex():
            try:
                req = struct.pack('256s', ifname.encode()[:15])
                addr = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, req)[20:24])  # SIOCGIFADDR
            except OSError:
                continue  # interface has no IPv4 address
            if not addr.startswith('127.'):
                addrs.append(addr)
    return addrs


async def serve(node):
    loop = asyncio.get_running_loop()
    for cam in node.cameras.values():
        cam.loop = loop

    stop = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    runner = web.AppRunner(make_app(node))
    await runner.setup()
    # The port may still be held for a moment by a previous instance that is shutting down.
    for attempt in range(10):
        site = web.TCPSite(runner, node.host, node.port)
        try:
            await site.start()
            break
        except OSError as e:
            await site.stop()  # a failed site stays registered in the runner until stopped
            if e.errno != errno.EADDRINUSE or attempt == 9:
                node.get_logger().error(f'Cannot listen on {node.host}:{node.port}: {e.strerror}')
                await runner.cleanup()
                return False
            if attempt == 0:
                node.get_logger().warn(f'Port {node.port} is in use, retrying...')
            await asyncio.sleep(1.0)

    hosts = local_ipv4_addresses() if node.host in ('', '0.0.0.0') else [node.host]
    for host in hosts:
        node.get_logger().info(f'Web UI on http://{host}:{node.port}')
    try:
        await stop.wait()
    finally:
        await runner.cleanup()
    return True


def _spin_forever(executor, node):
    """executor.spin(), but one failing callback must not stop every other callback.

    rclpy (Humble) re-raises a callback's exception out of spin_once(), which ends
    executor.spin() for good: the buttons, LCD/panel refresh and drive timers then all stop
    while the web server keeps running. The usual cause is harmless: a subscription destroyed
    (viewer left) while the executor was taking a message from it -> InvalidHandle.
    """
    from rclpy.exceptions import InvalidHandle
    while rclpy.ok() and not executor._is_shutdown:
        try:
            executor.spin_once(timeout_sec=0.5)
        except InvalidHandle:
            pass   # entity destroyed while it was ready; nothing to do
        except Exception as e:  # noqa: BLE001 - keep spinning, but say what broke
            try:
                node.get_logger().error(f'Callback failed: {type(e).__name__}: {e}')
            except Exception:  # noqa: BLE001
                pass


def main(args=None):
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:
        rclpy.init(args=args)

    node = QCar2WebNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    spin_thread = threading.Thread(target=_spin_forever, args=(executor, node), daemon=True)
    spin_thread.start()

    ok = True
    try:
        ok = asyncio.run(serve(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.drive.estop()
        node.nav.cancel()
        if node.lcd is not None:
            node.lcd.close()
        if rclpy.ok():
            for _ in range(3):
                node.publish_motor(0.0, 0.0)
                time.sleep(0.05)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
