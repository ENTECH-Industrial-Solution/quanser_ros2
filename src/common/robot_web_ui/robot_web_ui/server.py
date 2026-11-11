"""robot_web_ui server side: what any robot's web node needs, independent of the robot.

- Camera topics as JPEG frames: CameraStream (subscribes only while someone is watching),
  sent to the page over /ws/stream/<name> (ws_stream_handler, with flow control) or as MJPEG
  over /stream/<name> (stream_handler).
- The page and the shared UI files: index_handler, static_script_handler, add_ui_routes
  (/ui/ = robot_web_ui's static files: theme.css, ui.css, core.js, i18n.js, ui_en.js).
- Optional HTTP Basic password (auth_middleware), listening with retry (serve), and an
  executor loop that survives a failing callback (spin_forever).

The robot's node must provide what these use:
  node.cameras          {name: CameraStream}
  node.camera_group     callback group for the camera subscriptions
  node.use_compressed   subscribe to <topic>/compressed for 'color' cameras
  node.recompress       decode + re-encode compressed JPEG instead of passing it on
  node.max_fps, node.jpeg_quality, node.stream_max_width (0 = keep), node.depth_max_mm
  node.password         '' = no password
  node.host, node.port
and the aiohttp app: app['node'], app['static_dir'] (the robot's static files with
index.html), optionally app['realm'] (name shown in the password prompt).
See example_server.py for a complete, small robot node.
"""

import asyncio
import base64
import errno
import fcntl
import hmac
import os
import signal
import socket
import struct
import threading
import time

import cv2
import numpy as np
import rclpy
from aiohttp import web
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


# WebSocket camera streams: at most this many frames sent but not yet acknowledged by the
# page. Without a limit, frames pile up in the TCP send buffer on a slow Wi-Fi link and the
# view lags seconds behind; with it, a slow link gets a lower frame rate instead of delay.
STREAM_WINDOW = 2
STREAM_ACK_TIMEOUT_S = 1.0  # assume the acks were lost (or an older page) and send anyway


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
            realm = request.app.get('realm', 'Robot')
            raise web.HTTPUnauthorized(headers={'WWW-Authenticate': f'Basic realm="{realm}"'})
    return await handler(request)


async def index_handler(request):
    return web.FileResponse(os.path.join(request.app['static_dir'], 'index.html'),
                            headers={'Cache-Control': 'no-cache'})


async def static_script_handler(request):
    """The page's own scripts: like index.html, the browser must check for a newer copy every
    time, or after an update it keeps running the old code behind the new page."""
    name = request.match_info['name']
    path = os.path.join(request.app['static_dir'], name)
    if not os.path.isfile(path):
        raise web.HTTPNotFound()
    return web.FileResponse(path, headers={'Cache-Control': 'no-cache'})


def ui_static_dir():
    """robot_web_ui's own static files (served at /ui/)."""
    return os.path.join(get_package_share_directory('robot_web_ui'), 'static')


async def ui_file_handler(request):
    """/ui/<file>: like the robot's scripts, checked for a newer copy on every load."""
    name = request.match_info['name']
    path = os.path.join(ui_static_dir(), name)
    if not os.path.isfile(path):
        raise web.HTTPNotFound()
    return web.FileResponse(path, headers={'Cache-Control': 'no-cache'})


def add_ui_routes(app):
    """The page, the shared UI files, the robot's static files and the camera streams.

    Add the robot's own routes before or after; app['static_dir'] must be set.
    """
    app.router.add_get('/', index_handler)
    app.router.add_get(r'/ui/{name:[A-Za-z0-9_.-]+\.(?:js|css)}', ui_file_handler)
    app.router.add_get('/stream/{name}', stream_handler)
    app.router.add_get('/ws/stream/{name}', ws_stream_handler)
    app.router.add_get(r'/static/{name:[a-z0-9_]+\.js}', static_script_handler)
    app.router.add_static('/static/', app['static_dir'])


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


async def serve(node, app):
    """Run the aiohttp app on node.host:node.port until SIGINT/SIGTERM; False if it cannot listen."""
    loop = asyncio.get_running_loop()
    for cam in node.cameras.values():
        cam.loop = loop

    stop = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    runner = web.AppRunner(app)
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


def spin_forever(executor, node):
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
