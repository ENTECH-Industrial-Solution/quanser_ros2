"""Example robot web node built on robot_web_ui: the starting point for a new robot.

    ros2 run robot_web_ui example_server --ros-args \\
        -p cameras:="['front=/camera/image_raw', 'depth=/camera/depth/image_raw:depth']" \\
        -p cmd_vel_topic:=/cmd_vel -p port:=8080

Serves robot_web_ui's template page (template/index.html), streams the camera topics, and lets
one browser at a time drive the robot by publishing geometry_msgs/Twist on cmd_vel_topic.

Safety, as on the QCar2: while a browser holds control a command is published at a fixed rate;
if the browser stops sending (closed tab, Wi-Fi drop) for longer than command_timeout seconds,
zero velocity is published instead. E-STOP stops and takes control away from everyone until it
is reset. Nobody in control -> nothing is published (other nodes may drive).

To make a robot's own web package: copy this file and template/ into it, rename, then add the
robot's pages (HTML), its windows (CSS) and its APIs (routes in make_app). See README.md.
"""

import asyncio
import itertools
import json
import math
import os
import threading
import time

import rclpy
from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_web_ui.server import CameraStream, add_ui_routes, auth_middleware, serve, spin_forever

PUBLISH_RATE_HZ = 20.0
STOP_REPEAT_S = 1.0   # keep sending zeros this long after control is released


def _unit(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0.0
    return max(-1.0, min(1.0, value)) if math.isfinite(value) else 0.0


class Drive:
    """Who may drive, the latest command (linear, angular in -1..1), watchdog and E-STOP."""

    def __init__(self, command_timeout):
        self.command_timeout = command_timeout
        self._lock = threading.Lock()
        self._owner = None
        self._cmd = (0.0, 0.0)
        self._last = 0.0
        self._estop = False
        self._stop_until = 0.0

    def take(self, client):
        with self._lock:
            if self._owner is None and not self._estop:
                self._owner, self._cmd, self._last = client, (0.0, 0.0), 0.0
            return self._owner == client

    def release(self, client):
        with self._lock:
            if self._owner == client:
                self._owner, self._cmd = None, (0.0, 0.0)
                self._stop_until = time.monotonic() + STOP_REPEAT_S

    def command(self, client, linear, angular):
        with self._lock:
            if self._owner == client and not self._estop:
                self._cmd, self._last = (_unit(linear), _unit(angular)), time.monotonic()

    def estop(self):
        with self._lock:
            self._estop, self._owner, self._cmd = True, None, (0.0, 0.0)

    def reset_estop(self):
        with self._lock:
            self._estop = False
            self._stop_until = time.monotonic() + STOP_REPEAT_S

    def output(self):
        """(linear, angular) to publish now, or None to stay silent."""
        now = time.monotonic()
        with self._lock:
            if self._estop:
                return 0.0, 0.0
            if self._owner is None:
                return (0.0, 0.0) if now < self._stop_until else None
            self._stop_until = now + STOP_REPEAT_S
            return self._cmd if now - self._last <= self.command_timeout else (0.0, 0.0)

    def state_for(self, client):
        now = time.monotonic()
        with self._lock:
            control = 'none' if self._owner is None else 'me' if self._owner == client else 'other'
            return {'control': control, 'estop': self._estop,
                    'stale': self._owner is not None and now - self._last > self.command_timeout}


class ExampleWebNode(Node):
    def __init__(self):
        super().__init__('robot_web')
        p = self.declare_parameter
        p('robot_name', 'Robot')
        p('host', '0.0.0.0')
        p('port', 8080)
        p('password', '')
        p('static_dir', '')   # '' = robot_web_ui's template page
        # [''] not []: Humble takes an empty list for a byte array
        p('cameras', [''], ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY,
                                               description="'name=topic' or 'name=topic:depth'"))
        p('cmd_vel_topic', '/cmd_vel')
        p('max_linear', 0.5)    # m/s at full joystick
        p('max_angular', 1.0)   # rad/s at full joystick
        p('command_timeout', 0.5)
        p('use_compressed', False)   # True: subscribe to <topic>/compressed (image_transport)
        p('recompress', False)
        p('max_fps', 15.0)
        p('jpeg_quality', 70)
        p('stream_max_width', 960)
        p('depth_max_mm', 4000.0)
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.robot_name = g('robot_name')
        self.host, self.port, self.password = g('host'), g('port'), g('password')
        self.static_dir = g('static_dir') or os.path.join(
            get_package_share_directory('robot_web_ui'), 'template')
        self.max_linear, self.max_angular = g('max_linear'), g('max_angular')
        self.use_compressed, self.recompress = g('use_compressed'), g('recompress')
        self.max_fps, self.jpeg_quality = g('max_fps'), g('jpeg_quality')
        self.stream_max_width, self.depth_max_mm = g('stream_max_width'), g('depth_max_mm')

        self.camera_group = ReentrantCallbackGroup()
        self.cameras = {}
        for entry in g('cameras'):
            name, _, topic = entry.partition('=')
            topic, _, kind = topic.partition(':')
            if name and topic:
                self.cameras[name] = CameraStream(self, name, name, topic, kind or 'color')

        self.drive = Drive(g('command_timeout'))
        self.last_output = None
        self.cmd_pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)

    def _publish(self):
        out = self.drive.output()
        self.last_output = None if out is None else {
            'linear': out[0] * self.max_linear, 'angular': out[1] * self.max_angular}
        if out is not None:
            self.publish_twist(**self.last_output)

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.cmd_pub.publish(msg)

    def status_for(self, client):
        status = self.drive.state_for(client)
        status.update({'type': 'status', 'output': self.last_output})
        return status


async def info_handler(request):
    node = request.app['node']
    return web.json_response({
        'name': node.robot_name,
        'cameras': [cam.info() for cam in node.cameras.values()],
        'limits': {'max_linear': node.max_linear, 'max_angular': node.max_angular,
                   'command_timeout': node.drive.command_timeout},
    })


async def _push_status(ws, node, client):
    try:
        while not ws.closed:
            await ws.send_str(json.dumps(node.status_for(client)))
            await asyncio.sleep(0.2)
    except (ConnectionResetError, ConnectionError, RuntimeError):
        pass


async def ws_handler(request):
    """Status out (5 Hz); in: take, release, drive {linear, angular}, estop, estop_reset."""
    node = request.app['node']
    ws = web.WebSocketResponse(heartbeat=5.0)
    await ws.prepare(request)
    client = next(request.app['client_ids'])
    pusher = asyncio.ensure_future(_push_status(ws, node, client))
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
            except ValueError:
                continue
            kind = data.get('type') if isinstance(data, dict) else None
            if kind == 'drive':
                node.drive.command(client, data.get('linear'), data.get('angular'))
            elif kind == 'take':
                if node.drive.take(client):
                    node.get_logger().info(f'Client {client} took control')
            elif kind == 'release':
                node.drive.release(client)
            elif kind == 'estop':
                node.drive.estop()
                node.get_logger().warn(f'E-STOP pressed by client {client}')
            elif kind == 'estop_reset':
                node.drive.reset_estop()
    finally:
        pusher.cancel()
        node.drive.release(client)
    return ws


def make_app(node):
    app = web.Application(middlewares=[auth_middleware])
    app['node'] = node
    app['static_dir'] = node.static_dir
    app['realm'] = node.robot_name
    app['client_ids'] = itertools.count(1)
    app.router.add_get('/api/info', info_handler)
    app.router.add_get('/ws', ws_handler)
    add_ui_routes(app)
    return app


def main(args=None):
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:
        rclpy.init(args=args)
    node = ExampleWebNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=spin_forever, args=(executor, node), daemon=True).start()
    ok = True
    try:
        ok = asyncio.run(serve(node, make_app(node)))
    except KeyboardInterrupt:
        pass
    finally:
        node.drive.estop()
        if rclpy.ok():
            for _ in range(3):
                node.publish_twist(0.0, 0.0)
                time.sleep(0.05)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
