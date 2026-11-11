"""Map view, Nav2 goals and the AMCL start pose for the web interface.

While the map is watched (or the car is navigating) this subscribes to the occupancy grid
(from cartographer in SLAM mode, or map_server in navigation mode), the lidar scan, the Nav2
global plan and TF, and draws them into a top view. A goal picked on that view is sent to
Nav2's navigate_to_pose action; a start pose picked on it goes to AMCL's /initialpose.

Nav2's controller publishes geometry_msgs/Twist on /cmd_vel_nav. Instead of Quanser's
nav2_qcar2_converter (which publishes motor commands at 1 kHz all the time and would fight
the web driving), the web node reads that topic in 'nav' mode and publishes it itself, so
the E-STOP, the browser watchdog and the brake assist also apply to Nav2.

Twist -> QCar2 command: angular.z is a yaw rate; with the bicycle model the steering angle
is atan(wheelbase * w / v). steer_mode 'direct' uses angular.z as the steering angle instead,
like nav2_qcar2_converter does.
"""

import math
import os
import re
import shutil
import threading
import time

import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import tf2_ros

from qcar2_web import tuning

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:   # Nav2 not installed: map view only
    NavigateToPose = None
try:
    from cartographer_ros_msgs.srv import WriteState
except ImportError:
    WriteState = None

OUT_PX = 720           # longest side of the rendered map
MARGIN_M = 0.6         # around the known area
MIN_VIEW_M = 3.0
CMD_TIMEOUT_S = 0.5    # /cmd_vel_nav older than this counts as zero
GOAL_STATUS = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}   # action_msgs/GoalStatus

# BGR
COL_UNKNOWN = (52, 46, 40)
COL_FREE = (205, 205, 200)
COL_OCC = (25, 25, 25)
COL_GRID = (80, 72, 64)
COL_PLAN = (60, 190, 60)
COL_SCAN = (60, 60, 230)
COL_CAR = (255, 140, 59)
COL_GOAL = (43, 169, 240)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _tf2d(t):
    tr = t.transform.translation
    return tr.x, tr.y, yaw_from_quat(t.transform.rotation)


def compute_view(grid, extra_pts):
    """World window to draw: the known part of the map plus the given (x, y) points.

    Returns dict(x0, y1, scale px/m, w, h): pixel (u, v) = ((x - x0) * scale, (y1 - y) * scale).
    """
    data, ox, oy, res = grid
    rows, cols = np.nonzero(data >= 0)
    if len(rows):
        xs = [ox + cols.min() * res, ox + (cols.max() + 1) * res]
        ys = [oy + rows.min() * res, oy + (rows.max() + 1) * res]
    else:
        xs, ys = [ox, ox + data.shape[1] * res], [oy, oy + data.shape[0] * res]
    for x, y in extra_pts:
        xs.append(x)
        ys.append(y)
    xmin, xmax = min(xs) - MARGIN_M, max(xs) + MARGIN_M
    ymin, ymax = min(ys) - MARGIN_M, max(ys) + MARGIN_M
    if xmax - xmin < MIN_VIEW_M:
        c = (xmin + xmax) / 2
        xmin, xmax = c - MIN_VIEW_M / 2, c + MIN_VIEW_M / 2
    if ymax - ymin < MIN_VIEW_M:
        c = (ymin + ymax) / 2
        ymin, ymax = c - MIN_VIEW_M / 2, c + MIN_VIEW_M / 2
    scale = OUT_PX / max(xmax - xmin, ymax - ymin)
    return {'x0': xmin, 'y1': ymax, 'scale': scale,
            'w': int(round((xmax - xmin) * scale)), 'h': int(round((ymax - ymin) * scale))}


def render_map(grid, view, pose=None, scan=None, plan=None, goal=None,
               car_length=0.425, car_width=0.192):
    """BGR top view. pose/goal: (x, y, yaw); scan: Nx2 array; plan: list of (x, y)."""
    data, ox, oy, res = grid
    w, h, s, x0, y1 = view['w'], view['h'], view['scale'], view['x0'], view['y1']
    cells = np.empty(data.shape + (3,), np.uint8)
    cells[:] = COL_UNKNOWN
    cells[(data >= 0) & (data < 50)] = COL_FREE
    cells[data >= 50] = COL_OCC
    # grid row 0 is at the bottom (y = oy); image row 0 is at the top (y = y1)
    m = np.float32([[res * s, 0, (ox - x0) * s],
                    [0, -res * s, (y1 - oy) * s]])
    img = cv2.warpAffine(cells, m, (w, h), flags=cv2.INTER_NEAREST,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=COL_UNKNOWN)

    def px(x, y):
        return int(round((x - x0) * s)), int(round((y1 - y) * s))

    # 1 m grid, faint so it does not look like walls
    lines = img.copy()
    for gx in range(math.ceil(x0), math.floor(x0 + w / s) + 1):
        cv2.line(lines, px(gx, y1), px(gx, y1 - h / s), COL_GRID, 1)
    for gy in range(math.ceil(y1 - h / s), math.floor(y1) + 1):
        cv2.line(lines, px(x0, gy), px(x0 + w / s, gy), COL_GRID, 1)
    img = cv2.addWeighted(lines, 0.3, img, 0.7, 0)

    if plan is not None and len(plan) > 1:
        pts = np.array([px(x, y) for x, y in plan], np.int32)
        cv2.polylines(img, [pts], False, COL_PLAN, 2, cv2.LINE_AA)
    if scan is not None and len(scan):
        u = ((scan[:, 0] - x0) * s).astype(np.int32)
        v = ((y1 - scan[:, 1]) * s).astype(np.int32)
        ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        for uu, vv in zip(u[ok], v[ok]):
            cv2.circle(img, (int(uu), int(vv)), 2, COL_SCAN, -1)
    if goal is not None:
        gx, gy, gyaw = goal
        cv2.circle(img, px(gx, gy), 9, COL_GOAL, 2, cv2.LINE_AA)
        cv2.arrowedLine(img, px(gx, gy), px(gx + 0.5 * math.cos(gyaw), gy + 0.5 * math.sin(gyaw)),
                        COL_GOAL, 2, cv2.LINE_AA, tipLength=0.3)
    if pose is not None:
        x, y, yaw = pose
        c, sn = math.cos(yaw), math.sin(yaw)
        # base_link is on the rear axle, so most of the car is in front of it
        corners = [(-0.08, -car_width / 2), (car_length - 0.08, -car_width / 2),
                   (car_length - 0.08, car_width / 2), (-0.08, car_width / 2)]
        poly = np.array([px(x + cx * c - cy * sn, y + cx * sn + cy * c) for cx, cy in corners], np.int32)
        cv2.fillPoly(img, [poly], COL_CAR, cv2.LINE_AA)
        cv2.arrowedLine(img, px(x, y), px(x + 0.45 * c, y + 0.45 * sn), (255, 255, 255), 2,
                        cv2.LINE_AA, tipLength=0.3)

    # scale bar: 1 m
    bar = int(round(s))
    cv2.line(img, (12, h - 14), (12 + bar, h - 14), (255, 255, 255), 2)
    cv2.putText(img, '1 m', (12, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                cv2.LINE_AA)
    return img


def clean_map_name(name):
    """A safe file name for a map (letters, digits, _ . -), or a time stamp if nothing is left."""
    return (re.sub(r'[^A-Za-z0-9_.-]+', '_', name or '').strip('._')
            or time.strftime('map_%Y%m%d_%H%M%S'))


def write_map_files(grid, directory, name):
    """Save the grid as <name>.pgm + <name>.yaml (map_server format). Returns the yaml path."""
    data, ox, oy, res = grid
    pgm = np.full(data.shape, 205, np.uint8)   # unknown
    pgm[(data >= 0) & (data < 25)] = 254       # free
    pgm[data >= 65] = 0                        # occupied
    pgm = np.flipud(pgm)                       # PGM row 0 is the top of the map
    os.makedirs(directory, exist_ok=True)
    pgm_path = os.path.join(directory, name + '.pgm')
    yaml_path = os.path.join(directory, name + '.yaml')
    with open(pgm_path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (pgm.shape[1], pgm.shape[0]))
        f.write(pgm.tobytes())
    with open(yaml_path, 'w') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {res}\n'
                f'origin: [{ox}, {oy}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return yaml_path


class NavBridge:
    """Map/scan/plan/TF subscriptions while wanted, the rendered view, and the Nav2 goal."""

    def __init__(self, node, map_topic, scan_topic, plan_topic, cmd_topic, map_dir, steer_mode):
        self.node = node
        self.topics = {'map': map_topic, 'scan': scan_topic, 'plan': plan_topic, 'cmd': cmd_topic}
        self.map_dir = map_dir
        self.steer_mode = steer_mode
        self._lock = threading.Lock()
        self._subs = None
        self._tf_buffer = None
        self._tf_listener = None
        self._grid = None      # (data int8 HxW, origin x, origin y, resolution)
        self._grid_time = 0.0
        self.grid_version = 0  # +1 whenever the grid content changes (for the browser map)
        self._scan = None
        self._plan = None
        self._cmd = None       # (v, w, monotonic time)
        self.pose = None
        self.view = None       # view of the newest rendered frame
        self.goal = None       # (x, y, yaw) of the current/last goal
        self.state = 'idle'
        self.feedback = None
        self._goal_handle = None
        self._goal_seq = 0
        self.on_goal_done = None   # called with the final state when a goal ends
        self.keep_until = 0.0      # monotonic: stay subscribed until then (saving a map)
        self._action = (ActionClient(node, NavigateToPose, 'navigate_to_pose')
                        if NavigateToPose is not None else None)
        self._write_state = (node.create_client(WriteState, 'write_state')
                             if WriteState is not None else None)
        # RTAB-Map (V-SLAM): saves its database to <database_path>.back, then goes on mapping
        self._backup = node.create_client(Empty, 'backup')
        self.map_stats = None      # extent and area of the known part of the map
        # AMCL and RTAB-Map both listen to /initialpose
        self._initial_pose_pub = node.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 1)

    # ------------------------------------------------------------ subscriptions
    def set_wanted(self, wanted):
        if wanted and self._subs is None:
            n = self.node
            reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            # map_server publishes the map once, transient local; cartographer keeps
            # publishing it volatile. A subscription only matches one of the two, so use both.
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, n)
            self._subs = [
                n.create_subscription(OccupancyGrid, self.topics['map'], self._on_map, reliable),
                n.create_subscription(OccupancyGrid, self.topics['map'], self._on_map, latched),
                n.create_subscription(LaserScan, self.topics['scan'], self._on_scan,
                                      qos_profile_sensor_data),
                n.create_subscription(Path, self.topics['plan'], self._on_plan, 1),
                n.create_subscription(Twist, self.topics['cmd'], self._on_cmd, 1),
            ]
            n.get_logger().info('Nav: subscribed to map, scan, plan, TF and Nav2 commands')
        elif not wanted and self._subs is not None:
            for sub in self._subs:
                self.node.destroy_subscription(sub)
            self._subs = None
            self._tf_listener.unregister()
            self._tf_listener = self._tf_buffer = None
            with self._lock:
                self._scan = self._plan = self._cmd = None
            self.pose = None
            self.node.get_logger().info('Nav: not needed, unsubscribed')

    def forget_map(self):
        """Drop the shown map (SLAM/navigation switched). A new map_server's latched map still
        arrives: the transient local subscription gets it when it matches the new publisher."""
        with self._lock:
            self._grid = None
            self.map_stats = None
            self.grid_version += 1

    def _on_map(self, msg):
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        o = msg.info.origin.position
        known = data >= 0
        rows, cols = np.flatnonzero(known.any(axis=1)), np.flatnonzero(known.any(axis=0))
        stats = None
        if len(rows) and len(cols):
            res = msg.info.resolution
            stats = {'width_m': round(float((cols[-1] - cols[0] + 1) * res), 2),
                     'height_m': round(float((rows[-1] - rows[0] + 1) * res), 2),
                     'area_m2': round(float(np.count_nonzero(data == 0) * res * res), 1),
                     'resolution': round(float(res), 3)}
        with self._lock:
            old = self._grid
            changed = (old is None or old[1:] != (o.x, o.y, msg.info.resolution)
                       or old[0].shape != data.shape or not np.array_equal(old[0], data))
            self._grid = (data, o.x, o.y, msg.info.resolution)
            self._grid_time = time.monotonic()
            self.map_stats = stats
            if changed:
                self.grid_version += 1

    def _on_scan(self, msg):
        with self._lock:
            self._scan = msg

    def _on_plan(self, msg):
        with self._lock:
            self._plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _on_cmd(self, msg):
        with self._lock:
            self._cmd = (msg.linear.x, msg.angular.z, time.monotonic())

    def _lookup(self, child):
        buf = self._tf_buffer
        if buf is None:
            return None
        try:
            return _tf2d(buf.lookup_transform('map', child, Time()))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def _scan_points(self, scan_msg):
        """Lidar returns in the map frame (Nx2), or None."""
        if scan_msg is None:
            return None
        st = self._lookup(scan_msg.header.frame_id or 'base_scan')
        if st is None:
            return None
        r = np.asarray(scan_msg.ranges, np.float32)
        a = scan_msg.angle_min + np.arange(len(r), dtype=np.float32) * scan_msg.angle_increment
        ok = np.isfinite(r) & (r > scan_msg.range_min) & (r < scan_msg.range_max)
        r, a = r[ok], a[ok] + st[2]
        return np.stack([st[0] + r * np.cos(a), st[1] + r * np.sin(a)], axis=1)

    def grid_png(self):
        """(version, header dict, PNG bytes) of the grid for the browser map, or (version, None, None).

        One grey channel: 255 = unknown, else the occupancy 0..100. PNG row 0 = grid row 0 (y = oy).
        """
        with self._lock:
            grid, ver = self._grid, self.grid_version
        if grid is None:
            return ver, None, None
        data, ox, oy, res = grid
        img = np.where(data < 0, 255, np.clip(data, 0, 100)).astype(np.uint8)
        ok, png = cv2.imencode('.png', img)
        head = {'w': int(data.shape[1]), 'h': int(data.shape[0]), 'ox': float(ox), 'oy': float(oy),
                'res': float(res)}
        return ver, head, (png.tobytes() if ok else None)

    def snapshot(self, max_scan=720):
        """Everything the browser map draws besides the grid: car pose, scan, plan, goal (metres, map frame)."""
        with self._lock:
            scan_msg, plan = self._scan, self._plan
        scan = self._scan_points(scan_msg)
        pts = []
        if scan is not None and len(scan):
            step = max(1, len(scan) // max_scan)
            pts = np.round(scan[::step], 3).ravel().tolist()
        path = []
        if plan:
            step = max(1, len(plan) // 400)
            path = [round(v, 3) for p in plan[::step] + [plan[-1]] for v in p]
        active = self.state in ('sending', 'active')
        return {'pose': list(self.pose) if self.pose else None, 'scan': pts, 'plan': path,
                'goal': list(self.goal) if (self.goal and active) else None, 'state': self.state}

    # ------------------------------------------------------------ drawing
    def update(self, render, car_length, car_width):
        """Refresh the pose; with render, return a new frame (BGR) or None if there is no map."""
        self.pose = self._lookup('base_link')
        if not render:
            return None
        with self._lock:
            grid, scan_msg, plan = self._grid, self._scan, self._plan
        if grid is None:
            return None
        scan = self._scan_points(scan_msg)
        extra = [self.pose[:2]] if self.pose else []
        if self.goal and self.state in ('sending', 'active'):
            extra.append(self.goal[:2])
        view = compute_view(grid, extra)
        img = render_map(grid, view, self.pose, scan, plan,
                         self.goal if self.state in ('sending', 'active') else None,
                         car_length, car_width)
        self.view = view
        return img

    # ------------------------------------------------------------ driving
    def command(self, wheelbase, max_steer, max_speed):
        """(steer_rad, speed_mps) from Nav2's newest Twist; zeros if it is missing or stale."""
        with self._lock:
            cmd = self._cmd
        if cmd is None or time.monotonic() - cmd[2] > CMD_TIMEOUT_S:
            return 0.0, 0.0
        v, w, _ = cmd
        speed = max(-max_speed, min(max_speed, v))
        if self.steer_mode == 'direct':
            steer = w
        elif abs(v) > 1e-3:
            steer = math.atan(wheelbase * w / v)
        else:
            steer = math.copysign(max_steer, w) if abs(w) > 0.05 else 0.0
        return max(-max_steer, min(max_steer, steer)), speed

    # ------------------------------------------------------------ goals
    def view_to_world(self, u, v):
        view = self.view
        if view is None:
            return None
        return view['x0'] + u * view['w'] / view['scale'], view['y1'] - v * view['h'] / view['scale']

    def goal_from_view(self, u0, v0, u1, v1):
        """Goal at (u0, v0) of the shown frame (0..1), facing (u1, v1); None without a map."""
        p0 = self.view_to_world(u0, v0)
        p1 = self.view_to_world(u1, v1)
        if p0 is None:
            return None
        return self.goal_from_world(p0, p1)

    def goal_from_world(self, p0, p1):
        """Goal at p0 (map metres) facing p1; a click without a drag faces away from the car."""
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        if math.hypot(dx, dy) < 0.15:   # a click, not a drag: face away from the car
            if self.pose is None:
                return p0[0], p0[1], 0.0
            dx, dy = p0[0] - self.pose[0], p0[1] - self.pose[1]
        return p0[0], p0[1], math.atan2(dy, dx)

    def set_initial_pose(self, x, y, yaw):
        """Tell AMCL where the car is (as RViz's "2D Pose Estimate" does)."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25 ** 2
        msg.pose.covariance[35] = math.radians(15) ** 2
        self._initial_pose_pub.publish(msg)
        self.node.get_logger().info(
            f'Nav: start pose x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}°')

    def send_goal(self, x, y, yaw):
        """Returns an error message, or None once the goal is on its way."""
        if self._action is None:
            return 'ไม่พบ nav2_msgs (ติดตั้ง Nav2 หรือยัง?)'
        if not self._action.server_is_ready():
            return 'ไม่พบ Nav2 (navigate_to_pose) — รัน launch ที่มี Nav2 หรือยัง?'
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        with self._lock:
            self._goal_seq += 1
            seq = self._goal_seq
            old, self._goal_handle = self._goal_handle, None
            self.goal, self.state, self.feedback = (x, y, yaw), 'sending', None
        if old is not None:
            old.cancel_goal_async()   # Nav2 would preempt it anyway; this keeps its result quiet
        future = self._action.send_goal_async(msg, feedback_callback=self._on_feedback)
        future.add_done_callback(lambda f: self._on_goal_response(f, seq))
        self.node.get_logger().info(f'Nav: goal x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}°')
        return None

    def _on_goal_response(self, future, seq):
        handle = future.result()
        with self._lock:
            if seq != self._goal_seq:   # replaced or cancelled meanwhile
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.state = 'rejected'
            else:
                self._goal_handle = handle
                self.state = 'active'
        if handle.accepted:
            handle.get_result_async().add_done_callback(lambda f: self._on_result(f, seq))
        elif self.on_goal_done:
            self.on_goal_done('rejected')

    def _on_feedback(self, msg):
        fb = msg.feedback
        self.feedback = {'remaining': float(fb.distance_remaining),
                         'recoveries': int(fb.number_of_recoveries)}

    def _on_result(self, future, seq):
        state = GOAL_STATUS.get(future.result().status, 'aborted')
        with self._lock:
            if seq != self._goal_seq:
                return
            self._goal_handle = None
            self.state = state
        self.node.get_logger().info(f'Nav: goal {state}')
        if self.on_goal_done:
            self.on_goal_done(state)

    def cancel(self):
        with self._lock:
            if self.state not in ('sending', 'active'):
                return
            self._goal_seq += 1
            handle, self._goal_handle = self._goal_handle, None
            self.state = 'canceled'
        if handle is not None:
            handle.cancel_goal_async()
        self.node.get_logger().info('Nav: goal canceled')

    # ------------------------------------------------------------ saving
    def save_map(self, name, method='lidar'):
        """Write <map_dir>/<name>.pgm/.yaml, plus .pbstream (lidar, if cartographer answers) or
        .db (V-SLAM: RTAB-Map's database). Blocking. Returns (files, warning or None)."""
        name = clean_map_name(name)
        # The map subscription normally runs only while the map is watched: keep it (the
        # node's nav timer subscribes) and wait for a map.
        deadline = time.monotonic() + 8.0
        self.keep_until = deadline + 2.0
        while True:
            with self._lock:
                grid = self._grid
            if grid is not None or time.monotonic() > deadline:
                break
            time.sleep(0.2)
        if grid is None:
            raise ValueError('ยังไม่มีแผนที่ (เริ่มทำแผนที่หรือยัง?)')
        directory = os.path.expanduser(self.map_dir)
        files = [write_map_files(grid, directory, name)]
        warning = None
        if method == 'vslam':
            try:
                files.append(self._save_vslam_db(directory, name))
            except (ValueError, OSError) as e:
                warning = (f'บันทึกแผนที่กริดแล้ว แต่เก็บข้อมูลกล้อง (.db) ไม่ได้: {e} '
                           '— แผนที่นี้นำทางด้วย Lidar ได้เท่านั้น')
        else:
            client = self._write_state
            if client is not None and client.service_is_ready():
                req = WriteState.Request()
                req.filename = os.path.join(directory, name + '.pbstream')
                req.include_unfinished_submaps = True
                done = threading.Event()
                future = client.call_async(req)
                future.add_done_callback(lambda f: done.set())
                if done.wait(10.0) and future.result() is not None and future.result().status.code == 0:
                    files.append(req.filename)
        return files, warning

    def _save_vslam_db(self, directory, name):
        """Ask RTAB-Map to write its database, and copy it next to the grid."""
        if not self._backup.wait_for_service(timeout_sec=2.0):
            raise ValueError('ไม่พบ RTAB-Map (กำลังทำแผนที่ด้วยกล้องอยู่ไหม?)')
        src = os.path.expanduser(tuning.VSLAM_DB_PATH) + '.back'
        before = os.path.getmtime(src) if os.path.exists(src) else 0.0
        done = threading.Event()
        future = self._backup.call_async(Empty.Request())
        future.add_done_callback(lambda f: done.set())
        # It closes the database, copies it and loads it again: seconds for a large map.
        if not done.wait(120.0):
            raise ValueError('RTAB-Map ใช้เวลานานเกินไป')
        if not os.path.exists(src) or os.path.getmtime(src) <= before:
            raise ValueError('RTAB-Map ไม่ได้สร้างไฟล์ฐานข้อมูล')
        dst = os.path.join(directory, name + '.db')
        shutil.copyfile(src, dst)
        return dst

    def status(self):
        with self._lock:
            have_map = self._grid is not None
            map_age = time.monotonic() - self._grid_time if have_map else None
            cmd = self._cmd
        if cmd is not None and time.monotonic() - cmd[2] > CMD_TIMEOUT_S:
            cmd = None
        pose = self.pose
        return {
            'state': self.state,
            'server': self._action is not None and self._action.server_is_ready(),
            'map': have_map,
            'map_age': map_age,
            'map_stats': self.map_stats,
            'pose': {'x': pose[0], 'y': pose[1], 'yaw': pose[2]} if pose else None,
            'goal': ({'x': self.goal[0], 'y': self.goal[1], 'yaw': self.goal[2]}
                     if self.goal else None),
            'feedback': self.feedback if self.state == 'active' else None,
            'cmd': {'v': cmd[0], 'w': cmd[1]} if cmd else None,
        }
