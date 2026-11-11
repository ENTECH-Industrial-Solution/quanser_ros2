"""RTAB-Map's whole 3D map for the web page's 3D view (like rtabmap_rviz_plugins' MapCloud).

RTAB-Map publishes mapData: the optimised pose of every keyframe (node) and, for new nodes, the
colour + depth images. Each node's images become a small coloured point cloud in that node's own
frame (base_link at the time), built once and kept. The page gets every node cloud once and the
newest poses whenever they change, and places the clouds itself, so a loop closure straightens
the whole 3D map like it does the 2D one. On the first viewer the full map is asked for with
the publish_map service (also for a map loaded for navigation).

Subscribes only while someone watches; costs nothing otherwise.

Node message (binary, little endian):
    uint32 kind = 1, int32 node id, uint32 count
    int16  x, y, z per point (millimetres, node frame: x forward, y left, z up)
    uint8  r, g, b per point
"""

import struct
import threading
import time

import cv2
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy

try:
    from rtabmap_msgs.msg import MapData
    from rtabmap_msgs.srv import PublishMap
except ImportError:   # RTAB-Map not installed: no 3D map
    MapData = PublishMap = None

STRIDE = 3            # every 3rd depth pixel
MIN_RANGE_M = 0.2
MAX_RANGE_M = 3.5     # the depth gets noisy further away
MIN_Z, MAX_Z = -0.2, 2.0
VOXEL_M = 0.04
NODE_HEADER = struct.Struct('<IiI')


def _quat_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def node_cloud(nd):
    """(points Nx3 float32 in the node frame, colours Nx3 uint8 RGB) from a NodeData, or None."""
    if not len(nd.image) or not len(nd.depth) or not len(nd.fx) or not len(nd.local_transform):
        return None
    bgr = cv2.imdecode(np.frombuffer(nd.image, np.uint8), cv2.IMREAD_COLOR)
    dep = cv2.imdecode(np.frombuffer(nd.depth, np.uint8), cv2.IMREAD_UNCHANGED)
    if bgr is None or dep is None:
        return None
    if dep.dtype == np.uint8 and dep.ndim == 3 and dep.shape[2] == 4:   # float depth stored as RGBA
        depth_m = np.ascontiguousarray(dep).view(np.float32)[..., 0]
    else:
        depth_m = dep.astype(np.float32) * 0.001
    # Intrinsics are for the colour image; the depth image may be smaller.
    k = depth_m.shape[1] / float(nd.width[0] if len(nd.width) and nd.width[0] > 0 else bgr.shape[1])
    fx, fy, cx, cy = nd.fx[0] * k, nd.fy[0] * k, nd.cx[0] * k, nd.cy[0] * k
    d = depth_m[::STRIDE, ::STRIDE]
    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]].astype(np.float32) * STRIDE
    ok = np.isfinite(d) & (d > MIN_RANGE_M) & (d < MAX_RANGE_M)
    z = d[ok]
    pts = np.stack([(u[ok] - cx) * z / fx, (v[ok] - cy) * z / fy, z], axis=1)
    # colour pixel of each depth pixel
    sx, sy = bgr.shape[1] / depth_m.shape[1], bgr.shape[0] / depth_m.shape[0]
    cu = np.clip((u[ok] * sx).astype(np.int32), 0, bgr.shape[1] - 1)
    cv_ = np.clip((v[ok] * sy).astype(np.int32), 0, bgr.shape[0] - 1)
    col = bgr[cv_, cu][:, ::-1]
    t = nd.local_transform[0]
    rot = _quat_matrix(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    pts = pts @ rot.T + np.array([t.translation.x, t.translation.y, t.translation.z])
    keep = (pts[:, 2] > MIN_Z) & (pts[:, 2] < MAX_Z)
    pts, col = pts[keep], col[keep]
    if not len(pts):
        return None
    # one point per 4 cm voxel
    _, first = np.unique(np.floor(pts / VOXEL_M).astype(np.int32), axis=0, return_index=True)
    return pts[first].astype(np.float32), np.ascontiguousarray(col[first], dtype=np.uint8)


def pack_node(node_id, pts, col):
    mm = np.clip(np.rint(pts * 1000.0), -32767, 32767).astype('<i2')
    return NODE_HEADER.pack(1, node_id, len(mm)) + mm.tobytes() + col.tobytes()


class MapCloud:

    def __init__(self, node):
        self.node = node
        self._lock = threading.Lock()
        self._viewers = 0
        self._sub = None
        self._group = MutuallyExclusiveCallbackGroup()   # decoding takes a while: keep it apart
        self.nodes = {}          # id -> packed message
        self.order = []          # ids in arrival order (clients send the new ones)
        self.poses = {}          # id -> [x, y, z, qx, qy, qz, qw]
        self.pose_version = 0
        self.session = 0         # +1 whenever the kept clouds are dropped (pages start over)
        self.last_data = 0.0     # when mapData last arrived
        self._requested = False  # full map asked for since the first viewer
        self._max_id = 0
        self._client = node.create_client(PublishMap, 'publish_map') if PublishMap else None
        node.create_timer(2.0, self._tick, callback_group=MutuallyExclusiveCallbackGroup())

    @property
    def available(self):
        return MapData is not None

    @property
    def receiving(self):
        return time.monotonic() - self.last_data < 5.0

    def add_viewer(self):
        with self._lock:
            self._viewers += 1
            if self._sub is not None or MapData is None:
                return
            qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
            self._sub = self.node.create_subscription(MapData, 'mapData', self._on_map_data, qos,
                                                      callback_group=self._group)
            self._requested = False

    def remove_viewer(self):
        with self._lock:
            self._viewers = max(0, self._viewers - 1)
            if self._viewers or self._sub is None:
                return
            self.node.destroy_subscription(self._sub)
            self._sub = None
            self.nodes, self.order, self.poses = {}, [], {}
            self.pose_version += 1
            self.session += 1
            self._max_id = 0

    def snapshot(self):
        with self._lock:
            return self.session, list(self.order), self.nodes, self.poses, self.pose_version

    # ------------------------------------------------------------ ROS
    def _tick(self):
        """Ask RTAB-Map for the whole map once it is there (first viewer, or RTAB-Map restarted)."""
        if self._sub is None or self._requested or self._client is None:
            return
        if not self._client.service_is_ready():
            return
        req = PublishMap.Request()
        req.global_map, req.optimized, req.graph_only = True, True, False
        self._client.call_async(req)
        self._requested = True
        self.node.get_logger().info('3D map: asked RTAB-Map for the whole map')

    def _on_map_data(self, msg):
        self.last_data = time.monotonic()
        ids = list(msg.graph.poses_id)
        newest = max(ids) if ids else 0
        with self._lock:
            # A new RTAB-Map session (ids start again, small graph): start over. In localisation
            # RTAB-Map may send only part of the graph, so a smaller message alone is not enough.
            if newest and newest + 5 < self._max_id and len(ids) * 2 < len(self.poses):
                self.nodes, self.order, self.poses = {}, [], {}
                self.session += 1
                self._requested = False
                self._max_id = newest
            else:
                self._max_id = max(self._max_id, newest)
        poses = {}
        for i, p in zip(ids, msg.graph.poses):
            poses[int(i)] = [round(v, 4) for v in (p.position.x, p.position.y, p.position.z,
                                                    p.orientation.x, p.orientation.y,
                                                    p.orientation.z, p.orientation.w)]
        built = []
        for nd in msg.nodes:
            if nd.id in self.nodes:
                continue
            try:
                cloud = node_cloud(nd)
            except Exception as e:   # never let a display feature disturb the robot
                self.node.get_logger().warn(f'3D map: node {nd.id}: {e}', throttle_duration_sec=10.0)
                continue
            if cloud is not None:
                built.append((nd.id, pack_node(nd.id, *cloud)))
        with self._lock:
            for i, data in built:
                if i not in self.nodes:
                    self.order.append(i)
                self.nodes[i] = data
            # merge: a partial graph only moves the nodes it contains
            merged = dict(self.poses)
            merged.update(poses)
            if merged != self.poses:
                self.poses = merged
                self.pose_version += 1
