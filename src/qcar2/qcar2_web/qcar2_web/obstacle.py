"""Obstacle detection from the RealSense depth image: brake assist and auto-avoid.

Every depth pixel (subsampled by `stride`) becomes a 3D point in the vehicle frame used by
surround.py (origin on the ground under the middle of the car, x forward, y left, z up).
Points between min_height and max_height above the ground are obstacles; the floor and
anything above the car are ignored.

For a steering angle, the free distance is how far the front bumper can travel along that
arc (bicycle model around the rear axle) before the car, widened by `margin` on each side,
reaches an obstacle point. The brake assist scales forward speed by the free distance on the
commanded arc; auto mode picks the steering angle with the most free distance.

The D435 publishes 0 where it has no depth: closer than ~0.3 m, glass, black or shiny
surfaces. When most of the area straight ahead is 0 the view is "blind" and treated as
blocked, because something may be right in front of the camera.

No camera_info is published, so the intrinsics come from the field of view (square pixels,
principal point in the middle).
"""

import copy
import json
import math
import threading
import time

import cv2
import numpy as np

DEFAULT_CONFIG = {
    'enabled': True,          # brake assist while driving manually
    'depth_scale': 0.001,     # metres per depth unit (Quanser video3d: millimetres)
    'hfov_deg': 0.0,          # 0 = from the image size (D435: 87 deg at 16:9, 80 deg at 4:3)
    'stride': 4,              # use every n-th pixel in both directions
    'max_range': 4.0,         # ignore depth beyond this (m)
    # RealSense mounting, vehicle frame (m / deg); positive pitch tilts the camera down.
    'cam_x': 0.20, 'cam_y': 0.0, 'cam_z': 0.20, 'cam_pitch': 0.0, 'cam_roll': 0.0,
    'min_height': 0.05,       # points lower than this are floor
    'max_height': 0.40,       # points higher than this are above the car
    'car_length': 0.425, 'car_width': 0.192, 'wheelbase': 0.256,
    'margin': 0.05,           # extra clearance on each side (m)
    'stop_dist': 0.30,        # free distance at which forward speed is 0
    'slow_dist': 1.00,        # free distance below which forward speed is reduced
    'min_points': 6,          # fewer obstacle points than this on a path = noise
    'blind_fraction': 0.6,    # share of 0 pixels straight ahead that counts as blind
    'auto_speed': 0.4,        # auto mode speed, fraction of max_speed
    'show_points': True,
}

LIMITS = {
    'depth_scale': (1e-5, 1.0), 'hfov_deg': (0.0, 170.0), 'stride': (1, 16),
    'max_range': (0.5, 10.0),
    'cam_x': (-1.0, 1.0), 'cam_y': (-1.0, 1.0), 'cam_z': (0.02, 1.5),
    'cam_pitch': (-45.0, 60.0), 'cam_roll': (-30.0, 30.0),
    'min_height': (0.0, 0.5), 'max_height': (0.05, 2.0),
    'car_length': (0.05, 2.0), 'car_width': (0.05, 2.0), 'wheelbase': (0.05, 2.0),
    'margin': (0.0, 0.5), 'stop_dist': (0.05, 3.0), 'slow_dist': (0.1, 5.0),
    'min_points': (1, 500), 'blind_fraction': (0.1, 1.0), 'auto_speed': (0.05, 1.0),
}

N_CANDIDATES = 15        # steering angles tried in auto mode
STALE_S = 0.5            # depth older than this counts as missing


def merge_config(base, update):
    """Copy of base with the known keys of update applied (unknown keys are ignored)."""
    out = copy.deepcopy(base)
    if not isinstance(update, dict):
        return out
    for key, value in update.items():
        if key not in out:
            continue
        if isinstance(out[key], bool):
            out[key] = bool(value)
            continue
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(value):
            continue
        lo, hi = LIMITS.get(key, (-math.inf, math.inf))
        value = max(lo, min(hi, value))
        out[key] = int(round(value)) if isinstance(out[key], int) else value
    out['slow_dist'] = max(out['slow_dist'], out['stop_dist'] + 0.05)
    out['max_height'] = max(out['max_height'], out['min_height'] + 0.02)
    return out


def make_config(update=None):
    return merge_config(DEFAULT_CONFIG, update)


def load_config(path):
    try:
        with open(path) as f:
            return make_config(json.load(f))
    except (OSError, ValueError):
        return make_config()


def _hfov(cfg, w, h):
    if cfg['hfov_deg'] > 0:
        return cfg['hfov_deg']
    return 87.0 if w / h > 1.55 else 80.0


def camera_rays(cfg, w, h):
    """Unit-depth rays (x right, y down) for the subsampled pixel grid, shape (rows, cols)."""
    s = cfg['stride']
    f = (w / 2.0) / math.tan(math.radians(_hfov(cfg, w, h)) / 2.0)
    us = (np.arange(0, w, s, dtype=np.float32) - (w - 1) / 2.0) / f
    vs = (np.arange(0, h, s, dtype=np.float32) - (h - 1) / 2.0) / f
    return np.meshgrid(us, vs)


def camera_to_vehicle(cfg, xc, yc, zc):
    """Camera (x right, y down, z forward) to vehicle (x forward, y left, z up) coordinates."""
    p, r = math.radians(cfg['cam_pitch']), math.radians(cfg['cam_roll'])
    # roll about the optical axis, then pitch down about the camera x axis
    xr = xc * math.cos(r) - yc * math.sin(r)
    yr = xc * math.sin(r) + yc * math.cos(r)
    x = zc * math.cos(p) - yr * math.sin(p) + cfg['cam_x']
    y = -xr + cfg['cam_y']
    z = cfg['cam_z'] - zc * math.sin(p) - yr * math.cos(p)
    return x, y, z


def free_distance(cfg, ox, oy, steer_rad):
    """How far (m) the front bumper can move along the arc for steer_rad before touching a point.

    ox, oy: obstacle points in the vehicle frame. Positive steer = left.
    """
    half_w = cfg['car_width'] / 2.0 + cfg['margin']
    front = cfg['car_length'] / 2.0
    horizon = cfg['max_range']
    if ox.size == 0:
        return horizon
    k = math.tan(steer_rad) / cfg['wheelbase']   # curvature of the rear-axle path
    if abs(k) < 1e-3:
        hit = (np.abs(oy) < half_w) & (ox > 0)
        dist = ox[hit] - front
    else:
        # Rear axle at x = -wheelbase/2, turning centre at (rear, R) in the vehicle frame.
        rear = -cfg['wheelbase'] / 2.0
        radius = 1.0 / k
        px = ox - rear
        r = np.hypot(px, oy - radius)
        hit = np.abs(r - abs(radius)) < half_w
        phi = np.arctan2(px[hit], math.copysign(1.0, radius) * (radius - oy[hit]))
        ahead = phi > 0
        dist = phi[ahead] * abs(radius) - (front - rear)
    if dist.size < cfg['min_points']:
        return horizon
    # The n-th nearest point rather than the nearest, so a few noisy pixels do not brake.
    n = cfg['min_points'] - 1
    near = np.partition(dist, n)[n]
    return float(max(0.0, min(horizon, near)))


def speed_factor(cfg, free):
    """0 at stop_dist or closer, 1 at slow_dist or further."""
    return float(np.clip((free - cfg['stop_dist']) / (cfg['slow_dist'] - cfg['stop_dist']), 0.0, 1.0))


def fit_floor(cfg, depth_m):
    """Estimate camera height, pitch and roll from the floor in the lower half of the image.

    Returns {'cam_z', 'cam_pitch', 'cam_roll', 'inliers'} or raises ValueError.
    """
    h, w = depth_m.shape
    us, vs = camera_rays(cfg, w, h)
    s = cfg['stride']
    d = depth_m[::s, ::s][:us.shape[0], :us.shape[1]]
    rows = slice(us.shape[0] // 2, None)
    d, us, vs = d[rows], us[rows], vs[rows]
    ok = (d > 0.15) & (d < 3.0)
    pts = np.stack([us[ok] * d[ok], vs[ok] * d[ok], d[ok]], axis=1)
    if len(pts) < 200:
        raise ValueError('มีจุด depth ที่ครึ่งล่างของภาพไม่พอ')
    rng = np.random.default_rng(0)
    best, best_count = None, 0
    for _ in range(200):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n[1] > 0:
            n = -n                     # point the normal up (camera -y)
        if -n[1] < 0.7:                # floor must be roughly horizontal (within ~45 deg)
            continue
        count = int(np.count_nonzero(np.abs((pts - a) @ n) < 0.015))
        if count > best_count:
            best, best_count = (n, a), count
    if best is None or best_count < 0.3 * len(pts):
        raise ValueError('หาพื้นเรียบไม่เจอ วางรถบนพื้นโล่งแล้วลองใหม่')
    n, a = best
    inl = pts[np.abs((pts - a) @ n) < 0.015]
    centroid = inl.mean(axis=0)
    _, _, vt = np.linalg.svd(inl - centroid)  # refine with least squares on the inliers
    n = vt[2] if vt[2][1] < 0 else -vt[2]
    height = float(abs(centroid @ n))
    # Up in camera coordinates is (-sin(roll), -cos(roll)cos(pitch), -cos(roll)sin(pitch)).
    roll = math.degrees(math.asin(max(-1.0, min(1.0, -n[0]))))
    pitch = math.degrees(math.atan2(-n[2], -n[1]))
    return {'cam_z': round(height, 3), 'cam_pitch': round(pitch, 1),
            'cam_roll': round(roll, 1), 'inliers': round(best_count / len(pts), 2)}


class ObstacleDetector:
    """Keeps the obstacle points from the newest depth frame and answers path queries."""

    def __init__(self, config):
        self._lock = threading.Lock()
        self.config = config
        self._rays_key = None
        self._rays = None
        self._ox = np.zeros(0, np.float32)
        self._oy = np.zeros(0, np.float32)
        self._oz = np.zeros(0, np.float32)
        self._blind = False
        self._stamp = 0.0
        self._depth_m = None

    def set_config(self, config):
        with self._lock:
            self.config = config
            self._rays_key = None

    def process(self, raw, scale):
        """raw: (h, w) uint16 depth image."""
        cfg = self.config
        h, w = raw.shape
        key = (w, h, cfg['stride'], cfg['hfov_deg'])
        if key != self._rays_key:
            self._rays = camera_rays(cfg, w, h)
            self._rays_key = key
        us, vs = self._rays
        s = cfg['stride']
        d = raw[::s, ::s][:us.shape[0], :us.shape[1]].astype(np.float32) * scale

        # Blind check: middle 40% of the width, rows from the centre to 85% down.
        rh, rw = d.shape
        centre = d[rh // 2:int(rh * 0.85), int(rw * 0.3):int(rw * 0.7)]
        blind = centre.size > 0 and np.count_nonzero(centre == 0) > cfg['blind_fraction'] * centre.size

        ok = (d > 0) & (d < cfg['max_range'])
        zc = d[ok]
        x, y, z = camera_to_vehicle(cfg, us[ok] * zc, vs[ok] * zc, zc)
        obst = (z > cfg['min_height']) & (z < cfg['max_height'])
        with self._lock:
            self._ox, self._oy, self._oz = x[obst], y[obst], z[obst]
            self._blind = bool(blind)
            self._stamp = time.monotonic()
            self._depth_m = (raw, scale)

    def snapshot(self):
        with self._lock:
            return self.config, self._ox, self._oy, self._oz, self._blind, self._stamp

    def latest_depth_m(self):
        with self._lock:
            if self._depth_m is None or time.monotonic() - self._stamp > STALE_S:
                return None
            raw, scale = self._depth_m
        return raw.astype(np.float32) * scale

    def assess(self, steer_rad):
        """Free distance and speed factor for driving forward with steer_rad."""
        cfg, ox, oy, _, blind, stamp = self.snapshot()
        stale = time.monotonic() - stamp > STALE_S
        if stale or blind:
            return {'free': 0.0, 'factor': 0.0, 'stale': stale, 'blind': blind}
        free = free_distance(cfg, ox, oy, steer_rad)
        return {'free': free, 'factor': speed_factor(cfg, free), 'stale': False, 'blind': False}

    def plan(self, max_steer, prev_steer):
        """Auto mode: (steer_rad, speed_factor, free, candidates) for the best direction."""
        cfg, ox, oy, _, blind, stamp = self.snapshot()
        if time.monotonic() - stamp > STALE_S or blind:
            return prev_steer, 0.0, 0.0, []
        horizon = min(cfg['max_range'], 2.0 * cfg['slow_dist'])
        cands = []
        best = None
        for steer in np.linspace(-max_steer, max_steer, N_CANDIDATES):
            free = min(horizon, free_distance(cfg, ox, oy, float(steer)))
            # Prefer open paths, then straight ahead, then keeping the current direction.
            score = (min(free, horizon) / horizon
                     - 0.15 * abs(steer) / max_steer
                     - 0.10 * abs(steer - prev_steer) / max_steer)
            cands.append((float(steer), free))
            if best is None or score > best[0]:
                best = (score, float(steer), free)
        _, steer, free = best
        return steer, speed_factor(cfg, free), free, cands


def render_topdown(cfg, ox, oy, oz, steer_rad, free, blind, stale, cands=(), size=360):
    """Top view: car at the bottom, obstacle points, the path for steer_rad and its free distance."""
    img = np.full((size, size, 3), 24, np.uint8)
    view_m = max(1.5, min(cfg['max_range'], 2.0 * cfg['slow_dist'] + 0.5))
    ppm = (size - 20) / view_m                       # pixels per metre
    ox0 = size // 2                                  # image column of y = 0
    oy0 = size - 10 - int(cfg['car_length'] / 2 * ppm)  # image row of x = 0

    def to_px(x, y):
        return int(round(ox0 - y * ppm)), int(round(oy0 - x * ppm))

    for m in np.arange(0.5, view_m + 0.01, 0.5):     # distance rings every 0.5 m
        cv2.circle(img, (ox0, oy0), int(m * ppm), (50, 50, 50), 1, cv2.LINE_AA)
        cv2.putText(img, f'{m:.1f}', (ox0 + 3, oy0 - int(m * ppm) + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (110, 110, 110), 1, cv2.LINE_AA)

    def arc_points(steer, length):
        k = math.tan(steer) / cfg['wheelbase']
        rear = -cfg['wheelbase'] / 2.0
        start = cfg['car_length'] / 2.0 - rear
        ts = np.linspace(start, start + max(0.0, length), 30)
        if abs(k) < 1e-3:
            xs, ys = ts + rear, np.zeros_like(ts)
        else:
            xs = np.sin(ts * k) / k + rear
            ys = (1 - np.cos(ts * k)) / k
        return np.array([to_px(x, y) for x, y in zip(xs, ys)], np.int32)

    for steer, f in cands:                            # auto mode candidates, faint
        cv2.polylines(img, [arc_points(steer, f)], False, (70, 70, 70), 1, cv2.LINE_AA)

    if cfg['show_points'] and ox.size:
        cols, rows = (ox0 - oy * ppm).astype(np.int32), (oy0 - ox * ppm).astype(np.int32)
        inside = (cols >= 0) & (cols < size) & (rows >= 0) & (rows < size)
        t = np.clip((oz[inside] - cfg['min_height']) / (cfg['max_height'] - cfg['min_height']), 0, 1)
        img[rows[inside], cols[inside]] = np.stack(
            [60 + 60 * t, 80 + 60 * (1 - t), np.full_like(t, 230)], axis=1).astype(np.uint8)

    half_w = cfg['car_width'] / 2.0
    if blind or stale:
        color = (60, 60, 230)
    elif free <= cfg['stop_dist']:
        color = (60, 60, 230)
    elif free < cfg['slow_dist']:
        color = (40, 170, 240)
    else:
        color = (110, 200, 60)
    path = arc_points(steer_rad, min(free, view_m))
    width = max(1, int(round((half_w + cfg['margin']) * 2 * ppm)))
    overlay = img.copy()
    cv2.polylines(overlay, [path], False, color, width, cv2.LINE_AA)
    cv2.addWeighted(overlay, 0.35, img, 0.65, 0, img)
    cv2.polylines(img, [path], False, color, 2, cv2.LINE_AA)

    l2 = cfg['car_length'] / 2.0
    cv2.rectangle(img, to_px(l2, half_w), to_px(-l2, -half_w), (200, 200, 200), -1)
    cv2.circle(img, to_px(cfg['cam_x'], cfg['cam_y']), 3, (255, 140, 40), -1)

    if stale:
        label = 'NO DEPTH'
    elif blind:
        label = 'BLIND (too close / no depth)'
    else:
        label = f'free {free:.2f} m'
    cv2.putText(img, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return img
