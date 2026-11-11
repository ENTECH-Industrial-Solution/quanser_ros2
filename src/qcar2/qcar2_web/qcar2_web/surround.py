"""Bird's-eye surround view from the four QCar2 CSI cameras.

Every output pixel is a point on the ground plane (z = 0) around the car. For each camera we
project those points into the fisheye image once and store the result as remap tables plus a
blend weight, so a frame costs only four cv2.remap calls and a weighted sum.

Frames:
- vehicle: origin on the ground under the middle of the car, x forward, y left, z up.
- camera (OpenCV): x right, y down, z along the optical axis.
  yaw = 0 looks forward, positive yaw turns left; positive pitch tilts the camera down.

Lens model: OpenCV fisheye (equidistant + polynomial). Quanser's calibration of the CSI camera
(fx 318.86, fy 312.14 at 820x410) uses the pinhole radial model, which turns back on itself
past ~40 degrees off-axis and so cannot map the edges of a ~150 degree lens; the same focal
lengths with an equidistant model cover the whole image.
"""

import copy
import json
import math
import os

import cv2
import numpy as np

SIDES = ('front', 'left', 'right', 'back')
TOP_ELEV = 89.5        # view elevations above this use the orthographic top view
GROUND_MAX_M = 6.0     # perspective views: ground further than this from the car stays empty
CAMERA_KEYS = ('x', 'y', 'z', 'yaw', 'pitch', 'roll')

DEFAULT_CONFIG = {
    'size_px': 480,           # output is size_px x size_px
    'range_m': 2.4,           # ground width/height shown, car in the middle
    'show_grid': True,        # 0.5 m guide rectangles around the car
    'blend_power': 4.0,       # higher = narrower seams between cameras
    'car_length': 0.425,
    'car_width': 0.192,
    'car_height': 0.18,       # box drawn for the car in the 3D (perspective) views
    # Virtual camera the surround view is seen from. elev 90 = straight down (the classic
    # bird's-eye view, zoom = range_m); below that a perspective camera orbits the car:
    # orbit = direction it looks (0 = from behind looking forward, 90 = from the right
    # looking left), dist = metres from the car centre, fov = horizontal field of view.
    'view': {'elev': 90.0, 'orbit': 0.0, 'dist': 1.6, 'fov': 75.0},
    'intrinsics': {           # at calib_width x calib_height; scaled for other sizes
        'calib_width': 820, 'calib_height': 410,
        'fx': 318.86, 'fy': 312.14, 'cx': 401.34, 'cy': 201.50,
        'k1': 0.0, 'k2': 0.0, 'k3': 0.0, 'k4': 0.0,
        'max_angle_deg': 85.0,
    },
    # Mounting estimates; tune them from the web page (metres / degrees).
    'cameras': {
        'front': {'x': 0.19, 'y': 0.0, 'z': 0.17, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0},
        'left': {'x': 0.0, 'y': 0.09, 'z': 0.17, 'yaw': 90.0, 'pitch': 0.0, 'roll': 0.0},
        'right': {'x': 0.0, 'y': -0.09, 'z': 0.17, 'yaw': -90.0, 'pitch': 0.0, 'roll': 0.0},
        'back': {'x': -0.20, 'y': 0.0, 'z': 0.17, 'yaw': 180.0, 'pitch': 0.0, 'roll': 0.0},
    },
}

# key: (min, max) accepted from clients
LIMITS = {
    'size_px': (160, 1000),
    'range_m': (0.6, 10.0),
    'blend_power': (0.5, 20.0),
    'car_length': (0.05, 2.0),
    'car_width': (0.05, 2.0),
    'car_height': (0.0, 1.0),
    'elev': (5.0, 90.0), 'orbit': (-360.0, 360.0), 'dist': (0.3, 10.0), 'fov': (30.0, 140.0),
    'x': (-2.0, 2.0), 'y': (-2.0, 2.0), 'z': (0.01, 2.0),
    'yaw': (-360.0, 360.0), 'pitch': (-89.0, 89.0), 'roll': (-180.0, 180.0),
    'fx': (10.0, 5000.0), 'fy': (10.0, 5000.0), 'cx': (0.0, 5000.0), 'cy': (0.0, 5000.0),
    'k1': (-1.0, 1.0), 'k2': (-1.0, 1.0), 'k3': (-1.0, 1.0), 'k4': (-1.0, 1.0),
    'max_angle_deg': (10.0, 120.0),
    'calib_width': (16, 8192), 'calib_height': (16, 8192),
}


def merge_config(base, update):
    """Copy of base with the known keys of update applied (unknown keys are ignored)."""
    out = copy.deepcopy(base)
    if not isinstance(update, dict):
        return out
    for key, value in update.items():
        if key not in out:
            continue
        if isinstance(out[key], dict):
            out[key] = merge_config(out[key], value)
        elif isinstance(out[key], bool):
            out[key] = bool(value)
        else:
            try:
                value = float(value)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(value):
                continue
            lo, hi = LIMITS.get(key, (-math.inf, math.inf))
            value = max(lo, min(hi, value))
            out[key] = int(round(value)) if isinstance(out[key], int) else value
    return out


def make_config(update=None):
    return merge_config(DEFAULT_CONFIG, update)


def load_config(path):
    try:
        with open(path) as f:
            return make_config(json.load(f))
    except (OSError, ValueError):
        return make_config()


def save_config(path, config):
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(config, f, indent=2)
    os.replace(tmp, path)


def _rotation(yaw_deg, pitch_deg, roll_deg):
    """Rotation taking camera-frame vectors to the vehicle frame."""
    y, p, r = (math.radians(a) for a in (yaw_deg, pitch_deg, roll_deg))
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    # Columns: camera x (right) = -y, camera y (down) = -z, camera z (optical axis) = +x.
    base = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    return rz @ ry @ rx @ base


class SurroundModel:
    """Precomputed remap tables and blend weights for one configuration."""

    def __init__(self, config):
        self.config = config
        size = config['size_px']
        rng = config['range_m']
        intr = config['intrinsics']
        self.calib_size = (intr['calib_width'], intr['calib_height'])
        view = config['view']
        self.top = view['elev'] > TOP_ELEV
        half_l, half_w = config['car_length'] / 2, config['car_width'] / 2

        if self.top:
            self.h = self.w = size
            mpp = rng / size
            # Ground point under the centre of every output pixel; image up = forward, left = left.
            idx = (np.arange(size, dtype=np.float64) + 0.5) * mpp
            gx, gy = np.meshgrid(rng / 2 - idx, rng / 2 - idx, indexing='ij')
            ground = np.stack([gx, gy, np.zeros_like(gx)], axis=-1)
            off_ground = np.zeros_like(gx, dtype=bool)
        else:
            mpp = None
            self.w, self.h = size, int(round(size * 0.625)) // 2 * 2   # 16:10
            self._set_virtual_camera(view)
            # Ray through every pixel, intersected with the ground plane z = 0.
            jj, ii = np.meshgrid(np.arange(self.w) + 0.5 - self.w / 2,
                                 np.arange(self.h) + 0.5 - self.h / 2)
            rays = (self.fwd[None, None, :] + (jj / self.f_px)[..., None] * self.right
                    + (ii / self.f_px)[..., None] * self.down)
            rz = rays[..., 2]
            hits = rz < -1e-6
            t = np.where(hits, -self.cam_pos[2] / np.where(hits, rz, -1.0), 0.0)
            ground = self.cam_pos + t[..., None] * rays
            ground[..., 2] = 0.0
            gx, gy = ground[..., 0], ground[..., 1]
            off_ground = ~hits | (np.hypot(gx, gy) > GROUND_MAX_M)
            ground[off_ground] = (1e3, 1e3, 0.0)
            gx, gy = ground[..., 0], ground[..., 1]
        self.size = self.w

        under_car = ((np.abs(gx) <= half_l) & (np.abs(gy) <= half_w)) | off_ground

        weights, us, vs = {}, {}, {}
        for side in SIDES:
            mx, my, w = self._project(ground, config['cameras'][side], intr)
            w[under_car] = 0.0
            weights[side], us[side], vs[side] = w, mx, my
        covered = sum(weights.values()) > 1e-6

        # Every output pixel blends its two best cameras (weights renormalised): with
        # blend_power a third camera adds next to nothing. The four frames are stacked into
        # one image (SIDES order, top to bottom), so each of the two layers is a single
        # remap, and all the blending stays in uint8 (cv2.blendLinear), not float32.
        w_all = np.stack([weights[s] for s in SIDES])
        u_all = np.stack([us[s] for s in SIDES])
        v_all = np.stack([vs[s] for s in SIDES])
        order = np.argsort(-w_all, axis=0)[:2]
        w_ab = np.take_along_axis(w_all, order, 0)
        w_sum = w_ab.sum(axis=0)
        frac_a = np.where(covered, w_ab[0] / np.where(covered, w_sum, 1.0), 1.0)
        self.blend = (frac_a.astype(np.float32), (1.0 - frac_a).astype(np.float32))
        layers = []
        for k in range(2):
            u = np.take_along_axis(u_all, order[k:k + 1], 0)[0]
            v = np.take_along_axis(v_all, order[k:k + 1], 0)[0]
            ok = (u >= 0) & covered
            layers.append((ok, u, v, order[k]))
        # Remap tables for full-size frames and for frames decoded at half size
        # (cv2.IMREAD_REDUCED_COLOR_2): {frame (w, h): (map A, map B)}.
        self.maps = {}
        cw, ch = self.calib_size
        for scale in (1, 2):
            fw, fh = cw // scale, ch // scale
            pair = []
            for ok, u, v, cam in layers:
                # pixel centres: x_small = (x + 0.5) / scale - 0.5
                mu = np.where(ok, (u + 0.5) / scale - 0.5, -10.0).astype(np.float32)
                mv = np.where(ok, (v + 0.5) / scale - 0.5 + cam * fh, -10.0).astype(np.float32)
                pair.append(cv2.convertMaps(mu, mv, cv2.CV_16SC2))
            self.maps[(fw, fh)] = pair
        self.background = 40

        # Area where each camera contributes most, for highlighting it while calibrating.
        dominant = np.argmax(np.stack([weights[side] for side in SIDES]), axis=0)
        self.regions = {}
        for i, side in enumerate(SIDES):
            region = (dominant == i) & covered
            contours, _ = cv2.findContours(region.astype(np.uint8), cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            self.regions[side] = (~region, contours)

        overlay, alpha = (self._draw_overlay(config, mpp) if self.top
                          else self._draw_overlay_3d(config))
        self.overlay = overlay
        a = np.ascontiguousarray(alpha[:, :, 0], dtype=np.float32)
        self.alpha = (1.0 - a, a)   # weights of (camera view, overlay)

    def _project(self, ground, cam, intr):
        """Remap tables (float32) and raw blend weight for one camera."""
        h, w = self.h, self.w
        rot = _rotation(cam['yaw'], cam['pitch'], cam['roll'])
        t = np.array([cam['x'], cam['y'], cam['z']])
        # = rot.T @ (p - t) per point. einsum, not "@": "@" goes to OpenBLAS, whose thread
        # pool can spin forever when called from the executor threads (seen: a core at 100%
        # for 20+ minutes in this line, starving every other callback of the node).
        pc = np.einsum('ni,ij->nj', ground.reshape(-1, 3) - t, rot)
        x, y, z = pc[:, 0], pc[:, 1], pc[:, 2]

        r = np.hypot(x, y)
        theta = np.arctan2(r, z)  # angle from the optical axis
        t2 = theta * theta
        theta_d = theta * (1 + t2 * (intr['k1'] + t2 * (intr['k2'] + t2 * (intr['k3'] + t2 * intr['k4']))))
        scale = np.where(r > 1e-9, theta_d / np.maximum(r, 1e-9), 1.0)
        u = intr['fx'] * x * scale + intr['cx']
        v = intr['fy'] * y * scale + intr['cy']

        cw, ch = self.calib_size
        max_angle = math.radians(intr['max_angle_deg'])
        valid = (theta < max_angle) & (u >= 0) & (u <= cw - 1) & (v >= 0) & (v <= ch - 1)

        # Favour the camera that looks most directly at a point, fading out towards the
        # image border so seams are soft instead of a hard cut.
        border = np.minimum.reduce([u, cw - 1 - u, v, ch - 1 - v])
        feather = np.clip(border / 25.0, 0.0, 1.0)
        facing = np.clip(np.cos(theta), 0.0, 1.0) ** self.config['blend_power']
        weight = np.where(valid, facing * feather, 0.0)

        u = np.where(valid, u, -1).astype(np.float32).reshape(h, w)
        v = np.where(valid, v, -1).astype(np.float32).reshape(h, w)
        return u, v, weight.reshape(h, w)

    def _draw_overlay(self, config, mpp):
        size = self.size
        overlay = np.zeros((size, size, 3), np.uint8)
        mask = np.zeros((size, size), np.uint8)
        c = size / 2.0

        def px(x_m, y_m):  # vehicle metres -> integer pixel (col, row)
            return int(round(c - y_m / mpp)), int(round(c - x_m / mpp))

        if config['show_grid']:
            step = 0.5
            n = int(config['range_m'] / 2 / step) + 1
            for i in range(1, n + 1):
                d = i * step
                p1 = px(config['car_length'] / 2 + d, config['car_width'] / 2 + d)
                p2 = px(-config['car_length'] / 2 - d, -config['car_width'] / 2 - d)
                color = (60, 200, 255) if i == 1 else (200, 200, 200)
                cv2.rectangle(overlay, p1, p2, color, 1, cv2.LINE_AA)
                cv2.rectangle(mask, p1, p2, 255, 1, cv2.LINE_AA)
                label = f'{d:.1f} m'
                cv2.putText(overlay, label, (p1[0] + 3, p1[1] + 12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.35, color, 1, cv2.LINE_AA)
                cv2.putText(mask, label, (p1[0] + 3, p1[1] + 12), cv2.FONT_HERSHEY_SIMPLEX,
                            0.35, 255, 1, cv2.LINE_AA)

        # The car itself: body, windscreen and a nose marker so "forward" is obvious.
        half_l, half_w = config['car_length'] / 2, config['car_width'] / 2
        body = np.array([px(half_l, half_w), px(half_l, -half_w),
                         px(-half_l, -half_w), px(-half_l, half_w)], np.int32)
        cv2.fillConvexPoly(overlay, body, (70, 60, 55), cv2.LINE_AA)
        cv2.polylines(overlay, [body], True, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.fillConvexPoly(mask, body, 255, cv2.LINE_AA)
        glass = np.array([px(half_l * 0.55, half_w * 0.8), px(half_l * 0.55, -half_w * 0.8),
                          px(half_l * 0.15, -half_w * 0.7), px(half_l * 0.15, half_w * 0.7)], np.int32)
        cv2.fillConvexPoly(overlay, glass, (150, 120, 90), cv2.LINE_AA)
        nose = np.array([px(half_l * 0.95, 0), px(half_l * 0.72, half_w * 0.35),
                         px(half_l * 0.72, -half_w * 0.35)], np.int32)
        cv2.fillConvexPoly(overlay, nose, (60, 200, 255), cv2.LINE_AA)

        alpha = cv2.merge([mask.astype(np.float32) / 255.0] * 3)
        return overlay, alpha

    def _set_virtual_camera(self, view):
        """Pose of the perspective virtual camera, looking at the car centre."""
        elev, orbit = math.radians(view['elev']), math.radians(view['orbit'])
        fwd = np.array([math.cos(elev) * math.cos(orbit), math.cos(elev) * math.sin(orbit),
                        -math.sin(elev)])
        target = np.array([0.0, 0.0, 0.0])
        self.cam_pos = target - view['dist'] * fwd
        right = np.cross(fwd, [0.0, 0.0, 1.0])
        right /= np.linalg.norm(right)
        self.fwd, self.right, self.down = fwd, right, np.cross(fwd, right)
        self.f_px = (self.w / 2) / math.tan(math.radians(view['fov']) / 2)

    def _to_px(self, pts):
        """Vehicle points (N, 3) -> pixel (N, 2) float and in-front mask, perspective view."""
        d = np.asarray(pts, dtype=np.float64) - self.cam_pos
        depth = d @ self.fwd
        ok = depth > 0.05
        z = np.where(ok, depth, 1.0)
        u = self.w / 2 + self.f_px * (d @ self.right) / z
        v = self.h / 2 + self.f_px * (d @ self.down) / z
        return np.stack([u, v], axis=-1), ok

    def _draw_overlay_3d(self, config):
        """Guide rectangles on the ground and the car as a box, seen by the virtual camera."""
        h, w = self.h, self.w
        overlay = np.zeros((h, w, 3), np.uint8)
        mask = np.zeros((h, w), np.uint8)
        lim = 4 * max(h, w)

        def line(pts, color, thick=1):
            px, ok = self._to_px(pts)
            for a, b, oa, ob in zip(px[:-1], px[1:], ok[:-1], ok[1:]):
                if oa and ob and np.all(np.abs(a) < lim) and np.all(np.abs(b) < lim):
                    p1, p2 = tuple(int(round(c)) for c in a), tuple(int(round(c)) for c in b)
                    cv2.line(overlay, p1, p2, color, thick, cv2.LINE_AA)
                    cv2.line(mask, p1, p2, 255, thick, cv2.LINE_AA)

        def rect(x0, y0, x1, y1, z=0.0, n=24):
            edges = []
            for (ax, ay), (bx, by) in (((x0, y0), (x0, y1)), ((x0, y1), (x1, y1)),
                                       ((x1, y1), (x1, y0)), ((x1, y0), (x0, y0))):
                tt = np.linspace(0.0, 1.0, n)
                edges.append(np.stack([ax + (bx - ax) * tt, ay + (by - ay) * tt,
                                       np.full(n, z)], axis=-1))
            return edges

        half_l, half_w = config['car_length'] / 2, config['car_width'] / 2
        if config['show_grid']:
            step = 0.5
            for i in range(1, int(3.0 / step) + 1):
                d = i * step
                color = (60, 200, 255) if i == 1 else (200, 200, 200)
                for edge in rect(half_l + d, half_w + d, -half_l - d, -half_w - d):
                    line(edge, color)
                px, ok = self._to_px([[half_l + d, half_w + d, 0.0]])
                if d <= 2.0 and ok[0] and 0 <= px[0][0] < w - 30 and h * 0.2 <= px[0][1] < h:
                    org = (int(px[0][0]) + 3, int(px[0][1]) - 3)
                    cv2.putText(overlay, f'{d:.1f} m', org, cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
                    cv2.putText(mask, f'{d:.1f} m', org, cv2.FONT_HERSHEY_SIMPLEX, 0.35, 255, 1, cv2.LINE_AA)

        # The car: a box, its silhouette filled, top face and edges drawn on it.
        ch = config['car_height']
        corners = np.array([[sx * half_l, sy * half_w, sz * ch]
                            for sz in (0, 1) for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))])
        px, ok = self._to_px(corners)
        if ok.all() and np.all(np.abs(px) < lim):
            hull = cv2.convexHull(np.round(px).astype(np.int32))
            cv2.fillConvexPoly(overlay, hull, (70, 60, 55), cv2.LINE_AA)
            cv2.fillConvexPoly(mask, hull, 255, cv2.LINE_AA)
            top = np.round(px[4:]).astype(np.int32)
            cv2.fillConvexPoly(overlay, top, (95, 85, 78), cv2.LINE_AA)
            glass = np.array([[half_l * 0.55, half_w * 0.8, ch], [half_l * 0.55, -half_w * 0.8, ch],
                              [half_l * 0.15, -half_w * 0.7, ch], [half_l * 0.15, half_w * 0.7, ch]])
            gp, gok = self._to_px(glass)
            if gok.all():
                cv2.fillConvexPoly(overlay, np.round(gp).astype(np.int32), (150, 120, 90), cv2.LINE_AA)
            nose = np.array([[half_l * 0.95, 0, ch], [half_l * 0.72, half_w * 0.35, ch],
                             [half_l * 0.72, -half_w * 0.35, ch]])
            npx, nok = self._to_px(nose)
            if nok.all():
                cv2.fillConvexPoly(overlay, np.round(npx).astype(np.int32), (60, 200, 255), cv2.LINE_AA)
            for a, b in ((0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
                         (0, 4), (1, 5), (2, 6), (3, 7)):
                cv2.line(overlay, tuple(np.round(px[a]).astype(int)), tuple(np.round(px[b]).astype(int)),
                         (200, 200, 200), 1, cv2.LINE_AA)

        alpha = cv2.merge([mask.astype(np.float32) / 255.0] * 3)
        return overlay, alpha

    def compose(self, frames, highlight=None):
        """frames: side -> BGR image (or None). Returns the surround BGR image.

        highlight: a side whose area is outlined while everything else is dimmed.
        """
        imgs = [frames.get(side) for side in SIDES]
        present = [img for img in imgs if img is not None]
        size = self.calib_size
        if present and all(img.shape[1::-1] == present[0].shape[1::-1] for img in present) \
                and present[0].shape[1::-1] in self.maps:
            size = present[0].shape[1::-1]   # all full size, or all decoded at half size
        stack = []
        for img in imgs:
            if img is None:
                img = np.zeros((size[1], size[0], 3), np.uint8)   # camera missing: black
            if img.ndim == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            if img.shape[1::-1] != size:
                img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
            stack.append(img)
        stack = np.concatenate(stack, axis=0)
        (a1, a2), (b1, b2) = self.maps[size]
        border = (self.background,) * 3
        layer_a = cv2.remap(stack, a1, a2, cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=border)
        layer_b = cv2.remap(stack, b1, b2, cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT, borderValue=border)
        out = cv2.blendLinear(layer_a, layer_b, *self.blend)
        # Car and guide lines on top: out = out * (1 - alpha) + overlay * alpha
        out = cv2.blendLinear(out, self.overlay, *self.alpha)
        if highlight in self.regions:
            outside, contours = self.regions[highlight]
            out[outside] //= 2
            cv2.drawContours(out, contours, -1, (0, 220, 255), 2, cv2.LINE_AA)
        return out
