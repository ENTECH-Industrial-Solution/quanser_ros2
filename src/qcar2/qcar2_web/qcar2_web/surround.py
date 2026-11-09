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
CAMERA_KEYS = ('x', 'y', 'z', 'yaw', 'pitch', 'roll')

DEFAULT_CONFIG = {
    'size_px': 480,           # output is size_px x size_px
    'range_m': 2.4,           # ground width/height shown, car in the middle
    'show_grid': True,        # 0.5 m guide rectangles around the car
    'blend_power': 4.0,       # higher = narrower seams between cameras
    'car_length': 0.425,
    'car_width': 0.192,
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
        self.size = size
        self.calib_size = (intr['calib_width'], intr['calib_height'])
        mpp = rng / size

        # Ground point under the centre of every output pixel; image up = forward, left = left.
        idx = (np.arange(size, dtype=np.float64) + 0.5) * mpp
        gx, gy = np.meshgrid(rng / 2 - idx, rng / 2 - idx, indexing='ij')
        ground = np.stack([gx, gy, np.zeros_like(gx)], axis=-1)

        half_l, half_w = config['car_length'] / 2, config['car_width'] / 2
        under_car = (np.abs(gx) <= half_l) & (np.abs(gy) <= half_w)

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

        overlay, alpha = self._draw_overlay(config, mpp)
        self.overlay = overlay
        a = np.ascontiguousarray(alpha[:, :, 0], dtype=np.float32)
        self.alpha = (1.0 - a, a)   # weights of (camera view, overlay)

    def _project(self, ground, cam, intr):
        """Remap tables (float32) and raw blend weight for one camera."""
        h, w = self.size, self.size
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
