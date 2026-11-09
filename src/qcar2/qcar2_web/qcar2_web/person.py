"""Person following: YOLO on the RealSense colour image, distance from the depth image.

Each detected person gets a distance from the depth pixels inside its segmentation mask (or
the middle of its box when the model has no masks) and a position in the vehicle frame used
by obstacle.py (x forward, y left, z up). One person is the target: the most centred one in
range when following starts, then whichever detection is nearest to where the target was.
If the target is not seen for `lost_timeout` the car stops; after `reacquire_s` the most
centred person in range becomes the new target.

The car steers towards the target with pure pursuit (bicycle model around the rear axle)
and drives forward faster the further the target is beyond `follow_dist`. It never reverses.

The colour and depth streams are not aligned: a colour pixel is mapped to the depth image by
its viewing angle (colour and depth fields of view; principal points in the middle). The
~15 mm between the two sensors is ignored, which is fine beyond ~0.5 m.

The model (ultralytics, PyTorch/TensorRT) is loaded the first time it is needed, which takes
a few seconds; ultralytics is only imported then, so the web server runs without it.
"""

import copy
import json
import math
import threading
import time

import cv2
import numpy as np

from qcar2_web import obstacle

DEFAULT_CONFIG = {
    'follow_dist': 1.0,       # keep this far from the target, measured from the front bumper (m)
    'speed_band': 1.0,        # full follow speed this far beyond follow_dist (m)
    'follow_speed': 0.4,      # top speed while following, fraction of max_speed
    'max_dist': 4.0,          # people further than this are ignored (m)
    'conf': 0.5,              # minimum detection confidence
    'gate': 0.7,              # how far the target may move between two frames (m)
    'lost_timeout': 0.7,      # stop when the target has not been seen this long (s)
    'reacquire_s': 3.0,       # then pick a new target after this long (s)
    'color_hfov_deg': 69.4,   # D435 colour camera at 16:9 (set equal to depth's if aligned)
    'steer_smooth': 0.5,      # 0 = keep the old steering, 1 = no smoothing
}

LIMITS = {
    'follow_dist': (0.4, 3.0), 'speed_band': (0.2, 3.0), 'follow_speed': (0.05, 1.0),
    'max_dist': (1.0, 8.0), 'conf': (0.1, 0.95), 'gate': (0.2, 2.0),
    'lost_timeout': (0.2, 5.0), 'reacquire_s': (0.5, 30.0), 'color_hfov_deg': (30.0, 120.0),
    'steer_smooth': (0.05, 1.0),
}

PERSON_CLASS = 0          # COCO
STALE_S = 0.5             # a detection result older than this counts as missing
MIN_DEPTH_M = 0.2


def merge_config(base, update):
    """Copy of base with the known keys of update applied (unknown keys are ignored)."""
    out = copy.deepcopy(base)
    if not isinstance(update, dict):
        return out
    for key, value in update.items():
        if key not in out:
            continue
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(value):
            continue
        lo, hi = LIMITS.get(key, (-math.inf, math.inf))
        out[key] = max(lo, min(hi, value))
    out['reacquire_s'] = max(out['reacquire_s'], out['lost_timeout'])
    return out


def make_config(update=None):
    return merge_config(DEFAULT_CONFIG, update)


def load_config(path):
    try:
        with open(path) as f:
            return make_config(json.load(f))
    except (OSError, ValueError):
        return make_config()


def _focal(w, hfov_deg):
    return (w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def color_to_depth_px(pts, color_shape, depth_shape, color_hfov, depth_hfov):
    """Map (n, 2) colour pixel coordinates to depth pixel coordinates by viewing angle."""
    hc, wc = color_shape[:2]
    hd, wd = depth_shape[:2]
    fc, fd = _focal(wc, color_hfov), _focal(wd, depth_hfov)
    out = np.empty_like(pts, dtype=np.float32)
    out[:, 0] = (pts[:, 0] - (wc - 1) / 2.0) / fc * fd + (wd - 1) / 2.0
    out[:, 1] = (pts[:, 1] - (hc - 1) / 2.0) / fc * fd + (hd - 1) / 2.0
    return out


def person_depth(depth_m, polygon, box, color_shape, color_hfov, depth_hfov, max_range):
    """Median depth (m) of the person's pixels, or None if too few are valid.

    polygon: (n, 2) mask outline in colour pixels, or None to use the middle of box.
    """
    hd, wd = depth_m.shape
    if polygon is not None and len(polygon) >= 3:
        poly = color_to_depth_px(polygon, color_shape, depth_m.shape, color_hfov, depth_hfov)
    else:
        x1, y1, x2, y2 = box
        cx, w, h = (x1 + x2) / 2.0, x2 - x1, y2 - y1
        # torso: middle 40 % of the width, 20-60 % of the height
        poly = np.array([[cx - 0.2 * w, y1 + 0.2 * h], [cx + 0.2 * w, y1 + 0.2 * h],
                         [cx + 0.2 * w, y1 + 0.6 * h], [cx - 0.2 * w, y1 + 0.6 * h]], np.float32)
        poly = color_to_depth_px(poly, color_shape, depth_m.shape, color_hfov, depth_hfov)
    x0, y0 = np.floor(poly.min(axis=0)).astype(int)
    x1, y1 = np.ceil(poly.max(axis=0)).astype(int)
    x0, y0, x1, y1 = max(0, x0), max(0, y0), min(wd, x1 + 1), min(hd, y1 + 1)
    if x1 <= x0 or y1 <= y0:
        return None
    mask = np.zeros((y1 - y0, x1 - x0), np.uint8)
    cv2.fillPoly(mask, [np.round(poly - [x0, y0]).astype(np.int32)], 1)
    vals = depth_m[y0:y1, x0:x1][mask > 0]
    vals = vals[(vals > MIN_DEPTH_M) & (vals < max_range)]
    if vals.size < 20:
        return None
    return float(np.median(vals))


class PersonFollower:
    """Runs the detector on the newest colour frame, keeps the target and the steering command."""

    def __init__(self, config, model_path, logger):
        self.config = config
        self.model_path = model_path
        self.logger = logger
        self._lock = threading.Lock()
        self._model = None
        self.state = 'idle'       # idle, loading, error, ready
        self.error = None
        self._people = []         # detections of the newest frame
        self._target = None       # {'x', 'y', 'seen', 'track'}
        self._stamp = 0.0
        self._steer = 0.0

    def set_config(self, config):
        with self._lock:
            self.config = config

    # ------------------------------------------------------------ model

    def _load_model(self):
        self.state = 'loading'
        try:
            from ultralytics import YOLO
            path = self.model_path
            task = 'segment' if '-seg' in path else 'detect'
            model = YOLO(path, task=task)
            # TensorRT engines have a fixed input size, but predict() still defaults to 640.
            imgsz = (self._engine_imgsz(path) if path.endswith('.engine') else None) or 640
            model.predict(np.zeros((480, 640, 3), np.uint8), classes=[PERSON_CLASS],
                          imgsz=imgsz, verbose=False)
            self._model, self._imgsz = model, imgsz
            self.state = 'ready'
            self.logger.info(f'Person: loaded {path} (imgsz {imgsz})')
        except Exception as e:  # noqa: BLE001 - any failure here just disables following
            self.state, self.error = 'error', f'{type(e).__name__}: {e}'
            self.logger.error(f'Person: cannot load {self.model_path}: {self.error}')

    @staticmethod
    def _engine_imgsz(path):
        """[h, w] from the JSON metadata block that ultralytics puts before the engine, or None."""
        try:
            with open(path, 'rb') as f:
                meta_len = int.from_bytes(f.read(4), byteorder='little')
                if not 0 < meta_len < 100000:
                    return None
                imgsz = json.loads(f.read(meta_len).decode('utf-8')).get('imgsz')
            if isinstance(imgsz, str):
                imgsz = json.loads(imgsz)
            return [int(v) for v in imgsz] if isinstance(imgsz, list) and len(imgsz) == 2 else None
        except (OSError, ValueError, UnicodeDecodeError, TypeError):
            return None

    def ensure_model(self):
        """True once the model is ready; loads it (blocking) the first time."""
        if self._model is None and self.state not in ('loading', 'error'):
            self._load_model()
        return self._model is not None

    # ------------------------------------------------------------ detection

    def process(self, color, depth_m, obs_cfg, depth_hfov):
        """Detect people in color (BGR), measure them in depth_m (metres or None), track."""
        cfg = self.config
        res = self._model.predict(color, classes=[PERSON_CLASS], conf=cfg['conf'],
                                  imgsz=self._imgsz, verbose=False)[0]
        boxes = res.boxes.xyxy.cpu().numpy() if res.boxes is not None else np.zeros((0, 4))
        confs = res.boxes.conf.cpu().numpy() if res.boxes is not None else np.zeros(0)
        polys = res.masks.xy if res.masks is not None else [None] * len(boxes)
        h, w = color.shape[:2]
        fc = _focal(w, cfg['color_hfov_deg'])
        people = []
        for box, conf, poly in zip(boxes, confs, polys):
            p = {'box': box, 'conf': float(conf), 'poly': poly, 'dist': None, 'x': None, 'y': None}
            cx = (box[0] + box[2]) / 2.0
            cy = box[1] + 0.4 * (box[3] - box[1])        # chest height
            if depth_m is not None:
                d = person_depth(depth_m, poly, box, color.shape, cfg['color_hfov_deg'],
                                 depth_hfov, min(cfg['max_dist'] + 1.0, 10.0))
                if d is not None:
                    xn, yn = (cx - (w - 1) / 2.0) / fc, (cy - (h - 1) / 2.0) / fc
                    x, y, _ = obstacle.camera_to_vehicle(obs_cfg, xn * d, yn * d, d)
                    p.update(dist=d, x=float(x), y=float(y))
            people.append(p)
        self._track(people, time.monotonic())

    def _track(self, people, now):
        cfg = self.config
        valid = [p for p in people if p['x'] is not None
                 and math.hypot(p['x'], p['y']) <= cfg['max_dist']]
        with self._lock:
            tgt = self._target
            if tgt is not None and now - tgt['seen'] > cfg['reacquire_s']:
                tgt = None
            if tgt is None:
                # New target: the most centred person, nearer ones slightly preferred.
                best = min(valid, key=lambda p: abs(math.atan2(p['y'], p['x']))
                           + 0.1 * p['dist'], default=None)
                if best is not None:
                    tgt = {'track': (self._target or {}).get('track', 0) + 1}
            else:
                near = [(math.hypot(p['x'] - tgt['x'], p['y'] - tgt['y']), p) for p in valid]
                near = [(d, p) for d, p in near if d < cfg['gate'] * (1.0 + now - tgt['seen'])]
                best = min(near, key=lambda dp: dp[0])[1] if near else None
            for p in people:
                p['target'] = p is best
            if best is not None:
                tgt.update(x=best['x'], y=best['y'], dist=best['dist'], seen=now)
            self._target, self._people, self._stamp = tgt, people, now

    def reset_target(self):
        """Forget the target; the next frame picks the most centred person."""
        with self._lock:
            self._target = None
            self._steer = 0.0

    # ------------------------------------------------------------ control

    def command(self, obs_cfg, max_steer):
        """(steer_rad, speed_fraction) towards the target; speed 0 when there is none."""
        cfg = self.config
        now = time.monotonic()
        with self._lock:
            tgt = self._target
            fresh = now - self._stamp < STALE_S
            if not fresh or tgt is None or now - tgt['seen'] > cfg['lost_timeout']:
                return self._steer, 0.0
            rear = -obs_cfg['wheelbase'] / 2.0
            dx, dy = tgt['x'] - rear, tgt['y']
            alpha = math.atan2(dy, dx)
            ld = max(0.3, math.hypot(dx, dy))
            steer = math.atan(2.0 * obs_cfg['wheelbase'] * math.sin(alpha) / ld)
            steer = max(-max_steer, min(max_steer, steer))
            self._steer += cfg['steer_smooth'] * (steer - self._steer)
            gap = math.hypot(tgt['x'], tgt['y']) - obs_cfg['car_length'] / 2.0
            frac = float(np.clip((gap - cfg['follow_dist']) / cfg['speed_band'], 0.0, 1.0))
            # Slow down while the target is far to the side, so the car turns before it runs off.
            frac *= max(0.3, math.cos(alpha))
            return self._steer, frac * cfg['follow_speed']

    def status(self):
        cfg = self.config
        now = time.monotonic()
        with self._lock:
            tgt = self._target
            out = {
                'state': self.state,
                'error': self.error,
                'stale': now - self._stamp > STALE_S,
                'count': len(self._people),
                'target': None,
            }
            if tgt is not None and 'x' in tgt:
                out['target'] = {
                    'dist': round(math.hypot(tgt['x'], tgt['y']), 2),
                    'bearing_deg': round(math.degrees(math.atan2(tgt['y'], tgt['x'])), 1),
                    'lost': now - tgt['seen'] > cfg['lost_timeout'],
                    'track': tgt['track'],
                }
            return out

    def render(self, color, out_w=640):
        """The colour frame with every detection drawn; the target in green."""
        scale = out_w / color.shape[1]
        img = cv2.resize(color, (out_w, int(round(color.shape[0] * scale))),
                         interpolation=cv2.INTER_AREA)
        with self._lock:
            people = list(self._people)
            tgt = self._target
        lost = tgt is not None and time.monotonic() - tgt.get('seen', 0) > self.config['lost_timeout']
        for p in people:
            target = p.get('target')
            color_ = (90, 220, 60) if target else (200, 200, 200) if p['dist'] is not None else (90, 90, 90)
            if p['poly'] is not None and len(p['poly']) >= 3:
                pts = np.round(p['poly'] * scale).astype(np.int32)
                if target:
                    overlay = img.copy()
                    cv2.fillPoly(overlay, [pts], color_)
                    cv2.addWeighted(overlay, 0.3, img, 0.7, 0, img)
                cv2.polylines(img, [pts], True, color_, 1, cv2.LINE_AA)
            x1, y1, x2, y2 = (np.asarray(p['box']) * scale).astype(int)
            cv2.rectangle(img, (x1, y1), (x2, y2), color_, 2 if target else 1)
            label = f'{p["dist"]:.2f} m' if p['dist'] is not None else 'no depth'
            if target:
                label = 'TARGET ' + label
            cv2.putText(img, label, (x1 + 3, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        color_, 1, cv2.LINE_AA)
        if self.state != 'ready':
            text, c = {'idle': ('waiting', (200, 200, 200)), 'loading': ('loading model...', (40, 170, 240)),
                       'error': ('model error', (60, 60, 230))}[self.state]
        elif lost:
            text, c = 'TARGET LOST - stopped', (60, 60, 230)
        elif tgt is None:
            text, c = 'no target', (40, 170, 240)
        else:
            text, c = f'following #{tgt["track"]}', (90, 220, 60)
        cv2.putText(img, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2, cv2.LINE_AA)
        return img
