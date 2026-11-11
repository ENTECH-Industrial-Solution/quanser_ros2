"""Object detection the web way: collect RealSense images, label them, train YOLO, use the result.

Everything lives in ~/qcar2_learn (nothing leaves the car):

    datasets/<name>/meta.json          {"classes": [...]}
    datasets/<name>/images/<id>.jpg    captured frames
    datasets/<name>/labels/<id>.txt    YOLO boxes "class cx cy w h" (0..1); file present = labelled
    datasets/<name>/runs/<run>/        one training: job.json, progress.json, info.json, weights/
    active.json                        the model the live detector uses

Training and TensorRT export run in a separate process (learn_train.py), one job at a time.
"""

import atexit
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

ROOT = os.path.expanduser('~/qcar2_learn')
NAME_RE = re.compile(r'^[\w฀-๿-]{1,40}$')
ID_RE = re.compile(r'^[\w-]{1,64}$')
COCO_MODEL = os.path.expanduser(
    '~/Documents/Quanser_demo/libraries/resources/pretrained_models/yolov8s-seg.pt')
WANT_S = 6.0               # the page keeps the camera subscribed by polling the status
AUTO_MIN_S = 0.5
BLUR_MIN = 25.0            # variance of the Laplacian below this = too blurry to keep
SAME_MAX = 5.0             # mean grey difference (0..255) below this = nearly the same picture
MIN_LABELLED = 4
CAM_ORDER = ['realsense', 'front', 'left', 'right', 'back']
CAM_LABELS = {'realsense': 'RealSense', 'front': 'กล้องหน้า', 'back': 'กล้องหลัง',
              'left': 'กล้องซ้าย', 'right': 'กล้องขวา'}
FRESH_S = 2.0              # a frame older than this is not captured


def _read_json(path, default=None):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def _write_json(path, data):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)


def check_name(name):
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise ValueError('ชื่อใช้ได้เฉพาะตัวอักษร ตัวเลข _ และ - (ไม่เกิน 40 ตัว)')
    return name


def check_id(i):
    if not isinstance(i, str) or not ID_RE.match(i):
        raise ValueError('bad image id')
    return i


def read_boxes(path, conf=False):
    """[[class, cx, cy, w, h]] from a YOLO label file; conf=True keeps a 6th column if there."""
    boxes = []
    try:
        with open(path) as f:
            for line in f:
                p = line.split()
                if len(p) >= 5:
                    n = 6 if conf and len(p) >= 6 else 5
                    boxes.append([int(float(p[0]))] + [round(float(v), 6) for v in p[1:n]])
    except (OSError, ValueError):
        pass
    return boxes


def write_boxes(path, boxes):
    lines = []
    for b in boxes:
        c, cx, cy, w, h = int(b[0]), *[min(1.0, max(0.0, float(v))) for v in b[1:5]]
        if w > 0.002 and h > 0.002:
            extra = f' {float(b[5]):.3f}' if len(b) > 5 else ''   # confidence (pending files only)
            lines.append(f'{c} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}{extra}')
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write('\n'.join(lines) + ('\n' if lines else ''))
    os.replace(tmp, path)


def _box_ok(b):
    w, h = min(1.0, max(0.0, float(b[3]))), min(1.0, max(0.0, float(b[4])))
    return w > 0.002 and h > 0.002


def clean_poly(poly):
    """[[x, y], ...] in 0..1 (3..1000 points) or None."""
    if poly is None:
        return None
    if not isinstance(poly, (list, tuple)) or not 3 <= len(poly) <= 1000:
        raise ValueError('bad polygon')
    out = []
    for pt in poly:
        if not isinstance(pt, (list, tuple)) or len(pt) != 2:
            raise ValueError('bad polygon point')
        out.append([round(min(1.0, max(0.0, float(pt[0]))), 5), round(min(1.0, max(0.0, float(pt[1]))), 5)])
    return out


def poly_box(poly):
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    return [(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, max(xs) - min(xs), max(ys) - min(ys)]


def shapes_path(label_path):
    """The polygons that go with a label file: labels/<id>.txt -> shapes/<id>.json,
    pending/<id>.txt -> pending/<id>.shapes.json. One entry per line of the label file."""
    d, fn = os.path.split(label_path)
    i = fn[:-4]
    if os.path.basename(d) == 'labels':
        return os.path.join(os.path.dirname(d), 'shapes', i + '.json')
    return os.path.join(d, i + '.shapes.json')


def read_polys(label_path, n):
    data = _read_json(shapes_path(label_path)) or {}
    ps = data.get('polys')
    return ps if isinstance(ps, list) and len(ps) == n else [None] * n


def write_objects(label_path, boxes, polys):
    """Label file + its polygons, filtered together so they stay line for line."""
    keep = [(b, p) for b, p in zip(boxes, polys) if _box_ok(b)]
    write_boxes(label_path, [b for b, _ in keep])
    sp = shapes_path(label_path)
    ps = [p for _, p in keep]
    if any(p is not None for p in ps):
        os.makedirs(os.path.dirname(sp), exist_ok=True)
        _write_json(sp, {'polys': ps})
    elif os.path.exists(sp):
        os.remove(sp)


def mask_polygons(mask, min_area=30):
    """Outlines (normalised [[x, y]...]) of the regions of a binary mask, biggest first."""
    h, w = mask.shape[:2]
    m = (mask > 0).astype(np.uint8)
    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in sorted(contours, key=cv2.contourArea, reverse=True):
        if cv2.contourArea(c) < max(min_area, 0.00002 * w * h):
            continue
        eps = 0.0015 * cv2.arcLength(c, True)
        a = cv2.approxPolyDP(c, max(0.7, eps), True)
        while len(a) > 300:
            eps *= 1.5
            a = cv2.approxPolyDP(c, eps, True)
        if len(a) >= 3:
            out.append([[round(float(x) / w, 5), round(float(y) / h, 5)] for x, y in a[:, 0, :]])
    return out


class Learn:

    def __init__(self, node, sources, root=ROOT):
        """sources: {camera name: colour image topic} of the cameras that can be captured."""
        self.node = node
        self.root = root = os.path.abspath(os.path.expanduser(root))
        if isinstance(sources, str):
            sources = {'realsense': sources}
        self.sources = {n: sources[n] for n in CAM_ORDER if n in sources}
        cfg = _read_json(os.path.join(root, 'capture.json'), {}) or {}
        first = next(iter(self.sources))
        self.selected = [n for n in cfg.get('cams', [first]) if n in self.sources] or [first]
        self.view = cfg.get('view') if cfg.get('view') in self.sources else self.selected[0]
        self._lock = threading.Lock()
        os.makedirs(os.path.join(root, 'datasets'), exist_ok=True)
        if not self.datasets():
            self._make_dataset('main')
        self._subs = {}           # camera -> subscription
        self._frames = {}         # camera -> (msg, time)
        self._want_until = 0.0
        self.detect_watched = False
        self.auto = {'on': False, 'ds': None, 'interval': 2.0, 'saved': 0, 'skipped': 0,
                     'last': 0.0, 'reason': ''}
        self._last_thumbs = {}    # camera -> small grey image of the last saved frame
        self.job = None           # {'type', 'ds', 'run', 'proc', 'started'}
        self.last_job = None
        self._models = {}         # path -> YOLO (suggestions and the live detector)
        self._auto_lock = threading.Lock()
        self._auto_cancel = False
        self.autolabel_state = None
        self._model_lock = threading.Lock()
        self.active = _read_json(os.path.join(root, 'active.json'))
        self.detect = {'state': 'idle', 'error': None, 'conf': 0.4, 'fps': 0.0, 'objects': []}
        atexit.register(self.stop_job)

    # ------------------------------------------------------------ datasets
    def ds_dir(self, ds):
        d = os.path.join(self.root, 'datasets', check_name(ds))
        if not os.path.isdir(d):
            raise ValueError(f'ไม่มีชุดข้อมูล {ds}')
        return d

    def datasets(self):
        base = os.path.join(self.root, 'datasets')
        return sorted(n for n in os.listdir(base)
                      if NAME_RE.match(n) and os.path.isdir(os.path.join(base, n)))

    def _make_dataset(self, name):
        d = os.path.join(self.root, 'datasets', check_name(name))
        if os.path.exists(d):
            raise ValueError(f'มีชุดข้อมูล {name} อยู่แล้ว')
        for sub in ('images', 'labels', 'runs'):
            os.makedirs(os.path.join(d, sub))
        _write_json(os.path.join(d, 'meta.json'), {'classes': [], 'created': time.time()})

    def create_dataset(self, name):
        with self._lock:
            self._make_dataset(name)

    def delete_dataset(self, ds):
        d = self.ds_dir(ds)
        if self.job and self.job['ds'] == ds:
            raise ValueError('กำลัง train ชุดข้อมูลนี้อยู่')
        if self.active and self.active.get('ds') == ds:
            self.set_active(None)
        shutil.rmtree(d)
        if not self.datasets():
            self._make_dataset('main')

    def meta(self, ds):
        return _read_json(os.path.join(self.ds_dir(ds), 'meta.json'), {'classes': []})

    def classes(self, ds):
        return list(self.meta(ds).get('classes', []))

    def edit_classes(self, ds, op):
        """op: {"op": "add", "name"} | {"op": "rename", "i", "name"} | {"op": "delete", "i"}."""
        d = self.ds_dir(ds)
        with self._lock:
            meta = self.meta(ds)
            cls = list(meta.get('classes', []))
            kind = op.get('op')
            if kind in ('add', 'rename'):
                name = str(op.get('name', '')).strip()
                if not name or len(name) > 40 or any(ch in name for ch in ':\n\r\'"'):
                    raise ValueError('ชื่อ class ไม่ถูกต้อง')
                if name in cls:
                    raise ValueError(f'มี class {name} อยู่แล้ว')
                if kind == 'add':
                    cls.append(name)
                else:
                    cls[int(op['i'])] = name
            elif kind == 'delete':
                i = int(op['i'])
                if not 0 <= i < len(cls):
                    raise ValueError('ไม่มี class นี้')
                cls.pop(i)
                for sub in ('labels', 'pending'):
                    lab = os.path.join(d, sub)
                    if not os.path.isdir(lab):
                        continue
                    for fn in os.listdir(lab):
                        if fn.endswith('.txt'):
                            p = os.path.join(lab, fn)
                            boxes = read_boxes(p, conf=True)
                            polys = read_polys(p, len(boxes))
                            keep = [(b, q) for b, q in zip(boxes, polys) if b[0] != i]
                            write_objects(p, [[b[0] - (b[0] > i)] + b[1:] for b, _ in keep],
                                          [q for _, q in keep])
            else:
                raise ValueError('unknown op')
            meta['classes'] = cls
            _write_json(os.path.join(d, 'meta.json'), meta)
        return cls

    def images(self, ds):
        d = self.ds_dir(ds)
        out = []
        for fn in sorted(os.listdir(os.path.join(d, 'images'))):
            if not fn.endswith('.jpg'):
                continue
            i = fn[:-4]
            lp = os.path.join(d, 'labels', i + '.txt')
            labelled = os.path.exists(lp)
            boxes = read_boxes(lp) if labelled else []
            pp = os.path.join(d, 'pending', i + '.txt')
            pending = not labelled and os.path.exists(pp)
            npoly = sum(q is not None for q in read_polys(lp, len(boxes))) if boxes else 0
            out.append({'id': i, 'cam': i.split('_', 1)[1] if '_' in i else 'realsense',
                        'labelled': labelled, 'n': len(boxes), 'npoly': npoly,
                        'cls': sorted({b[0] for b in boxes}), 'val': self._is_val(i),
                        'pending': pending, 'np': len(read_boxes(pp)) if pending else 0})
        return out

    def counts(self, ds):
        imgs = self.images(ds)
        per = {}
        for im in imgs:
            for c in im['cls']:
                per[c] = per.get(c, 0) + 1
        boxes = 0
        d = self.ds_dir(ds)
        per_box = {}
        for im in imgs:
            if im['n']:
                for b in read_boxes(os.path.join(d, 'labels', im['id'] + '.txt')):
                    per_box[b[0]] = per_box.get(b[0], 0) + 1
                    boxes += 1
        return {'images': len(imgs), 'labelled': sum(im['labelled'] for im in imgs),
                'pending': sum(im['pending'] for im in imgs),
                'boxes': boxes, 'per_class_images': per, 'per_class_boxes': per_box}

    @staticmethod
    def _is_val(i):
        return int(hashlib.md5(i.encode()).hexdigest()[:8], 16) % 5 == 0   # 20 % for checking

    def image_path(self, ds, i):
        return os.path.join(self.ds_dir(ds), 'images', check_id(i) + '.jpg')

    def thumb(self, ds, i, width=200):
        img = cv2.imread(self.image_path(ds, i))
        if img is None:
            return None
        h = max(1, round(img.shape[0] * width / img.shape[1]))
        ok, jpg = cv2.imencode('.jpg', cv2.resize(img, (width, h), interpolation=cv2.INTER_AREA),
                               [cv2.IMWRITE_JPEG_QUALITY, 75])
        return jpg.tobytes() if ok else None

    def get_label(self, ds, i):
        d = self.ds_dir(ds)
        p = os.path.join(d, 'labels', check_id(i) + '.txt')
        if os.path.exists(p):
            boxes = read_boxes(p)
            return {'labelled': True, 'boxes': boxes, 'polys': read_polys(p, len(boxes)),
                    'pending': None}
        pp = os.path.join(d, 'pending', i + '.txt')
        if not os.path.exists(pp):
            return {'labelled': False, 'boxes': [], 'polys': [], 'pending': None}
        pend = read_boxes(pp, conf=True)
        return {'labelled': False, 'boxes': [], 'polys': [], 'pending': pend,
                'pending_polys': read_polys(pp, len(pend))}

    def set_label(self, ds, i, boxes, polys=None):
        d = self.ds_dir(ds)
        check_id(i)
        if not os.path.exists(os.path.join(d, 'images', i + '.jpg')):
            raise ValueError('no such image')
        n = len(self.classes(ds))
        boxes = boxes or []
        polys = list(polys) if polys else [None] * len(boxes)
        if len(polys) != len(boxes):
            raise ValueError('polys must match boxes')
        clean, cpolys = [], []
        for b, q in zip(boxes, polys):
            if not isinstance(b, (list, tuple)) or len(b) != 5 or not 0 <= int(b[0]) < n:
                raise ValueError('bad box')
            q = clean_poly(q)
            if q is not None:   # the box of a shape is its outline's bounding box
                b = [int(b[0])] + poly_box(q)
            clean.append(b)
            cpolys.append(q)
        write_objects(os.path.join(d, 'labels', i + '.txt'), clean, cpolys)
        pp = os.path.join(d, 'pending', i + '.txt')
        for f in (pp, shapes_path(pp)):
            if os.path.exists(f):
                os.remove(f)

    def clear_label(self, ds, i):
        p = os.path.join(self.ds_dir(ds), 'labels', check_id(i) + '.txt')
        for f in (p, shapes_path(p)):
            if os.path.exists(f):
                os.remove(f)

    # ------------------------------------------------------------ shape tools
    def wand(self, ds, i, x, y, tol=20):
        """Magic wand: the region of similar colour around pixel (x, y) (0..1), as RGBA PNG bytes
        (alpha = the region), the size of the image."""
        img = cv2.imread(self.image_path(ds, i))
        if img is None:
            raise ValueError('no such image')
        h, w = img.shape[:2]
        px, py = int(min(w - 1, max(0, float(x) * w))), int(min(h - 1, max(0, float(y) * h)))
        tol = int(min(120, max(1, float(tol))))
        smooth = cv2.GaussianBlur(img, (5, 5), 0)
        mask = np.zeros((h + 2, w + 2), np.uint8)
        flags = 4 | cv2.FLOODFILL_MASK_ONLY | cv2.FLOODFILL_FIXED_RANGE | (255 << 8)
        cv2.floodFill(smooth, mask, (px, py), 0, (tol,) * 3, (tol,) * 3, flags)
        m = mask[1:-1, 1:-1]
        k = np.ones((5, 5), np.uint8)
        m = cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_CLOSE, k), cv2.MORPH_OPEN, k)
        rgba = np.zeros((h, w, 4), np.uint8)
        rgba[..., :3] = 255
        rgba[..., 3] = m
        ok, png = cv2.imencode('.png', rgba)
        return png.tobytes(), float(np.count_nonzero(m)) / (w * h)

    @staticmethod
    def mask_to_polys(png_bytes):
        m = cv2.imdecode(np.frombuffer(png_bytes, np.uint8), cv2.IMREAD_UNCHANGED)
        if m is None:
            raise ValueError('bad mask image')
        if m.ndim == 3:
            m = m[..., 3] if m.shape[2] == 4 else cv2.cvtColor(m, cv2.COLOR_BGR2GRAY)
        return mask_polygons(m)

    def delete_images(self, ds, ids):
        d = self.ds_dir(ds)
        n = 0
        for i in ids:
            check_id(i)
            for p in (os.path.join(d, 'images', i + '.jpg'), os.path.join(d, 'labels', i + '.txt'),
                      os.path.join(d, 'shapes', i + '.json'), os.path.join(d, 'pending', i + '.txt'),
                      os.path.join(d, 'pending', i + '.shapes.json')):
                if os.path.exists(p):
                    os.remove(p)
                    n += 1
        return n

    # ------------------------------------------------------------ camera + capture
    def want_camera(self):
        self._want_until = time.monotonic() + WANT_S

    def set_cams(self, cams=None, view=None):
        if cams is not None:
            cams = [n for n in CAM_ORDER if n in cams and n in self.sources]
            if not cams:
                raise ValueError('เลือกกล้องอย่างน้อย 1 ตัว')
            self.selected = cams
        if view is not None:
            if view not in self.sources:
                raise ValueError('ไม่มีกล้องนี้')
            self.view = view
        _write_json(os.path.join(self.root, 'capture.json'), {'cams': self.selected, 'view': self.view})

    def wanted_cams(self):
        """The cameras to subscribe to now: the page open = the chosen ones + the preview one,
        auto capture = the chosen ones, the trained detector watched = the preview one."""
        now = time.monotonic()
        out = set()
        if now < self._want_until:
            out |= set(self.selected) | {self.view}
        if self.auto['on']:
            out |= set(self.selected)
        if self.detect_watched:
            out.add(self.view)
        return out

    @property
    def camera_wanted(self):
        return bool(self.wanted_cams())

    def update_subscription(self):
        """Called a few times a second: subscribe to each colour image only while it is used."""
        wanted = self.wanted_cams()
        node = self.node
        for name in wanted - set(self._subs):
            topic = self.sources[name]
            cb = (lambda msg, n=name: self._on_color(n, msg))
            if node.use_compressed:
                self._subs[name] = node.create_subscription(
                    CompressedImage, topic + '/compressed', cb, qos_profile_sensor_data,
                    callback_group=node.camera_group)
            else:
                self._subs[name] = node.create_subscription(
                    Image, topic, cb, qos_profile_sensor_data, callback_group=node.camera_group)
            node.get_logger().info(f'Learn: subscribed to {topic}')
        for name in set(self._subs) - wanted:
            node.destroy_subscription(self._subs.pop(name))
            self._frames.pop(name, None)
            node.get_logger().info(f'Learn: {name} not needed, unsubscribed')

    def _on_color(self, name, msg):
        self._frames[name] = (msg, time.monotonic())

    def frame_age(self, name=None):
        f = self._frames.get(name or self.view)
        return None if f is None else time.monotonic() - f[1]

    def camera_status(self):
        ages = {}
        for n in self.sources:
            a = self.frame_age(n)
            ages[n] = None if a is None else round(a, 2)
        return {'sources': [{'name': n, 'label': CAM_LABELS.get(n, n)} for n in self.sources],
                'selected': self.selected, 'view': self.view, 'ages': ages,
                'age': ages.get(self.view)}

    def latest_bgr(self, name=None):
        f = self._frames.get(name or self.view)
        if f is None or time.monotonic() - f[1] > FRESH_S:
            return None, None
        msg = f[0]
        if isinstance(msg, CompressedImage):
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR), msg
        from .web_server import decode_image
        return decode_image(msg, self.node.depth_max_mm), msg

    def capture(self, ds, check=False, cams=None):
        """Save the newest frame of each chosen camera, all under the same time stamp
        (<time>_<camera>). check: skip blurry / nearly identical frames (auto capture).
        Returns ([ids saved], {camera: why not})."""
        d = self.ds_dir(ds)
        stamp = time.strftime('%Y%m%d-%H%M%S') + f'-{int(time.time() * 1000) % 1000:03d}'
        saved, why = [], {}
        for cam in cams or self.selected:
            img, msg = self.latest_bgr(cam)
            if img is None:
                why[cam] = 'ยังไม่มีภาพ'
                continue
            small = cv2.cvtColor(cv2.resize(img, (160, 120), interpolation=cv2.INTER_AREA),
                                 cv2.COLOR_BGR2GRAY)
            last = self._last_thumbs.get(cam)
            if check:
                if cv2.Laplacian(small, cv2.CV_64F).var() < BLUR_MIN * 0.25:
                    why[cam] = 'ภาพเบลอ'
                    continue
                if last is not None and float(np.mean(cv2.absdiff(small, last))) < SAME_MAX:
                    why[cam] = 'ภาพเหมือนภาพก่อน'
                    continue
            self._last_thumbs[cam] = small
            i = f'{stamp}_{cam}'
            fmt = getattr(msg, 'format', '').lower()
            if isinstance(msg, CompressedImage) and ('jpeg' in fmt or 'jpg' in fmt):
                data = bytes(msg.data)
            else:
                ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if not ok:
                    why[cam] = 'บันทึกภาพไม่ได้'
                    continue
                data = jpg.tobytes()
            with open(os.path.join(d, 'images', i + '.jpg'), 'wb') as f:
                f.write(data)
            saved.append(i)
        return saved, why

    def set_auto(self, on, ds=None, interval=None):
        if on:
            self.ds_dir(ds)
            self.auto.update(on=True, ds=ds, saved=0, skipped=0, last=0.0, reason='',
                             interval=max(AUTO_MIN_S, float(interval or self.auto['interval'])))
            self._last_thumbs = {}
        else:
            self.auto['on'] = False

    def tick_auto(self):
        a = self.auto
        if not a['on'] or time.monotonic() - a['last'] < a['interval']:
            return
        a['last'] = time.monotonic()
        try:
            ids, why = self.capture(a['ds'], check=True)
        except (ValueError, OSError) as e:
            a.update(on=False, reason=str(e))
            return
        a['saved'] += len(ids)
        a['skipped'] += len(why)
        a['reason'] = ', '.join(f'{CAM_LABELS.get(c, c)}: {w}' for c, w in why.items())

    # ------------------------------------------------------------ models
    def _model(self, path):
        with self._model_lock:
            m = self._models.get(path)
            if m is None:
                os.environ['YOLO_OFFLINE'] = 'true'
                from ultralytics import YOLO
                m = YOLO(path)   # the task (detect / segment) is stored in the file
                if len(self._models) >= 3:
                    self._models.pop(next(iter(self._models)))
                self._models[path] = m
            return m

    def helper(self, ds):
        """The model that suggests boxes for ds: {"mode", "run", "path", "own", "why"}.

        mode "auto" (default): the newest training on the dataset that knows at least one of
        its classes (by name: a model from before a class was added or deleted still helps);
        a run name: that training; "coco": the general model. Boxes are matched by class name."""
        mode = self.meta(ds).get('helper', 'auto')
        cls = {c.lower() for c in self.classes(ds)}
        runs = [r for r in self.runs(ds) if r.get('best') and not r['running']
                and r['state'] in ('done', 'stopped')]
        if mode not in ('auto', 'coco'):
            r = next((r for r in runs if r['run'] == mode), None)
            if r:
                return {'mode': mode, 'run': r['run'], 'path': r['best'], 'own': True, 'why': ''}
            mode, why = 'auto', 'โมเดลที่เลือกไว้ถูกลบแล้ว ใช้แบบอัตโนมัติแทน'
        else:
            why = ''
        if mode == 'auto':
            for r in runs:   # newest first
                if cls & {c.lower() for c in (r.get('classes') or [])}:
                    return {'mode': 'auto', 'run': r['run'], 'path': r['best'], 'own': True, 'why': why}
            why = why or 'ยังไม่มีโมเดลที่ train จากชุดนี้ (กด "สอนตัวช่วย")'
        return {'mode': mode, 'run': None, 'path': COCO_MODEL, 'own': False, 'why': why}

    def set_helper(self, ds, mode):
        if mode not in ('auto', 'coco'):
            check_name(mode)
        with self._lock:
            meta = self.meta(ds)
            meta['helper'] = mode
            _write_json(os.path.join(self.ds_dir(ds), 'meta.json'), meta)
        return self.helper(ds)

    def model_for(self, ds):
        h = self.helper(ds)
        return h['path'], h['own']

    def suggest(self, ds, i, conf=0.35, only=None, path=None, own=None):
        """Boxes a model sees in the image: {"boxes": [[c, cx, cy, w, h, conf]], "other": {name: n}}."""
        if path is None:
            path, own = self.model_for(ds)
        img = cv2.imread(self.image_path(ds, i))
        if img is None:
            raise ValueError('no such image')
        cls = self.classes(ds)
        lower = {c.lower(): k for k, c in enumerate(cls)}
        res = self._model(path).predict(img, conf=conf, verbose=False, device=0)[0]
        names = res.names
        only = None if only is None else {int(k) for k in only}
        model_names = {str(n).lower() for n in names.values()}
        wanted = [cls[k] for k in (only if only is not None else range(len(cls))) if 0 <= k < len(cls)]
        known = any(c.lower() in model_names for c in wanted)
        h, w = img.shape[:2]
        boxes, other, polys = [], {}, []
        segs = res.masks.xyn if res.masks is not None else [None] * len(res.boxes)
        for xyxy, c, p, seg in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.cls.cpu().numpy(),
                                   res.boxes.conf.cpu().numpy(), segs):
            name = names[int(c)]
            k = lower.get(str(name).lower())
            x0, y0, x1, y1 = [float(v) for v in xyxy]
            box = [(x0 + x1) / 2 / w, (y0 + y1) / 2 / h, (x1 - x0) / w, (y1 - y0) / h, float(p)]
            if k is None:
                other.setdefault(name, []).append(box)
            elif only is None or k in only:
                boxes.append([k] + box)
                poly = None
                if seg is not None and len(seg) >= 3:
                    step = -(-len(seg) // 150)   # masks come with many points: keep <= 150
                    poly = [[round(float(a), 5), round(float(b), 5)] for a, b in seg[::step]]
                polys.append(poly)
        return {'boxes': boxes, 'polys': polys, 'other': other, 'model': 'own' if own else 'coco',
                'known': known}

    def autolabel(self, ds, conf=0.4, only=None, redo=False):
        """Run the helper over every image that has no label yet; the boxes go to pending/ (not used
        for training) until a person checks them in the labeller. One at a time."""
        if not self._auto_lock.acquire(blocking=False):
            raise ValueError('กำลัง label อัตโนมัติอยู่แล้ว')
        try:
            path, own = self.model_for(ds)
            d = self.ds_dir(ds)
            os.makedirs(os.path.join(d, 'pending'), exist_ok=True)
            todo = [im['id'] for im in self.images(ds)
                    if not im['labelled'] and (redo or not im['pending'])]
            st = self.autolabel_state = {'ds': ds, 'done': 0, 'total': len(todo), 'boxes': 0,
                                         'running': True, 'own': own, 'error': None}
            for i in todo:
                if self._auto_cancel:
                    break
                r = self.suggest(ds, i, conf, only, path, own)
                write_objects(os.path.join(d, 'pending', i + '.txt'), r['boxes'],
                              r.get('polys') or [None] * len(r['boxes']))
                st['done'] += 1
                st['boxes'] += len(r['boxes'])
            return st
        except Exception as e:  # noqa: BLE001 - shown on the page
            self.autolabel_state = dict(self.autolabel_state or {}, error=f'{type(e).__name__}: {e}')
            raise ValueError(str(e))
        finally:
            if self.autolabel_state:
                self.autolabel_state['running'] = False
            self._auto_cancel = False
            self._auto_lock.release()

    def clear_pending(self, ds):
        d = os.path.join(self.ds_dir(ds), 'pending')
        n = 0
        if os.path.isdir(d):
            for fn in os.listdir(d):
                os.remove(os.path.join(d, fn))
                n += 1
        return n

    # ------------------------------------------------------------ runs
    def runs(self, ds):
        base = os.path.join(self.ds_dir(ds), 'runs')
        out = []
        for r in sorted(os.listdir(base), reverse=True):
            rd = os.path.join(base, r)
            info = _read_json(os.path.join(rd, 'info.json'))
            if not info:
                continue
            prog = _read_json(os.path.join(rd, 'progress.json'), {})
            exp = _read_json(os.path.join(rd, 'export.json'), {})
            hist = prog.get('history') or []
            best_pt = os.path.join(rd, 'weights', 'best.pt')
            engine = os.path.join(rd, 'weights', 'best.engine')
            running = bool(self.job and self.job['run'] == r and self.job['ds'] == ds)
            state = prog.get('state', 'unknown')
            if state in ('starting', 'training') and not running:
                state = 'stopped'
            best_map = max((h.get('map50') or 0 for h in hist), default=None)
            out.append({
                'run': r, 'created': info.get('created'), 'size': info.get('size'),
                'imgsz': info.get('imgsz'), 'epochs': info.get('epochs'),
                'n_train': info.get('n_train'), 'n_val': info.get('n_val'),
                'classes': info.get('classes'), 'state': state, 'error': prog.get('error'),
                'epoch': prog.get('epoch', 0), 'history': hist, 'best_map50': best_map,
                'last': hist[-1] if hist else None,
                'best': best_pt if os.path.exists(best_pt) else None,
                'engine': engine if os.path.exists(engine) and exp.get('state') == 'done' else None,
                'export': exp.get('state'), 'export_error': exp.get('error'), 'running': running,
                'quick': bool(info.get('quick')), 'patience': info.get('patience'),
                'task': info.get('task', 'detect'),
                'early_stop': state == 'done' and 0 < prog.get('epoch', 0) < (info.get('epochs') or 0),
            })
        return out

    def delete_run(self, ds, run):
        check_name(run)
        if self.job and self.job['ds'] == ds and self.job['run'] == run:
            raise ValueError('run นี้กำลังทำงานอยู่')
        if self.active and self.active.get('ds') == ds and self.active.get('run') == run:
            self.set_active(None)
        shutil.rmtree(os.path.join(self.ds_dir(ds), 'runs', run))

    # ------------------------------------------------------------ jobs
    def poll_job(self):
        j = self.job
        if j is None or j['proc'].poll() is None:
            return
        self.last_job = {k: j[k] for k in ('type', 'ds', 'run')}
        self.last_job['code'] = j['proc'].returncode
        self.job = None
        self.node.get_logger().info(f'Learn: {j["type"]} {j["ds"]}/{j["run"]} ended '
                                    f'({j["proc"].returncode})')
        if j['type'] == 'export' and j['proc'].returncode == 0 and self.active \
                and self.active.get('ds') == j['ds'] and self.active.get('run') == j['run']:
            self.set_active(j['ds'], j['run'])   # switch to the TensorRT engine

    def _spawn(self, what, ds, run, rd):
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'learn_train.py')
        log = open(os.path.join(rd, f'{what}.log'), 'w')
        env = dict(os.environ, YOLO_OFFLINE='true', PYTHONUNBUFFERED='1')
        proc = subprocess.Popen(['nice', '-n', '10', sys.executable, script, what, rd],
                                stdout=log, stderr=subprocess.STDOUT, env=env,
                                cwd=rd, start_new_session=True)
        log.close()
        info = _read_json(os.path.join(rd, 'info.json'), {})
        self.job = {'type': what, 'ds': ds, 'run': run, 'proc': proc, 'started': time.time(),
                    'quick': bool(info.get('quick')) and what == 'train'}
        self.node.get_logger().info(f'Learn: {what} {ds}/{run} started (pid {proc.pid})')

    def start_train(self, ds, size='s', epochs=50, imgsz=640, batch=16, from_run=None, quick=False,
                    patience=None, task='detect'):
        if task not in ('detect', 'segment'):
            raise ValueError('bad task')
        if self.job:
            raise ValueError('มีงาน train/แปลงโมเดลทำอยู่แล้ว')
        d = self.ds_dir(ds)
        cls = self.classes(ds)
        if quick:   # the labelling helper: short, continues from the newest model with these classes
            size, epochs, imgsz, batch = 's', 25, 640, 8
            prev = [r for r in self.runs(ds) if r.get('best') and r['state'] in ('done', 'stopped')
                    and list(r.get('classes') or []) == cls and r.get('task', 'detect') == task]
            from_run = prev[0]['run'] if prev else None
        if not cls:
            raise ValueError('ยังไม่มี class')
        labelled = [im for im in self.images(ds) if im['labelled']]
        if len(labelled) < MIN_LABELLED:
            raise ValueError(f'ต้อง label อย่างน้อย {MIN_LABELLED} ภาพ (ตอนนี้ {len(labelled)})')
        if not any(im['n'] for im in labelled):
            raise ValueError('ยังไม่มีกรอบวัตถุเลย')
        val = [im for im in labelled if im['val']]
        train = [im for im in labelled if not im['val']]
        if not val:
            val = [train.pop()]
        if not train:
            train = [val.pop()]
        epochs = int(min(300, max(1, int(epochs))))
        imgsz = int(min(1280, max(320, int(imgsz) // 32 * 32)))
        batch = int(min(64, max(1, int(batch))))
        if size not in ('n', 's', 'm'):
            raise ValueError('bad size')

        run = time.strftime('%Y%m%d-%H%M%S')
        rd = os.path.join(d, 'runs', run)
        os.makedirs(rd)
        imgdir = os.path.join(d, 'images')
        n_shapes = 0
        if task == 'segment':
            # YOLO segmentation labels ("class x1 y1 x2 y2 ..."), made for this run next to links to
            # the images (ultralytics finds a label by swapping /images/ for /labels/ in the path).
            # An object labelled with a box only becomes its rectangle.
            simg, slab = os.path.join(rd, 'seg', 'images'), os.path.join(rd, 'seg', 'labels')
            os.makedirs(simg)
            os.makedirs(slab)
            for im in labelled:
                lp = os.path.join(d, 'labels', im['id'] + '.txt')
                boxes = read_boxes(lp)
                lines = []
                for b, q in zip(boxes, read_polys(lp, len(boxes))):
                    if q is None:
                        l, t, r_, bt = b[1] - b[3] / 2, b[2] - b[4] / 2, b[1] + b[3] / 2, b[2] + b[4] / 2
                        q = [[l, t], [r_, t], [r_, bt], [l, bt]]
                    else:
                        n_shapes += 1
                    lines.append(f'{b[0]} ' + ' '.join(f'{x:.5f} {y:.5f}' for x, y in q))
                with open(os.path.join(slab, im['id'] + '.txt'), 'w') as f:
                    f.write('\n'.join(lines) + ('\n' if lines else ''))
                os.symlink(os.path.join(imgdir, im['id'] + '.jpg'), os.path.join(simg, im['id'] + '.jpg'))
            imgdir = simg
        with open(os.path.join(rd, 'train.txt'), 'w') as f:
            f.write(''.join(os.path.join(imgdir, im['id'] + '.jpg') + '\n' for im in train))
        with open(os.path.join(rd, 'val.txt'), 'w') as f:
            f.write(''.join(os.path.join(imgdir, im['id'] + '.jpg') + '\n' for im in val))
        names = '\n'.join(f'  {k}: {json.dumps(c, ensure_ascii=False)}' for k, c in enumerate(cls))
        with open(os.path.join(rd, 'data.yaml'), 'w') as f:
            f.write(f'path: {d}\ntrain: {rd}/train.txt\nval: {rd}/val.txt\nnames:\n{names}\n')
        cache = os.path.join(d, 'labels.cache')   # ultralytics may miss same-size label edits
        if os.path.exists(cache):
            os.remove(cache)

        if from_run:
            prev = os.path.join(d, 'runs', check_name(from_run), 'weights', 'best.pt')
            if not os.path.exists(prev):
                raise ValueError('run ที่เลือกไม่มีโมเดล')
            weights, mode, base = prev, 'file', f'ต่อจาก {from_run}'
        else:
            seg = '-seg' if task == 'segment' else ''
            own = os.path.join(self.root, 'weights', f'yolov8{size}{seg}.pt')
            if os.path.exists(own):
                weights, mode, base = own, 'file', f'yolov8{size}{seg} (COCO)'
            else:   # only the segmentation model is on the car: reuse its backbone (all of it for -seg s)
                weights, mode, base = COCO_MODEL, 'yaml+load', f'yolov8{size}{seg} + yolov8s-seg (COCO)'
        # Stop when the checking images have not improved for this many rounds (0 = never).
        patience = max(10, epochs // 3) if patience is None else int(min(300, max(0, int(patience))))
        job = {'data': os.path.join(rd, 'data.yaml'), 'epochs': epochs, 'imgsz': imgsz,
               'patience': patience,
               'batch': batch, 'weights': weights, 'weights_mode': mode,
               'yaml': f'yolov8{size}{"-seg" if task == "segment" else ""}.yaml'}
        _write_json(os.path.join(rd, 'job.json'), job)
        _write_json(os.path.join(rd, 'info.json'), {
            'created': time.time(), 'size': size, 'imgsz': imgsz, 'epochs': epochs, 'batch': batch,
            'n_train': len(train), 'n_val': len(val), 'classes': cls, 'base': base,
            'quick': bool(quick), 'patience': patience, 'task': task, 'n_shapes': n_shapes})
        self._spawn('train', ds, run, rd)
        return run

    def start_export(self, ds, run):
        if self.job:
            raise ValueError('มีงาน train/แปลงโมเดลทำอยู่แล้ว')
        rd = os.path.join(self.ds_dir(ds), 'runs', check_name(run))
        if not os.path.exists(os.path.join(rd, 'weights', 'best.pt')):
            raise ValueError('run นี้ยังไม่มีโมเดล')
        self._spawn('export', ds, run, rd)

    def stop_job(self, reason=''):
        j = self.job
        if j is None:
            return False
        try:
            os.killpg(j['proc'].pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            j['proc'].wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(j['proc'].pid, signal.SIGKILL)
        if reason:
            self.node.get_logger().warn(f'Learn: stopped {j["type"]} ({reason})')
            prog = os.path.join(self.ds_dir(j['ds']), 'runs', j['run'],
                                'progress.json' if j['type'] == 'train' else 'export.json')
            data = _read_json(prog, {})
            data.update(state='stopped', error=reason)
            _write_json(prog, data)
        self.poll_job()
        return True

    def job_status(self):
        j = self.job
        if j is None:
            return None
        rd = os.path.join(self.root, 'datasets', j['ds'], 'runs', j['run'])
        prog = _read_json(os.path.join(rd, 'progress.json' if j['type'] == 'train'
                                       else 'export.json'), {})
        log = ''
        try:
            with open(os.path.join(rd, f'{j["type"]}.log'), 'rb') as f:
                f.seek(0, 2)
                f.seek(max(0, f.tell() - 3000))
                log = f.read().decode('utf-8', 'replace')
        except OSError:
            pass
        log = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', log).replace('\r', '\n')
        lines = [ln for ln in log.split('\n') if ln.strip()][-8:]
        return {'type': j['type'], 'ds': j['ds'], 'run': j['run'], 'quick': j.get('quick', False),
                'elapsed': time.time() - j['started'], 'state': prog.get('state', 'starting'),
                'epoch': prog.get('epoch', 0), 'epochs': prog.get('epochs'),
                'history': prog.get('history', []), 'log': lines}

    # ------------------------------------------------------------ live detector
    def set_active(self, ds, run=None):
        if ds is None:
            self.active = None
            p = os.path.join(self.root, 'active.json')
            if os.path.exists(p):
                os.remove(p)
            return None
        rd = os.path.join(self.ds_dir(ds), 'runs', check_name(run))
        info = _read_json(os.path.join(rd, 'info.json'), {})
        engine = os.path.join(rd, 'weights', 'best.engine')
        exp = _read_json(os.path.join(rd, 'export.json'), {})
        path = engine if os.path.exists(engine) and exp.get('state') == 'done' \
            else os.path.join(rd, 'weights', 'best.pt')
        if not os.path.exists(path):
            raise ValueError('run นี้ยังไม่มีโมเดล')
        self.active = {'ds': ds, 'run': run, 'path': path, 'classes': info.get('classes', []),
                       'imgsz': info.get('imgsz', 640), 'engine': path.endswith('.engine')}
        _write_json(os.path.join(self.root, 'active.json'), self.active)
        self.detect.update(state='idle', error=None)
        return self.active

    def run_detector(self):
        """One frame of the live detector: (BGR image with boxes, objects) or (None, None)."""
        a = self.active
        if not a:
            self.detect.update(state='none', objects=[])
            return None, None
        img, _ = self.latest_bgr(self.view)
        if img is None:
            return None, None
        try:
            if a['path'] not in self._models:
                self.detect['state'] = 'loading'
            model = self._model(a['path'])
            t0 = time.monotonic()
            kw = {'imgsz': a.get('imgsz', 640)} if a.get('engine') else {}
            res = model.predict(img, conf=self.detect['conf'], verbose=False, device=0, **kw)[0]
            dt = time.monotonic() - t0
        except Exception as e:  # noqa: BLE001 - shown on the page, never stops the node
            self.detect.update(state='error', error=f'{type(e).__name__}: {e}')
            return None, None
        objs, outlines = [], []
        segs = res.masks.xy if res.masks is not None else [None] * len(res.boxes)
        for xyxy, c, p, seg in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.cls.cpu().numpy(),
                                   res.boxes.conf.cpu().numpy(), segs):
            objs.append({'cls': int(c), 'name': str(res.names[int(c)]), 'conf': round(float(p), 3),
                         'box': [round(float(v), 1) for v in xyxy]})
            outlines.append(seg)
        self.detect.update(state='ready', error=None, objects=objs,
                           fps=round(0.8 * self.detect['fps'] + 0.2 / max(dt, 1e-3), 1))
        return draw_boxes(img, objs, outlines), objs


PALETTE = [(56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255), (49, 210, 207),
           (10, 249, 72), (23, 204, 146), (134, 219, 61), (52, 147, 26), (187, 212, 0),
           (168, 153, 44), (255, 194, 0), (147, 69, 52), (255, 115, 100), (236, 24, 0)]


def draw_boxes(img, objs, outlines=None):
    out = img.copy()
    if outlines and any(s is not None and len(s) >= 3 for s in outlines):
        fill = out.copy()
        for o, seg in zip(objs, outlines):
            if seg is not None and len(seg) >= 3:
                cv2.fillPoly(fill, [np.asarray(seg, np.int32)], PALETTE[o['cls'] % len(PALETTE)])
        out = cv2.addWeighted(fill, 0.4, out, 0.6, 0)
    for o in objs:
        x0, y0, x1, y1 = [int(v) for v in o['box']]
        col = PALETTE[o['cls'] % len(PALETTE)]
        cv2.rectangle(out, (x0, y0), (x1, y1), col, 2)
        # cv2's font has no Thai: the page lists the names; the image shows the class number
        text = f'{o["cls"]} {o["conf"]:.2f}'
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(out, (x0, max(0, y0 - th - 6)), (x0 + tw + 6, y0), col, -1)
        cv2.putText(out, text, (x0 + 3, y0 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                    cv2.LINE_AA)
    return out
