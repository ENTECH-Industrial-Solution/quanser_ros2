"""Car-level controls for demos: the PCB user buttons, Wi-Fi hotspot/office, ROS/MATLAB mode
and a status line on the PCB LCD.

- Buttons come from qcar2_hardware's qcar2_user_buttons topic (bits 0..2 = S2..S4, 1 =
  pressed) while the ROS nodes run. In MATLAB mode nobody owns the HIL card, so HilButtons
  reads them directly (open, read, close each time, never while MATLAB is connected).
- Wi-Fi and mode changes run system/qcar-wifi and system/qcar-mode (installed by
  system/install.sh) in a background thread; they take several seconds.
- The LCD belongs to quarc_power_monitor (battery, CPU, Wi-Fi, IP): only its bottom two text
  lines are written, and nothing at all in MATLAB mode, where a model may use the display.
"""

import json
import os
import re
import subprocess
import tempfile
import threading
import time

WIFI_CMD = '/usr/local/sbin/qcar-wifi'
MODE_CMD = '/usr/local/sbin/qcar-mode'
NAV_CMD = '/usr/local/sbin/qcar-nav'
MAP_NAME_RE = re.compile(r'^[A-Za-z0-9_.-]+$')
MAP_EXTENSIONS = ('.yaml', '.pgm', '.pbstream', '.db')
ROBOT_SERVICE = 'qcar2-robot.service'
HOTSPOT_CONNECTION = 'qcar-demo'
ACTION_TIMEOUT_S = 120.0   # qcar-wifi office may search for a network ~40 s + connect
BUTTONS_STALE_S = 3.0   # qcar2_hardware resends the state every second
LCD_LINES = (10, 11)    # of 12 (0..11), 25 characters each
LCD_WIDTH = 25

ACTIONS = {
    'wifi_demo': ('เปิด hotspot', [WIFI_CMD, 'demo']),
    'wifi_office': ('ต่อ Wi-Fi ปกติ', [WIFI_CMD, 'office']),
    'wifi_toggle': ('สลับ Wi-Fi', [WIFI_CMD, 'toggle']),
    'matlab': ('ปล่อยฮาร์ดแวร์ให้ MATLAB', [MODE_CMD, 'matlab']),
    'ros': ('เริ่ม ROS', [MODE_CMD, 'ros']),
    'nav_slam': ('เริ่มทำแผนที่ (Lidar)', [NAV_CMD, 'slam']),
    'nav_vslam': ('เริ่มทำแผนที่ (V-SLAM)', [NAV_CMD, 'vslam']),
    'nav_map': ('โหลดแผนที่และเริ่มนำทาง (Lidar)', [NAV_CMD, 'nav']),      # + the map name
    'nav_vmap': ('โหลดแผนที่และเริ่มนำทาง (V-SLAM)', [NAV_CMD, 'vnav']),  # + the map name
    'nav_stop': ('ปิดทำแผนที่/นำทาง', [NAV_CMD, 'stop']),
}
# The same, for the LCD (ASCII), keyed by the label above.
ACTIONS_LCD = {
    'เปิด hotspot': 'Starting hotspot...',
    'ต่อ Wi-Fi ปกติ': 'Joining Wi-Fi...',
    'สลับ Wi-Fi': 'Switching Wi-Fi...',
    'ปล่อยฮาร์ดแวร์ให้ MATLAB': 'Stopping ROS...',
    'เริ่ม ROS': 'Starting ROS...',
    'เริ่มทำแผนที่ (Lidar)': 'Starting SLAM...',
    'เริ่มทำแผนที่ (V-SLAM)': 'Starting V-SLAM...',
    'โหลดแผนที่และเริ่มนำทาง (Lidar)': 'Starting Nav2...',
    'โหลดแผนที่และเริ่มนำทาง (V-SLAM)': 'Starting V-Nav2...',
    'ปิดทำแผนที่/นำทาง': 'Stopping SLAM/Nav2...',
}
MAP_ACTIONS = ('nav_map', 'nav_vmap')   # actions that take a map name


def list_maps(map_dir):
    """Names of the saved maps (<map_dir>/<name>.yaml), newest first."""
    directory = os.path.expanduser(map_dir)
    try:
        files = [f for f in os.listdir(directory) if f.endswith('.yaml')]
    except OSError:
        return []
    files.sort(key=lambda f: os.path.getmtime(os.path.join(directory, f)), reverse=True)
    return [f[:-5] for f in files if MAP_NAME_RE.match(f[:-5])]


def _pgm_size(path):
    """(width, height) in cells from a binary PGM header, or None."""
    try:
        with open(path, 'rb') as f:
            head = f.read(64).split()
        if head[0] == b'P5':
            return int(head[1]), int(head[2])
    except (OSError, ValueError, IndexError):
        pass
    return None


def _yaml_resolution(path):
    try:
        with open(path) as f:
            m = re.search(r'^resolution:\s*([0-9.eE+-]+)', f.read(), re.M)
        return float(m.group(1)) if m else None
    except (OSError, ValueError):
        return None


def map_info(map_dir):
    """Saved maps, newest first: name, time, size in metres, and which localisation they support.

    lidar: AMCL can use the grid (every map). vslam: a RTAB-Map database was saved with it.
    """
    directory = os.path.expanduser(map_dir)
    out = []
    for name in list_maps(map_dir):
        base = os.path.join(directory, name)
        size = _pgm_size(base + '.pgm')
        res = _yaml_resolution(base + '.yaml')
        out.append({
            'name': name,
            'time': os.path.getmtime(base + '.yaml'),
            'width_m': round(size[0] * res, 1) if size and res else None,
            'height_m': round(size[1] * res, 1) if size and res else None,
            'lidar': os.path.isfile(base + '.pgm'),
            'vslam': os.path.isfile(base + '.db'),
            'pbstream': os.path.isfile(base + '.pbstream'),
            'bytes': sum(os.path.getsize(base + e) for e in MAP_EXTENSIONS if os.path.isfile(base + e)),
        })
    return out


def delete_map(map_dir, name):
    """Remove every file of a saved map. Returns the file names removed."""
    if not MAP_NAME_RE.match(name or ''):
        raise ValueError('ชื่อแผนที่ไม่ถูกต้อง')
    base = os.path.join(os.path.expanduser(map_dir), name)
    if not os.path.isfile(base + '.yaml'):
        raise ValueError('ไม่พบแผนที่นี้')
    removed = []
    for ext in MAP_EXTENSIONS:
        if os.path.isfile(base + ext):
            os.remove(base + ext)
            removed.append(name + ext)
    return removed


def nav_mode():
    """(mode, method, map name) from the systemd units.

    mode: 'slam' (mapping), 'nav' (navigating) or 'off'; method: 'lidar' (cartographer / AMCL)
    or 'vslam' (RTAB-Map with the camera), None when off.
    """
    out = _run(['systemctl', 'list-units', '--plain', '--no-legend', '--state=active,activating',
                'qcar2-slam.service', 'qcar2-vslam.service', 'qcar2-nav@*.service',
                'qcar2-vnav@*.service'])
    for line in out.splitlines():
        unit = line.split()[0] if line.split() else ''
        if unit == 'qcar2-slam.service':
            return 'slam', 'lidar', None
        if unit == 'qcar2-vslam.service':
            return 'slam', 'vslam', None
        for prefix, method in (('qcar2-nav@', 'lidar'), ('qcar2-vnav@', 'vslam')):
            if unit.startswith(prefix):
                name = _run(['systemd-escape', '--unescape', unit[len(prefix):-len('.service')]])
                return 'nav', method, name.strip()
    return 'off', None, None


def _run(cmd, timeout=5.0):
    """stdout of cmd, or '' if it fails."""
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError):
        return ''


class SystemControl:
    """Wi-Fi and ROS/MATLAB mode: current state (refresh()) and changes (start())."""

    def __init__(self, logger):
        self.log = logger
        self._lock = threading.Lock()
        self.busy = None        # label of the action running
        self.message = None     # {'text', 'ok'} of the last action
        self.state = {'wifi': None, 'ssid': None, 'ip': None, 'robot': None,
                      'nav_mode': 'off', 'nav_method': None, 'nav_map': None}

    def refresh(self):
        """Read the Wi-Fi and service state (blocks for a few ms; call from a timer)."""
        active = _run(['nmcli', '-t', '-f', 'NAME,DEVICE', 'con', 'show', '--active'])
        ssid = None
        for line in active.splitlines():
            name, _, dev = line.rpartition(':')
            if dev == 'wlan0':
                ssid = name
        ip = None
        for word in _run(['ip', '-4', '-br', 'addr', 'show', 'wlan0']).split():
            if '/' in word:
                ip = word.split('/')[0]
        robot = _run(['systemctl', 'is-active', ROBOT_SERVICE]).strip() or None
        mode, method, map_name = nav_mode()
        state = {
            'nav_mode': mode,
            'nav_method': method,
            'nav_map': map_name,
            'wifi': 'demo' if ssid == HOTSPOT_CONNECTION else 'office',
            'ssid': 'QCar2-Demo' if ssid == HOTSPOT_CONNECTION else ssid,
            'ip': ip,
            'robot': robot,
        }
        with self._lock:
            self.state = state

    def status(self):
        with self._lock:
            return dict(self.state, busy=self.busy, message=self.message)

    def start(self, action, arg=None):
        """Run one of ACTIONS in the background (MAP_ACTIONS: arg = map name). False if unknown,
        invalid or another one is running."""
        if action not in ACTIONS:
            return False
        if action in MAP_ACTIONS and not (isinstance(arg, str) and MAP_NAME_RE.match(arg)):
            return False
        with self._lock:
            if self.busy:
                return False
            self.busy = ACTIONS[action][0]
        threading.Thread(target=self._run_action, args=(action, arg), daemon=True).start()
        return True

    def _run_action(self, action, arg=None):
        label, cmd = ACTIONS[action]
        if action in MAP_ACTIONS:
            label, cmd = f'{label} {arg}', cmd + [arg]
        ok, text = False, f'{label} ไม่สำเร็จ'
        try:
            self.log.info(f'System: {label} ({" ".join(cmd)})')
            try:
                res = subprocess.run(cmd, capture_output=True, text=True, timeout=ACTION_TIMEOUT_S)
                ok = res.returncode == 0
                detail = (res.stderr or res.stdout).strip().splitlines()
                text = label + (' สำเร็จ' if ok else ' ไม่สำเร็จ: ' + (detail[-1] if detail else ''))
            except (OSError, subprocess.SubprocessError) as e:
                ok, text = False, f'{label} ไม่สำเร็จ: {e}'
            # Separate call sites: rclpy raises if one line logs at two different severities.
            if ok:
                self.log.info(f'System: {text}')
            else:
                self.log.error(f'System: {text}')
            self.refresh()
        except Exception as e:  # never leave the panel stuck "busy"
            try:
                self.log.error(f'System: {label}: {e!r}')
            except Exception:
                pass
        finally:
            with self._lock:
                self.busy = None
                self.message = {'text': text, 'ok': ok}


class Buttons:
    """Turns qcar2_user_buttons messages into actions.

    actions: {bit: (hold_s, callback)}. hold_s 0 fires on press; otherwise once the button
    has been held that long (one call per press).

    A button only counts once it has been seen released, so a button that reads as pressed
    from the start (stuck, or a node with the wrong polarity) never acts on its own.

    A release shorter than RELEASE_DEBOUNCE_S while holding is contact bounce (or a thumb
    that slipped) and does not restart the hold time.
    """

    RELEASE_DEBOUNCE_S = 0.25

    def __init__(self, actions):
        self.actions = actions
        self._lock = threading.Lock()
        self._pressed_at = {}   # bit -> time pressed, or None once fired
        self._released_at = {}  # bit -> time it read released while in _pressed_at
        self._armed = set()     # bits seen released since the last (re)start
        self._last_msg = 0.0

    def holding(self):
        """{bit: seconds held} for hold buttons pressed but not fired yet."""
        now = time.monotonic()
        with self._lock:
            return {bit: now - since for bit, since in self._pressed_at.items()
                    if since is not None and bit not in self._released_at}

    def update(self, mask):
        now = time.monotonic()
        fire = []
        with self._lock:
            self._last_msg = now
            for bit, (hold_s, callback) in self.actions.items():
                down = bool(mask & (1 << bit))
                if not down:
                    if bit in self._pressed_at:
                        self._released_at.setdefault(bit, now)   # final in poll()
                    self._armed.add(bit)
                elif bit in self._released_at:
                    del self._released_at[bit]   # bounced: still the same press
                elif bit in self._armed and bit not in self._pressed_at:
                    if hold_s <= 0:
                        self._pressed_at[bit] = None
                        fire.append(callback)
                    else:
                        self._pressed_at[bit] = now
        for callback in fire:
            callback()

    def poll(self):
        """Call periodically: fires held buttons."""
        now = time.monotonic()
        fire = []
        with self._lock:
            if now - self._last_msg > BUTTONS_STALE_S:
                self._pressed_at.clear()   # hardware node stopped: forget half-held buttons
                self._released_at.clear()
                self._armed.clear()
                return
            for bit, at in list(self._released_at.items()):
                if now - at >= self.RELEASE_DEBOUNCE_S:
                    del self._released_at[bit]
                    self._pressed_at.pop(bit, None)
            for bit, since in self._pressed_at.items():
                if (since is not None and bit not in self._released_at
                        and now - since >= self.actions[bit][0]):
                    self._pressed_at[bit] = None
                    fire.append(self.actions[bit][1])
        for callback in fire:
            callback()


class HilButtons:
    """Reads S2..S4 straight from the HIL card, for MATLAB mode (no qcar2_hardware).

    The card is exclusive: while it is open here a QUARC model cannot open it. So it is only
    held for one read (~25 ms), and not at all while a MATLAB host is connected to the QUARC
    target manager (deploying or running a model) or after the card was found busy.
    """

    CHANNELS = (11, 12, 13)      # user buttons S2, S3, S4; 0 = pressed
    QUARC_PORT = 17000           # quarc_target_manager
    BUSY_BACKOFF_S = 5.0

    def __init__(self, logger):
        self.log = logger
        self._busy_until = 0.0

    @classmethod
    def matlab_connected(cls):
        """True if a TCP connection to the QUARC target manager is established."""
        port = f':{cls.QUARC_PORT:04X}'
        for path in ('/proc/net/tcp', '/proc/net/tcp6'):
            try:
                with open(path) as f:
                    next(f)
                    for line in f:
                        fields = line.split()
                        if fields[1].endswith(port) and fields[3] == '01':
                            return True
            except (OSError, StopIteration):
                pass
        return False

    def read(self):
        """Button mask (bit 0 = S2, 1 = pressed), or None if the card may not be used now."""
        now = time.monotonic()
        if now < self._busy_until or self.matlab_connected():
            return None
        try:
            import numpy as np
            from quanser.hardware import HIL
            card = HIL()
            card.open('qcar2', '0')
        except Exception as e:   # in use by a QUARC model, or no QUARC libraries
            self._busy_until = now + self.BUSY_BACKOFF_S
            self.log.info(f'Buttons: HIL card not available ({e})', throttle_duration_sec=60.0)
            return None
        try:
            buf = np.zeros(len(self.CHANNELS), dtype=np.int8)
            card.read_digital(np.array(self.CHANNELS, dtype=np.uint32), len(self.CHANNELS), buf)
        except Exception as e:
            self.log.warn(f'Buttons: HIL read failed: {e}', throttle_duration_sec=60.0)
            return None
        finally:
            card.close()
        return sum(1 << i for i, v in enumerate(buf) if not v)


class EcoSensors:
    """Starts and stops the camera services (qcar2-csi, qcar2-realsense).

    Eco off: both run whenever the ROS robot nodes do (as before). Eco on: a camera only runs
    while something uses it (update()'s `wanted`); it starts at once when wanted and stops
    after IDLE_OFF_S without use, so reloading the page does not restart it. The robot nodes
    (hardware, lidar, odometry) and everything that is used keep their full performance.
    The setting is saved in `path` and survives restarts.
    """

    UNITS = {'csi': 'qcar2-csi.service', 'realsense': 'qcar2-realsense.service'}
    IDLE_OFF_S = 30.0

    def __init__(self, logger, path):
        self.log = logger
        self.path = os.path.expanduser(path)
        self._lock = threading.Lock()
        self.eco = False
        try:
            with open(self.path) as f:
                self.eco = bool(json.load(f).get('eco', False))
        except (OSError, ValueError, AttributeError):
            pass
        self.state = {name: None for name in self.UNITS}   # systemd ActiveState
        self._last_wanted = {name: 0.0 for name in self.UNITS}
        self._pending = set()   # units with a start/stop running
        self._retry_at = {name: 0.0 for name in self.UNITS}   # after a failed start

    def set_eco(self, on):
        on = bool(on)
        with self._lock:
            self.eco = on
        try:
            os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
            tmp = self.path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'eco': on}, f)
            os.replace(tmp, self.path)
        except OSError as e:
            self.log.warn(f'Eco: cannot save the setting: {e}')
        self.log.info(f'Eco mode {"on" if on else "off"}')

    def update(self, busy, wanted):
        """Call periodically. wanted: {'csi': bool, 'realsense': bool} = in use right now;
        busy: a system action (ROS on/off, Wi-Fi) is running.

        The robot state is read here, fresh, not from SystemControl's 2 s cache: starting a
        camera while qcar2-robot is stopping cancels that stop job (the cameras require the
        robot unit), which made "stop ROS / MATLAB mode" fail.
        """
        out = _run(['systemctl', 'is-active', ROBOT_SERVICE] + list(self.UNITS.values()))
        states = out.split()
        robot_state, states = (states[0], states[1:]) if states else (None, [])
        robot_active = robot_state == 'active' and not busy
        now = time.monotonic()
        with self._lock:
            eco = self.eco
            for name, st in zip(self.UNITS, states):
                self.state[name] = st
        for name, unit in self.UNITS.items():
            running = self.state[name] in ('active', 'activating', 'reloading')
            need = robot_active and (not eco or wanted.get(name, False))
            if need:
                self._last_wanted[name] = now
                if not running and now >= self._retry_at[name]:
                    self._systemctl('start', name)
            elif running and robot_state == 'active' and not busy and (
                    not eco or now - self._last_wanted[name] >= self.IDLE_OFF_S):
                # Eco: unused for a while. (With the robot stopping or off, the cameras stop
                # with qcar2-robot anyway: leave them to systemd.)
                self._systemctl('stop', name)

    def status(self):
        with self._lock:
            return {'eco': self.eco, **{name: self.state[name] for name in self.UNITS}}

    def _systemctl(self, verb, name):
        with self._lock:
            if name in self._pending:
                return
            self._pending.add(name)
        unit = self.UNITS[name]

        def run():
            try:
                res = subprocess.run(['sudo', '-n', 'systemctl', verb, unit],
                                     capture_output=True, text=True, timeout=30.0)
                if res.returncode != 0:
                    if verb == 'start':
                        self._retry_at[name] = time.monotonic() + 10.0
                    self.log.warn(f'Eco: systemctl {verb} {unit} failed: {res.stderr.strip()}',
                                  throttle_duration_sec=30.0)
                else:
                    self.log.info(f'Eco: {verb} {unit}')
            except (OSError, subprocess.SubprocessError) as e:
                self.log.warn(f'Eco: systemctl {verb} {unit}: {e}', throttle_duration_sec=30.0)
            finally:
                with self._lock:
                    self._pending.discard(name)

        threading.Thread(target=run, daemon=True).start()


class LcdStatus:
    """Writes LCD_LINES of the PCB LCD, sharing it with quarc_power_monitor."""

    def __init__(self, logger, rotate_180=False):
        self.log = logger
        self.rotate_180 = rotate_180    # our two lines upside down, in the same place
        self._display = None
        self._shown = None
        self._failed_at = None
        self._lock = threading.Lock()   # shown from the panel timer and the button callbacks
        fd, self._scratch = tempfile.mkstemp(prefix='qcar2_web_lcd_', suffix='.pbm')
        os.close(fd)

    def show(self, lines):
        with self._lock:
            self._show(lines)

    def _show(self, lines):
        lines = [str(t)[:LCD_WIDTH].ljust(LCD_WIDTH) for t in lines]
        if lines == self._shown:
            return
        if not self._open():
            return
        try:
            self._display.beginDraw()
            for row, text in zip(LCD_LINES, lines):
                self._display.printText(row, 0, text, len(text))
            if self.rotate_180:
                self._rotate_lines()
            self._display.endDraw()
            self._shown = lines
        except Exception as e:   # quanser.devices.DeviceError and friends
            self.log.warn(f'LCD: {e}', throttle_duration_sec=60.0)
            self.close()

    def _rotate_lines(self):
        """Turn the area of LCD_LINES 180 degrees (rendered with the display's own font).

        Rotating the whole display (setRotation) would also turn quarc_power_monitor's
        lines, so the text is printed normally, read back from this handle's buffer (save)
        and drawn over itself upside down. Line order flips too, as it should.
        """
        import cv2
        import numpy as np
        self._display.save(self._scratch)
        img = cv2.imread(self._scratch, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return
        line_h = img.shape[0] // 12
        top, bottom = LCD_LINES[0] * line_h, (LCD_LINES[-1] + 1) * line_h
        band = np.ascontiguousarray(img[top:bottom, :][::-1, ::-1])
        col_major = band.transpose().copy()
        self._display.drawImage(top, 0, col_major.shape, col_major)

    def forget(self):
        """Redraw on the next show() even if the text did not change."""
        self._shown = None

    def _open(self):
        if self._display is not None:
            return True
        if self._failed_at is not None and time.monotonic() - self._failed_at < 30.0:
            return False
        try:
            from quanser.devices import LCDAccess, LS027B7DSH01Display
            display = LS027B7DSH01Display()
            display.open(access=LCDAccess.SHARED)
        except Exception as e:
            self._failed_at = time.monotonic()
            self.log.warn(f'LCD not available: {e}', throttle_duration_sec=300.0)
            return False
        self._display = display
        self._failed_at = None
        return True

    def close(self):
        display, self._display, self._shown = self._display, None, None
        if display is not None:
            try:
                display.close()
            except Exception:
                pass
