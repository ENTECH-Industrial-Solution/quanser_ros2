# robot_web_ui

The shared web UI of our robots: the look (theme), the page layout (pages, windows, drag bars,
phone dock), camera streaming, the Thai/English switch, and the server-side helpers.
`qcar2_web` is built on it; a new robot starts from the template and the example node here.

```
static/            served at /ui/ by every robot's web node
  theme.css        all colours + font (CSS variables) — override in the robot's CSS to re-brand
  ui.css           layout and components (header, chips, buttons, windows, control panel, dock, ...)
  core.js          window.RobotUI: shell, windows, drag bars, pages, camera streams, inputs
  i18n.js          Thai -> English switch (header button), dictionaries below
  ui_en.js         robot_web_ui's own dictionary (built from i18n_en.tsv)
robot_web_ui/
  server.py        CameraStream, /ws/stream + /stream, /ui/ + /static/ routes, password, serve()
  example_server.py  a complete small robot node (cameras + geometry_msgs/Twist driving)
template/
  index.html       the example node's page = the starting point of a robot's page
i18n_build.py      .tsv -> dictionary .js (core's and robots')
```

## Try it

```bash
colcon build --packages-select robot_web_ui     # humble only, see the workspace README
source install/setup.bash
ros2 run robot_web_ui example_server --ros-args \
  -p robot_name:=MyBot -p port:=8080 -p cmd_vel_topic:=/cmd_vel \
  -p cameras:="['front=/camera/color/image_raw', 'depth=/camera/depth/image_rect_raw:depth']"
```

Open `http://<robot-ip>:8080`. Parameters: `robot_name`, `host`, `port`, `password` (HTTP Basic,
'' = none), `cameras` (`name=topic`, `:depth` for 16-bit depth images), `cmd_vel_topic`,
`max_linear` (m/s), `max_angular` (rad/s), `command_timeout` (s), `use_compressed`, `recompress`,
`max_fps`, `jpeg_quality`, `stream_max_width`, `depth_max_mm`, `static_dir` (a page other than the
template).

## A new robot

1. Make the package `src/<robot>/<robot>_web` (ament_python) with `<exec_depend>robot_web_ui</exec_depend>`.
2. Copy `robot_web_ui/example_server.py` -> `<robot>_web/web_server.py` and
   `template/index.html` -> `<robot>_web/static/index.html`; install `static/*` in `setup.py`
   (see `qcar2_web/setup.py`) and point `static_dir` at it.
3. In `index.html` set `<html data-store="<robot>">` (keeps its saved layout apart from other
   robots on the same browser) and the name in `<h1>`.
4. Driving: the template sends `{type: 'drive', linear, angular}` (-1..1) 20 times a second; the
   node turns it into a Twist. A robot with other commands (steering, arms, ...) changes the drive
   loop at the end of the page and `Drive` / `ws_handler` in its node. **Keep the watchdog and
   E-STOP**: the page's messages are the heartbeat, no message for `command_timeout` = stop.
5. Add pages and windows (below), the robot's CSS in a `<style>` or `static/<robot>.css` after
   `/ui/ui.css`, its APIs as routes in `make_app` (before `add_ui_routes(app)`).
6. English: put the robot's phrases in `<robot>_web/i18n_en.tsv`, build with
   `python3 <path>/robot_web_ui/i18n_build.py i18n_en.tsv static/i18n_en.js RUI_EN` and load
   `/static/i18n_en.js` before `/ui/i18n.js`.

## Page structure

Everything is in Thai; i18n.js translates it in English mode. Elements by id are looked up by
core.js, so keep the ids.

```
header
  h1                              robot name
  nav#page-tabs > button.ptab[data-page=X]     one per page (+ button#win-reset)
  span.chip#chip-conn, #chip-ctrl, ...         status chips (setChip(id, text, 'ok'|'warn'|'bad'))
main
  section                         the pages: every block carries data-pg="X" (or "X Y")
    .cam (window) > .cam-head + .view          a camera/drawing window
    div#grid.splitview[data-pg=cams]           split-screen camera page (+ #split-n, #split-fit)
    div#sys-host.sys-host[data-pg=sys]         tablet/computer: the sys panels move here
  aside#ctrl-aside.stack          control panel (right or left; phone: bottom sheet)
    .side-bar  button#btn-side, #btn-sheet-close
    button#btn-estop.estop
    .panel[data-pg=drive]         shown on every page (initPages always)
    .panel[data-pg=X]             shown on page X
div#dock                          phone only: button#dock-estop, #dock-ctrl, button.tab[data-page=X]
div#page-hint                     "keep scrolling to go to the next page"
```

Screens:
* **phone** — `(max-width: 700px), (max-height: 520px) and (max-width: 1100px)`: one page at a
  time, the page tabs and E-STOP in the bottom dock, the panels of the page in one column.
* **tablet / computer** (`body.desk`): the page fills the screen beside the control panel and
  scrolls inside itself; E-STOP and the joystick never move. Windows of a page share the space;
  drag bars between them are remembered per browser; "↺ จัดหน้าต่าง" resets them.

## RobotUI (core.js)

```js
const { $, streamView, splitPair, junction, winify, setChip, nudge } = RobotUI;
RobotUI.initShell();                       // header height, panel side, phone sheet + dock, win-reset
winify(el);                                // el.cam gets the maximise button (Esc = back)
const x = splitPair(box, a, b, 'x', 'key', 0.5);   // a | b with a drag bar ('y' = a over b)
junction(box, x, y);                       // handle where two bars meet: drags both
RobotUI.initPages({ home: 'drive', always: ['drive'],
  streams: { drive: ['cam1-toggle'] },     // switched off when leaving the page, back on return
  defaults: { drive: ['cam1-toggle'] },    // switched on at the first visit
  deskHost: { sys: 'sys-host' },
  onLeave: (old, page) => {} });
addEventListener(RobotUI.PAGE_EVENT, e => e.detail.page);    // page changed
const show = streamView(viewEl); show('front'); show(null);  // camera over /ws/stream/front
RobotUI.splitScreen(info.cameras, { labels: {front: 'กล้องหน้า'}, defaults: ['front'], page: 'cams' });
const { send } = RobotUI.connect('/ws', { onmessage: msg => {}, onclose: () => {} });
const pad = RobotUI.joystick($('pad'), $('knob'), () => mayDrive);   // pad.active, pad.x, pad.y
const keys = RobotUI.driveKeys({ onEstop: () => send({type: 'estop'}), skip: () => false });
RobotUI.store.get('key') / .set / .json / .setJson                 // localStorage, data-store prefix
```

## Style rules

* Colours only through the variables in `theme.css` (`var(--accent)` etc.), so a robot can
  re-brand by overriding them. Status colours mean the same everywhere: `--ok` running / safe,
  `--warn` attention, `--bad` error and E-STOP. White text on `--accent`, `--ok`, `--warn`, `--bad`.
* Components: `.panel` (+ `h2` title), `.cam` window, `button` / `.on` / `.primary` / `.danger` /
  `.auto-on`, `.seg` (choice row), `.tabs` + `.tabpane`, `.sliders` (label | range | number),
  `.quick` (label | range | value), `.kv` (dl of values), `.chip`, `.badge`, `.hint`, `ol.steps`.
* E-STOP is always one tap away: `#btn-estop` in the control panel, `#dock-estop` on phones,
  Space on a keyboard. Buttons blur after a click so Space never re-presses them.
* Touch: buttons at least 40 px high on phones, inputs 16 px font (no iOS zoom).
* Text: Thai in the page; every new phrase also goes in the dictionary (.tsv). Never compare
  displayed text in code (it may be translated).
* Static files are served `no-cache`: after an update the browser picks up new code at reload.

Changing robot_web_ui changes every robot's page: keep the ids, classes and RobotUI functions
compatible, and check `qcar2_web` (all pages, phone and computer) after a change.
