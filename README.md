# quanser_ros2

ROS 2 **Humble** workspace for Quanser robots running on real hardware (NVIDIA Jetson).
For the Isaac Sim (Jazzy) side see [quanser_issac_ros2](https://github.com/ENTECH-Industrial-Solution/quanser_issac_ros2).

## Layout

```
src/
├── qcar2/      QCar2: qcar2_web (web UI + systemd services), qcar2_nodes,
│               qcar2_autonomy, qcar2_odom, qcar2_interfaces
└── common/     shared packages (when needed)
scripts/clone_robot.sh
```

Each robot lives in `src/<robot>/`. To add one, create `src/<robot>/`.

---

## Setting up a new / factory QCar2

> **Paths are fixed:** the systemd units and `qcar2-run` assume user **`nvidia`** and the
> workspace at **`/home/nvidia/ros2`**. Clone to exactly that path.

### 0. Check the car first

The factory image must already have:

| Needed | Check |
|---|---|
| ROS 2 Humble | `ls /opt/ros/humble/setup.bash` |
| Quanser libraries (QAL) | `ls ~/Documents/Quanser` |
| CycloneDDS: `rmw_cyclonedds_cpp`, `~/cyclonedds.xml`, `~/cyclonedds_ws` | `ls ~/cyclonedds.xml ~/cyclonedds_ws/install/setup.bash` |
| Cartographer, Nav2, RTAB-Map, image_transport (used by launch files) | `ros2 pkg list \| grep -E 'cartographer_ros\|nav2_bringup\|rtabmap_slam'` |

`cyclonedds.xml` and `cyclonedds_ws` are **not** in this repo; `qcar2-run` sources them.
If something is missing, install it before step 3 (`rosdep install` below covers the apt ones).

### 1. Get the code — pick ONE of the two ways

Both clone **only QCar2** (not other robots) into `~/ros2`.

**Way A — with the script**
```bash
cd ~
curl -fsSLO https://raw.githubusercontent.com/ENTECH-Industrial-Solution/quanser_ros2/main/scripts/clone_robot.sh
bash clone_robot.sh qcar2 -d ~/ros2
```
(Download first, then run, so you can read the script before running it.)

**Way B — plain git, no script**
```bash
git clone --filter=blob:none --no-checkout https://github.com/ENTECH-Industrial-Solution/quanser_ros2.git ~/ros2
cd ~/ros2
git sparse-checkout init --cone
git sparse-checkout set src/qcar2 scripts
git checkout
```

> Works without login while the repo is public. If it becomes private, use an SSH key / token
> (`QUANSER_ROS2_URL=git@github.com:ENTECH-Industrial-Solution/quanser_ros2.git bash clone_robot.sh ...`).

### 2. Build — **humble only**

A galactic (or other distro) in the environment breaks `qcar2-robot`. Use a clean shell:

```bash
source /opt/ros/humble/setup.bash
source ~/cyclonedds_ws/install/setup.bash   # needed if RMW_IMPLEMENTATION=rmw_cyclonedds_cpp (factory ~/.bashrc sets it)
cd ~/ros2
rosdep install --from-paths src --ignore-src -y     # apt deps, if any are missing
colcon build
```

### 3. Install the services (once per car)

```bash
sudo ~/ros2/src/qcar2/qcar2_web/system/install.sh
```

This installs `qcar2-web` (enabled at boot) plus the robot / SLAM / nav / camera units and the
`qcar-*` helper commands. `qcar2-robot` is **deliberately not enabled at boot** (the clock is still
1970 then and the hardware fails with "A GPIO is in use").

### 4. Use it

1. Open `http://<car-ip>` (port 80, or `:8080`) in a browser.
2. Press **"เริ่ม ROS"** (or hardware button S3) to start ROS. `sudo qcar-mode ros` does the same from a terminal.
3. Cameras (`qcar2-csi`, `qcar2-realsense`) are started by the web node; S4 = eco mode.

Do not start the ROS hardware nodes while a QUARC (MATLAB/Simulink) model is running — `qcar2-run` refuses.

---

## Updating a car that is already set up

```bash
cd ~/ros2 && git pull
source /opt/ros/humble/setup.bash && colcon build
sudo src/qcar2/qcar2_web/system/install.sh      # only if files in system/ changed
```

## Adding another robot later

On a machine that already has the repo: `./scripts/clone_robot.sh --list`, or
`git sparse-checkout add src/<robot>` inside `~/ros2`.
