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

## Clone only one robot

```bash
./scripts/clone_robot.sh qcar2          # or: qbot, qdrone, ...
./scripts/clone_robot.sh --list
```

(Or without the script: `git clone --filter=blob:none --no-checkout <url> && cd quanser_ros2 && git sparse-checkout init --cone && git sparse-checkout set src/qcar2 && git checkout`.)

## Build (QCar2)

Use **humble only** — having galactic in the environment breaks qcar2-robot.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2 && colcon build
```

## Run

systemd units are in `src/qcar2/qcar2_web/system/`; install with
`sudo src/qcar2/qcar2_web/system/install.sh`. The robot service is disabled at boot —
start ROS from the web UI ("เริ่ม ROS"). Cameras (`qcar2-csi`, `qcar2-realsense`) are
started by the web node.
