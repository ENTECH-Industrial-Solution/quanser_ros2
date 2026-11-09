#!/bin/bash
# Installs the QCar2 demo services and helper commands:
#   qcar2-robot.service, qcar2-web.service   (started at boot)
#   qcar2-slam.service, qcar2-nav@.service   (lidar mapping / navigation, started by qcar-nav)
#   qcar2-vslam.service, qcar2-vnav@.service (camera V-SLAM mapping / navigation)
#   qcar2-csi.service, qcar2-realsense.service (cameras, started by the web node; eco mode)
#   /usr/local/bin/qcar2-run, /usr/local/sbin/qcar-wifi, qcar-mode, qcar-nav
#
#   sudo ~/ros2/src/qcar2/qcar2_web/system/install.sh
set -e
cd "$(dirname "$0")"
install -m 755 qcar2-run /usr/local/bin/qcar2-run
install -m 755 qcar-wifi /usr/local/sbin/qcar-wifi
install -m 755 qcar-mode /usr/local/sbin/qcar-mode
install -m 755 qcar-nav /usr/local/sbin/qcar-nav
install -m 755 qcar-health /usr/local/sbin/qcar-health
install -m 644 qcar2-robot.service qcar2-web.service qcar2-slam.service qcar2-nav@.service \
  qcar2-vslam.service qcar2-vnav@.service qcar2-health.service qcar2-health.timer \
  qcar2-csi.service qcar2-realsense.service /etc/systemd/system/
install -d /etc/systemd/journald.conf.d /var/log/journal
install -m 644 journald-qcar2.conf /etc/systemd/journald.conf.d/qcar2.conf
systemd-tmpfiles --create --prefix /var/log/journal
systemctl restart systemd-journald
systemctl daemon-reload
# qcar2-robot is deliberately NOT enabled: ROS is started by hand (web "เริ่ม ROS" / S3). At boot the clock
# is still 1970 and the hardware fails with "A GPIO is in use", leaving lidar running and buttons dead.
systemctl enable qcar2-web.service
systemctl disable qcar2-robot.service || true
systemctl enable --now qcar2-health.timer
echo "Installed. Start now with: sudo qcar-mode ros"
