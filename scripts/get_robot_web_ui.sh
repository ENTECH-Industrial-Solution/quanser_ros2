#!/usr/bin/env bash
# Get (or update) robot_web_ui, the robots' shared web UI, into <ws>/src/common/robot_web_ui.
# It is its own repo (ENTECH-Industrial-Solution/robot_web_ui), not part of quanser_ros2.
#
#   get_robot_web_ui.sh [<ws>]        <ws> = workspace root, default: the one this script is in
#
# Clones it the first time, `git pull --ff-only` after that. The repo is private: log in first
# (`gh auth login` + `gh auth setup-git`) or use SSH with a key that can read it:
#   ROBOT_WEB_UI_URL=git@github.com:ENTECH-Industrial-Solution/robot_web_ui.git get_robot_web_ui.sh
# Pin a version: ROBOT_WEB_UI_REF=<tag|branch|commit> get_robot_web_ui.sh
set -euo pipefail

URL="${ROBOT_WEB_UI_URL:-https://github.com/ENTECH-Industrial-Solution/robot_web_ui.git}"
REF="${ROBOT_WEB_UI_REF:-}"
ws="${1:-$(cd "$(dirname "$0")/.." && pwd)}"
dir="$ws/src/common/robot_web_ui"

if [ -d "$dir/.git" ]; then
  echo "robot_web_ui: updating $dir"
  if [ -n "$REF" ]; then
    git -C "$dir" fetch -q --tags origin
    git -C "$dir" checkout -q "$REF"
  else
    git -C "$dir" pull --ff-only
  fi
else
  # A car that had robot_web_ui inside quanser_ros2: `git pull` removed its files but can leave
  # __pycache__ behind, and git clone needs an empty directory.
  if [ -d "$dir" ] && [ -n "$(find "$dir" -type f ! -path '*/__pycache__/*' -print -quit)" ]; then
    echo "robot_web_ui: $dir has files but is not a git clone - move it away first" >&2
    exit 1
  fi
  rm -rf "$dir"
  mkdir -p "$(dirname "$dir")"
  echo "robot_web_ui: cloning $URL"
  git clone ${REF:+--no-checkout} "$URL" "$dir"
  [ -z "$REF" ] || git -C "$dir" checkout -q "$REF"
fi
