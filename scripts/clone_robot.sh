#!/usr/bin/env bash
# Clone quanser_ros2 and check out ONLY the packages for the given robot(s).
#
#   clone_robot.sh <robot> [<robot> ...] [-d <dir>] [-b <branch>]
#   clone_robot.sh --list
#
# Examples:
#   clone_robot.sh qcar2
#   clone_robot.sh qbot -d ~/ros2
#   clone_robot.sh qcar2 qdrone
#
# Layout expected in the repo:  src/<robot>/<package>...   (+ optional src/common)
# Add more robots later by just creating src/<robot>/ -- no change needed here.
set -euo pipefail

REPO_URL="${QUANSER_ROS2_URL:-https://github.com/ENTECH-Industrial-Solution/quanser_ros2.git}"
SHARED=(src/common scripts)   # always checked out if they exist

usage() { sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-1}"; }

robots=() dest="" branch=""
list=0
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage 0 ;;
    --list)    list=1 ;;
    -d)        dest="${2:?-d needs a directory}"; shift ;;
    -b)        branch="${2:?-b needs a branch}"; shift ;;
    -*)        echo "unknown option: $1" >&2; usage ;;
    *)         robots+=("$1") ;;
  esac
  shift
done

command -v git >/dev/null || { echo "git not found" >&2; exit 1; }

# --list: show robots available on the remote without cloning anything
if [ "$list" = 1 ]; then
  tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
  git clone -q --depth 1 --filter=blob:none --no-checkout \
      ${branch:+-b "$branch"} "$REPO_URL" "$tmp/r"
  echo "Available robots:"
  git -C "$tmp/r" ls-tree -d --name-only HEAD src/ | sed 's#^src/#  #' | grep -v '^  common$'
  exit 0
fi

[ "${#robots[@]}" -gt 0 ] || usage
dest="${dest:-quanser_ros2}"
[ -e "$dest" ] && { echo "'$dest' already exists - pick another with -d" >&2; exit 1; }

# NB: not "clone --sparse": git 2.25 (JetPack/Ubuntu 20.04) fails on it with some URLs.
git clone --filter=blob:none --no-checkout ${branch:+-b "$branch"} "$REPO_URL" "$dest"
cd "$dest"
git sparse-checkout init --cone

# Validate robot names against what really exists in the repo
available=$(git ls-tree -d --name-only HEAD src/ | sed 's#^src/##')
paths=()
for r in "${robots[@]}"; do
  if ! printf '%s\n' "$available" | grep -qx -- "$r"; then
    echo "robot '$r' not found. Available:" >&2
    printf '%s\n' "$available" | grep -vx common | sed 's/^/  /' >&2
    cd .. && rm -rf "$dest"
    exit 1
  fi
  paths+=("src/$r")
done
for s in "${SHARED[@]}"; do
  git ls-tree -d --name-only HEAD "$s" | grep -q . && paths+=("$s")
done

git sparse-checkout set "${paths[@]}"
git checkout -q

echo
echo "Checked out: ${paths[*]}"
echo "Next:  cd $dest && source /opt/ros/humble/setup.bash && colcon build"
