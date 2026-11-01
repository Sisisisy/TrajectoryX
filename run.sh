#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh" "$@"
if [[ ! -f "$CONTROLLER_WORKSPACE/install/setup.bash" ]]; then
  echo 'Run bash build.sh /path/to/competition-repo first.' >&2; exit 2
fi
set +u
source "$CONTROLLER_WORKSPACE/install/setup.bash"
set -u
SPEED_CAP="${2:-0.4}"
echo "Starting controller with requested speed cap $SPEED_CAP m/s (hard maximum 0.8)."
echo 'Stop any other controller or manual teleoperation first. Ctrl+C to stop.'
exec ros2 run team_controller controller --ros-args -p "max_speed:=$SPEED_CAP"
