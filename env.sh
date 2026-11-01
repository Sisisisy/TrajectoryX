#!/usr/bin/env bash
# Shared environment loader; invoked by the other scripts.
set -euo pipefail
SUBMISSION_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 1 ]]; then
  echo 'Provide the absolute path to the official competition repository.' >&2
  return 2 2>/dev/null || exit 2
fi
COMPETITION_REPO="$1"
if [[ ! -f /opt/ros/jazzy/setup.bash || ! -f "$COMPETITION_REPO/starter_ws/install/setup.bash" || ! -f "$COMPETITION_REPO/starter_ws/.venv/bin/activate" ]]; then
  echo 'ROS 2 Jazzy / official simulator setup is missing. Complete official INSTALL.md first.' >&2
  return 2 2>/dev/null || exit 2
fi
# ROS setup files may reference unset environment variables.
set +u
source /opt/ros/jazzy/setup.bash
source "$COMPETITION_REPO/starter_ws/.venv/bin/activate"
source "$COMPETITION_REPO/starter_ws/install/setup.bash"
set -u
CONTROLLER_WORKSPACE="$SUBMISSION_ROOT/controller_ws"
