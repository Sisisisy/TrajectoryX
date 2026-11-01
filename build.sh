#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh" "$@"
mkdir -p "$CONTROLLER_WORKSPACE"
cd "$CONTROLLER_WORKSPACE"
python /usr/bin/colcon build --symlink-install --base-paths "$SUBMISSION_ROOT/team_controller" --packages-select team_controller
set +u
source "$CONTROLLER_WORKSPACE/install/setup.bash"
set -u
ros2 pkg executables team_controller
echo 'Build complete. Start the official simulator, then run verify.sh.'
