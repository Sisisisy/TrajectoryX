#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh" "$@"
if [[ ! -f "$CONTROLLER_WORKSPACE/install/setup.bash" ]]; then
  echo 'Run bash build.sh /path/to/competition-repo first.' >&2; exit 2
fi
set +u
source "$CONTROLLER_WORKSPACE/install/setup.bash"
set -u
python "$SUBMISSION_ROOT/verify_interface.py"
