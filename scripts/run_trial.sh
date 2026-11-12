#!/usr/bin/env bash
set -euo pipefail
source /home/patton/cooperative_driving/scripts/env.sh
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/patton/.Xauthority}"
export SDL_AUDIODRIVER=dummy
cd "$COOP_ROOT"
exec "$HOCD_PYTHON" -B "$COOP_ROOT/vendor/HOCD/main.py" "$@"
