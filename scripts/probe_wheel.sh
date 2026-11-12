#!/usr/bin/env bash
set -euo pipefail
source /home/patton/cooperative_driving/scripts/env.sh
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/patton/.Xauthority}"
export SDL_AUDIODRIVER=dummy
exec "$HOCD_PYTHON" -B "$COOP_ROOT/scripts/probe_wheel.py" "$@"
