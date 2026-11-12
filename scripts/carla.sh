#!/usr/bin/env bash
set -euo pipefail
source /home/patton/cooperative_driving/scripts/env.sh
exec "$HOCD_PYTHON" -B "$COOP_ROOT/scripts/carla_service.py" "$@"
