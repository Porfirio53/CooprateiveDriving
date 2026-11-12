#!/usr/bin/env bash
set -euo pipefail
source /home/patton/cooperative_driving/scripts/env.sh
cd "$COOP_ROOT"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/patton/.Xauthority}"
exec /home/patton/carla_experiments/simulator/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping \
  "${COOP_CARLA_MAP:-/Game/Carla/Maps/Town04}" -quality-level=Low -carla-rpc-port=2000 -RenderOffScreen -nosound \
  -ResX=640 -ResY=360 -userdir="$COOP_ROOT/runtime/carla" \
  -abslog="$COOP_ROOT/logs/carla.log" "$@"
