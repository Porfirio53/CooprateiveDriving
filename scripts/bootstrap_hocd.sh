#!/usr/bin/env bash
set -euo pipefail
source /home/patton/cooperative_driving/scripts/env.sh
cd "$COOP_ROOT"
if [[ ! -x "$HOCD_PYTHON" ]]; then
  python3.10 -m venv "$COOP_ROOT/envs/hocd"
fi
"$HOCD_PYTHON" -m pip install 'torch==1.13.1+cpu' --index-url https://download.pytorch.org/whl/cpu
"$HOCD_PYTHON" -m pip install -r "$COOP_ROOT/configs/requirements-hocd.txt"
"$HOCD_PYTHON" -m pip check
