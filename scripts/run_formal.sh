#!/usr/bin/env bash
set -euo pipefail
cd /home/patton/cooperative_driving
exec bash scripts/run_trial.sh --formal "$@"
