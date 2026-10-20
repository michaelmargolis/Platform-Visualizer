#!/bin/sh
# run_pi3.sh ver 2
# Usage: ./pi/run_pi3.sh [POC_ROOT]
set -eu
HERE="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
POC_ROOT="${1:-$HOME/Runtime}"
exec python3 "$HERE/platform_visualizer.py" --poc-root "$POC_ROOT" --config "$HERE/app_config.json" --fullscreen
