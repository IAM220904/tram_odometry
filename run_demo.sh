#!/usr/bin/env bash
# One-command jury demo: prints the four-criterion technical report from saved evidence.
# Usage: ./run_demo.sh [--tests] [--open]
#   --tests  also run the unit tests (a few seconds)
#   --open   open results/final_demo in a file browser if a desktop is available
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
if [ -f /opt/ros/humble/setup.bash ] && [ -z "${ROS_DISTRO:-}" ]; then set +u; source /opt/ros/humble/setup.bash; set -u; fi
if [ -f install/setup.bash ]; then set +u; source install/setup.bash; set -u; fi
# A NumPy in ~/.local can break the system SciPy/Matplotlib; use system packages only.
export PYTHONNOUSERSITE=1 MPLBACKEND=Agg
for a in "$@"; do
  if [ "$a" = "--tests" ]; then python3 -m unittest discover -s tests 2>&1 | tail -3; fi
done
python3 tools/final_report.py
for a in "$@"; do
  if [ "$a" = "--open" ] && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && command -v xdg-open >/dev/null; then
    xdg-open results/final_demo >/dev/null 2>&1 || true
  fi
done
echo
echo "RUN AGAIN:"
echo "cd /home/iam220904/rezerv_odometr/tram_odometry_solution"
echo "./run_demo.sh"
