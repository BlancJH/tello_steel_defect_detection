#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$REPO_ROOT/tello_ws"
ENV_FILE="$REPO_ROOT/.env"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
VENV_DIR="${VENV_DIR:-$WS_DIR/venv}"

if [[ -f "$ENV_FILE" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$ENV_FILE"
  set +a
fi

CONFIG_FILE="${CONFIG_FILE:-$WS_DIR/src/tello_defect_pipeline/config/pipeline.yaml}"
if [[ "$CONFIG_FILE" != /* ]]; then
  CONFIG_FILE="$REPO_ROOT/$CONFIG_FILE"
fi

if [[ "$VENV_DIR" != /* ]]; then
  VENV_DIR="$REPO_ROOT/$VENV_DIR"
fi

require_file() {
  local path="$1"
  local label="$2"
  if [[ ! -e "$path" ]]; then
    echo "Missing $label: $path" >&2
    exit 1
  fi
}

require_file "$ROS_SETUP" "ROS setup file"
require_file "$WS_DIR/install/setup.bash" "workspace install setup file. Run colcon build first"
require_file "$CONFIG_FILE" "ROS parameter config file"

setup_lines() {
  cat <<EOF
cd '$WS_DIR'
source '$ROS_SETUP'
if [[ -f '$VENV_DIR/bin/activate' ]]; then
  source '$VENV_DIR/bin/activate'
  VENV_PYTHON_DIR=\$(find '$VENV_DIR/lib' -maxdepth 1 -type d -name 'python*' | head -n 1)
  if [[ -n "\$VENV_PYTHON_DIR" ]]; then
    export PYTHONPATH="\$VENV_PYTHON_DIR/site-packages:\${PYTHONPATH:-}"
  fi
fi
source '$WS_DIR/install/setup.bash'
EOF
}

open_terminal() {
  local title="$1"
  local command="$2"
  local full_command
  full_command="$(setup_lines)
clear
echo '=== $title ==='
echo '$command'
echo
$command
echo
echo '[$title exited] Press Ctrl-D or close this terminal.'
exec bash -i"

  if [[ -n "${TERMINAL_CMD:-}" ]]; then
    "$TERMINAL_CMD" -e bash -lc "$full_command" &
  elif command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$title" -- bash -lc "$full_command" &
  elif command -v kgx >/dev/null 2>&1; then
    kgx --title="$title" -- bash -lc "$full_command" &
  elif command -v konsole >/dev/null 2>&1; then
    konsole --new-tab --title "$title" -e bash -lc "$full_command" &
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    xfce4-terminal --title="$title" --command="bash -lc $(printf '%q' "$full_command")" &
  elif command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -T "$title" -e bash -lc "$full_command" &
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T "$title" -e bash -lc "$full_command" &
  else
    echo "No supported terminal emulator found." >&2
    echo "Install gnome-terminal, kgx, konsole, xfce4-terminal, or xterm." >&2
    exit 1
  fi
}

open_terminal \
  "Tello Live Pipeline" \
  "ros2 launch tello_defect_pipeline live_pipeline.launch.py config_file:='$CONFIG_FILE' use_keyboard:=false"

sleep 1

open_terminal \
  "Tello Keyboard Controller" \
  "ros2 launch tello_defect_pipeline teleop.launch.py config_file:='$CONFIG_FILE'"

