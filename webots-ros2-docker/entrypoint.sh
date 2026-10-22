#!/usr/bin/env bash
set -e

# Optional VNC password: docker run -e VNC_PASSWORD=secret ...
if [ -n "${VNC_PASSWORD:-}" ]; then
  mkdir -p "$HOME/.vnc"
  x11vnc -storepasswd "$VNC_PASSWORD" "$HOME/.vnc/passwd" >/dev/null
  export VNC_AUTH_ARGS="-rfbauth $HOME/.vnc/passwd"
else
  export VNC_AUTH_ARGS="-nopw"
fi

source /opt/ros/jazzy/setup.bash
exec "$@"
