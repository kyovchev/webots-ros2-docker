#!/usr/bin/env bash
# Wait until the X server is accepting connections, then run the command
until xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; do sleep 0.2; done
exec "$@"
