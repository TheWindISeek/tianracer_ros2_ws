#!/bin/bash

set -euo pipefail

echo "[nomoregazebo] current Gazebo related processes"
if ! pgrep -a gzserver >/dev/null; then
    echo "  no gzserver process"
else
    pgrep -a gzserver
fi

if ! pgrep -a gzclient >/dev/null; then
    echo "  no gzclient process"
else
    pgrep -a gzclient
fi

echo "[nomoregazebo] try to end gzserver/gzclient normally..."
killall gzserver 2>/dev/null || true
killall gzclient 2>/dev/null || true

sleep 1

echo "[nomoregazebo] force to end residual processes..."
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true

echo "[nomoregazebo] done."
