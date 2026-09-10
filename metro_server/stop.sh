#!/usr/bin/env bash
# stop.sh -- stop the web UI, the RAW/DAQ backend, and raw_capture.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "== stopping web UI + image_server + raw_capture =="
pkill -f "webui/server.py"                 2>/dev/null || true
pkill -f "image_server.py"                 2>/dev/null || true
pkill -f "$HERE/raw_capture/raw_capture"   2>/dev/null || true
# free the ports if anything lingers
for p in "${PORT:-8080}" "${IMG_PORT:-9000}" 9001; do
    fuser -k "${p}/tcp" 2>/dev/null || true
done
echo "== stopped =="
