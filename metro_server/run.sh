#!/usr/bin/env bash
# =============================================================================
# run.sh -- launch the IMX678 RAW/DAQ backend + web UI on the local Jetson,
# then open the UI in a browser.
#
#   ./run.sh              start everything + open the browser; Ctrl+C to stop
#   NO_BROWSER=1 ./run.sh start headless (no browser) -- for SSH / services
#   PORT=8080 IMG_PORT=9000 ./run.sh   override ports
#
# Tiers:  raw_capture (Argus, :9001, auto-spawned by image_server)
#         image_server.py  (RAW/DAQ backend, :9000)
#         webui/server.py  (FastAPI web UI, :8080)
# =============================================================================
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export IMG_HOST="${IMG_HOST:-127.0.0.1}"
export IMG_PORT="${IMG_PORT:-9000}"
export PORT="${PORT:-8080}"
LOGDIR="${METRO_LOG_DIR:-$HOME/metro_sessions/logs}"
mkdir -p "$LOGDIR"

# --- preflight ---------------------------------------------------------------
if [ ! -x "$HERE/raw_capture/raw_capture" ]; then
    echo "ERROR: raw_capture not built. Run ./install.sh first."; exit 1
fi
if ! systemctl is-active --quiet nvargus-daemon; then
    echo "== starting nvargus-daemon (Argus) =="
    sudo systemctl restart nvargus-daemon || echo "WARN: could not (re)start nvargus-daemon"
fi

pids=()
cleanup() {
    echo; echo "== stopping =="
    for p in "${pids[@]:-}"; do kill "$p" 2>/dev/null || true; done
    # raw_capture is a child of image_server, but make sure it's gone
    pkill -f "$HERE/raw_capture/raw_capture" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_port() {  # wait_port <port> <secs> <label>
    local port="$1" secs="$2" label="$3" i
    for ((i=0; i<secs; i++)); do
        (exec 3<>"/dev/tcp/127.0.0.1/$port") 2>/dev/null && { exec 3>&- 3<&-; return 0; }
        sleep 1
    done
    echo "WARN: $label did not open :$port within ${secs}s (see logs in $LOGDIR)"
    return 1
}

# --- 1) RAW/DAQ backend (image_server -> spawns raw_capture) ------------------
echo "== starting image_server on :$IMG_PORT (first Argus init ~7s) =="
( cd "$HERE" && exec python3 image_server.py ) >"$LOGDIR/image_server.log" 2>&1 &
pids+=($!)
wait_port "$IMG_PORT" 40 "image_server" || true

# --- 2) web UI (uvicorn) -----------------------------------------------------
echo "== starting web UI on :$PORT =="
( cd "$HERE" && exec python3 webui/server.py ) >"$LOGDIR/webui.log" 2>&1 &
pids+=($!)
wait_port "$PORT" 25 "web UI" || true

URL="http://localhost:$PORT"
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo
echo "======================================================================"
echo "  Web UI:   $URL"
[ -n "${IP:-}" ] && echo "  (LAN):    http://$IP:$PORT"
echo "  Logs:     $LOGDIR/{image_server,webui}.log"
echo "  Stop:     Ctrl+C  (or ./stop.sh from another shell)"
echo "======================================================================"

# --- 3) open the browser on the local Jetson ---------------------------------
if [ -z "${NO_BROWSER:-}" ] && [ -n "${DISPLAY:-}" ]; then
    ( xdg-open "$URL" >/dev/null 2>&1 || sensible-browser "$URL" >/dev/null 2>&1 || true ) &
elif [ -z "${NO_BROWSER:-}" ]; then
    echo "  (no DISPLAY -- open $URL in a browser yourself, or set NO_BROWSER=1)"
fi

# keep the two servers in the foreground; cleanup() stops them on exit
wait
