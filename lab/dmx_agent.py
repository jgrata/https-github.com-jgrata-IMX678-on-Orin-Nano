"""PC-side DMX HTTP agent so the (Jetson-served) web UI can control the lab
illuminants. The lights are on the PC's USB (ENTTEC Open DMX / COM5); the Jetson
webui proxies to this agent (default http://192.168.99.1:9200) over the direct link.

Burst-and-latch per request: each POST opens the FTDI, sets the levels, streams
~1.5s at 11 Hz, then closes -- so the port is FREE between sets and the agent
coexists with the MATLAB GUI and QLC+ (just not sending simultaneously). GET
returns the last-set levels from cache (no port access).

  GET  /dmx            -> {"d65":N,"tungsten":N,"ok":true}
  POST /dmx {d65,tungsten} -> apply + return the new state

Run:  python lab/dmx_agent.py            (binds 0.0.0.0:9200)
Stdlib only (http.server) -- no extra deps beyond dmx_lights' ftd2xx.
"""
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dmx_lights as dl

PORT = int(os.environ.get("DMX_AGENT_PORT", "9200"))
_state = {"d65": 0, "tungsten": 0}
_lock = threading.Lock()


def apply(d65, tungsten, hold=1.5):
    with _lock:                                   # serialize COM5 access
        d = dl.OpenDMX().start(rate_hz=11)
        try:
            d.set_many({dl.CH_D65: d65, dl.CH_TUNGSTEN: tungsten})
            time.sleep(hold)
        finally:
            d.close()
    _state["d65"] = d65
    _state["tungsten"] = tungsten


class Handler(BaseHTTPRequestHandler):
    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")

    def _json(self, code, obj):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self._cors()
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_OPTIONS(self):
        self.send_response(204); self._cors(); self.end_headers()

    def do_GET(self):
        if self.path.startswith("/dmx"):
            self._json(200, dict(_state, ok=True))
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        if not self.path.startswith("/dmx"):
            self._json(404, {"error": "not found"}); return
        try:
            n = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            body = {}
        d65 = max(0, min(255, int(body.get("d65", _state["d65"]))))
        tung = max(0, min(255, int(body.get("tungsten", _state["tungsten"]))))
        try:
            apply(d65, tung)
            self._json(200, dict(_state, ok=True))
        except Exception as e:
            msg = str(e)
            if "denied" in msg.lower() or "Access" in msg:
                msg = "COM5 busy (close QLC+)"
            self._json(503, {"error": msg, **_state})

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    print("DMX agent on http://0.0.0.0:%d   (GET/POST /dmx)" % PORT)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
