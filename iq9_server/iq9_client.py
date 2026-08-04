"""PC-side resilient supervisor for the IQ9 (QCS9075) IMX678 DAQ server.

Why this exists
---------------
The QCS9075 CamX/IFE stack is still immature relative to the Jetson: certain sensor
operations trip an *in-kernel* IFE SMMU page fault which the qcom_wdt hardware
watchdog converts into a full SoC reset. A userspace try/except on the board CANNOT
catch that -- by the time the fault fires the board is already going down.

So resilience is done from the PC instead, as detect-and-recover (not prevent-in-process):

  1. Per-call hard timeout        -- kills soft hangs (HTTP timeout).
  2. Vanish detection + recovery  -- when the board stops answering and comes back with a
                                     reset uptime, we know it rebooted; we wait for the
                                     board's own systemd (iq9web.service, Restart=always,
                                     enabled at boot) to restore the DAQ server, then retry.
  3. Checkpointed sweeps          -- each completed point is written to disk; a reboot mid
                                     sweep costs one point + recovery, not the whole run.
  4. Capability allowlist         -- routine DAQ issues only KNOWN-STABLE ops; the ops
                                     known to reboot the board require allow_unstable=True
                                     and are logged into the growing failure-mode map.
  5. Reboot-event log             -- every vanish/soft-fail is appended to a JSONL so we
                                     accumulate exactly which operations are still fragile.

Stdlib only (urllib + subprocess ssh) so it runs anywhere Python does; SSH is used only
for out-of-band health/recovery, never in the capture hot path.

CLI:
    python iq9_client.py health
    python iq9_client.py arm            # enable RAW (supervised), disarm to gate off
    python iq9_client.py capture        # one resilient RAW snapshot -> stats
    python iq9_client.py lightsweep     # DMX light sweep -> checkpointed RAW capture
"""
import json
import os
import subprocess
import sys
import time
import urllib.request

HOST = os.environ.get("IQ9_HOST", "192.168.99.2")
PORT = int(os.environ.get("IQ9_PORT", "8080"))
SSH_USER = os.environ.get("IQ9_SSH_USER", "metro")
DMX_AGENT = os.environ.get("DMX_AGENT", "http://127.0.0.1:9200")

# Operations empirically confirmed to hard-reboot the board (the failure-mode map).
# Keyed by op_name; value is why. resilient() refuses these unless allow_unstable=True.
KNOWN_UNSTABLE = {
    "nv12_manual_exposure": "IFE SMMU out-of-bounds page fault -> qcom_wdt reset "
                            "(NV12/processed path with control-mode=off + manual-exposure)",
    "raw_continuous": "continuous RAW/RDI streaming reboots; only snapshot RAW is stable",
    "nv12_raw_handoff": "rapid NV12<->RAW mode churn is the classic CAMSS/RDI hang trigger",
}

# Operations validated as stable (safe for routine DAQ).
KNOWN_STABLE = {
    "raw_snapshot": "open->grab->close single RAW16 frame (cold mode)",
    "nv12_auto": "NV12 live view with auto 3A",
    "info": "read-only status",
    "health": "ping / uptime / webui probe",
}


class RebootDetected(Exception):
    """Raised internally when the board vanished and came back with a reset uptime."""


class IQ9Error(Exception):
    pass


class IQ9:
    def __init__(self, host=HOST, port=PORT, ssh_user=SSH_USER,
                 event_log=None, verbose=True):
        self.host = host
        self.port = port
        self.ssh_user = ssh_user
        self.base = "http://%s:%d" % (host, port)
        self.verbose = verbose
        self.event_log = event_log or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "iq9_events.jsonl")

    # ---- low-level transport -------------------------------------------------
    def _http(self, path, method="GET", body=None, timeout=45):
        url = self.base + path
        data = None
        headers = {}
        if body is not None:
            data = json.dumps(body).encode()
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(url, data=data, method=method, headers=headers)
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read()
        ctype = r.headers.get("Content-Type", "")
        if "application/json" in ctype:
            return json.loads(raw)
        return raw

    def _ssh(self, cmd, timeout=20):
        """Out-of-band control channel. Returns (rc, stdout). Never used in the hot path."""
        full = ["ssh", "-o", "ConnectTimeout=8", "-o", "BatchMode=yes",
                "%s@%s" % (self.ssh_user, self.host), cmd]
        try:
            p = subprocess.run(full, capture_output=True, text=True, timeout=timeout)
            return p.returncode, (p.stdout or "").strip()
        except subprocess.TimeoutExpired:
            return 255, ""
        except Exception:
            return 255, ""

    def _log(self, *a):
        if self.verbose:
            print("[iq9]", *a, flush=True)

    def _event(self, kind, **fields):
        rec = {"t": time.strftime("%Y-%m-%dT%H:%M:%S"), "kind": kind}
        rec.update(fields)
        try:
            with open(self.event_log, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec) + "\n")
        except OSError:
            pass
        return rec

    # ---- health / recovery ---------------------------------------------------
    def uptime(self):
        """Board uptime in seconds via SSH, or None if unreachable."""
        rc, out = self._ssh("cut -d. -f1 /proc/uptime", timeout=12)
        if rc == 0 and out.isdigit():
            return int(out)
        return None

    def webui_ok(self, timeout=5):
        try:
            self._http("/api/frame.jpg?width=64", timeout=timeout)
            return True
        except Exception:
            return False

    def health(self):
        up = self.uptime()
        ok = self.webui_ok()
        info = {}
        if ok:
            try:
                info = self._http("/api/info", timeout=6)
            except Exception:
                info = {}
        return {"reachable": up is not None, "uptime_s": up,
                "webui": ok, "raw_enabled": bool(info.get("raw_enabled", False))
                if isinstance(info, dict) else False}

    def wait_healthy(self, timeout_s=240, poll=6):
        """Block until the DAQ server answers again (used after a reboot)."""
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.webui_ok(timeout=5):
                up = self.uptime()
                self._log("board healthy again (uptime=%ss)" % up)
                return True
            time.sleep(poll)
        return False

    def ensure_service(self):
        """Make sure the persistent DAQ service is up (belt-and-suspenders after recovery)."""
        rc, out = self._ssh("systemctl is-active iq9web", timeout=12)
        if out != "active":
            self._ssh("systemctl restart iq9web", timeout=20)
            time.sleep(5)

    # ---- the resilient wrapper ----------------------------------------------
    def resilient(self, op_name, fn, retries=2, per_call_timeout=45,
                  allow_unstable=False):
        """Run fn() with reboot-aware recovery.

        fn is a zero-arg callable that performs one board interaction (typically an
        HTTP capture). On a *soft* failure (board still up) we clean up and retry; on a
        *vanish* (board rebooted) we wait for it to come back, re-arm state, and retry.
        Raises IQ9Error only after exhausting retries.
        """
        if op_name in KNOWN_UNSTABLE and not allow_unstable:
            raise IQ9Error(
                "refusing KNOWN-UNSTABLE op %r (%s). Pass allow_unstable=True to override."
                % (op_name, KNOWN_UNSTABLE[op_name]))

        attempt = 0
        while True:
            up0 = self.uptime()
            try:
                return fn()
            except Exception as e:
                attempt += 1
                # classify: did the board vanish/reboot, or just a soft failure?
                time.sleep(2)
                up1 = self.uptime()
                rebooted = (up1 is None) or (up0 is not None and up1 is not None and up1 < up0)
                if rebooted:
                    ev = self._event("reboot", op=op_name, uptime_before=up0,
                                     uptime_after=up1, error=str(e)[:200])
                    self._log("REBOOT during %r (uptime %s->%s); waiting for recovery..."
                              % (op_name, up0, up1))
                    if not self.wait_healthy():
                        raise IQ9Error("board did not recover after reboot during %r" % op_name)
                    self.ensure_service()
                    if self._raw_armed:          # re-arm RAW if the caller had armed it
                        self.arm_raw()
                else:
                    self._event("soft_fail", op=op_name, uptime=up1, error=str(e)[:200])
                    self._log("soft failure during %r: %s" % (op_name, str(e)[:120]))
                    self._cleanup()
                if attempt > retries:
                    raise IQ9Error("op %r failed after %d attempts: %s"
                                   % (op_name, attempt, e))
                self._log("retry %d/%d for %r" % (attempt, retries, op_name))

    def _cleanup(self):
        """Best-effort recovery of a wedged camera between soft retries."""
        self._ssh("pkill -9 -f camera_worker.py 2>/dev/null; "
                  "pkill -f gst-launch 2>/dev/null; true", timeout=12)
        time.sleep(2)
        self.ensure_service()
        time.sleep(2)

    # ---- RAW arming (gated capability) --------------------------------------
    _raw_armed = False

    def arm_raw(self):
        """Enable RAW capture on the board for a supervised session (IQ9_RAW_ENABLE=1)."""
        self._ssh("mkdir -p /etc/systemd/system/iq9web.service.d && "
                  "printf '[Service]\\nEnvironment=IQ9_RAW_ENABLE=1\\n' "
                  "> /etc/systemd/system/iq9web.service.d/raw.conf && "
                  "systemctl daemon-reload && systemctl restart iq9web", timeout=25)
        self._raw_armed = True
        self.wait_healthy(timeout_s=60)
        self._log("RAW armed (IQ9_RAW_ENABLE=1)")

    def disarm_raw(self):
        self._ssh("rm -f /etc/systemd/system/iq9web.service.d/raw.conf && "
                  "systemctl daemon-reload && systemctl restart iq9web", timeout=25)
        self._raw_armed = False
        self.wait_healthy(timeout_s=60)
        self._log("RAW disarmed (gated off)")

    # ---- capture helpers (KNOWN-STABLE) -------------------------------------
    def capture_raw(self, width=960, retries=2, timeout=45):
        """One resilient RAW16 snapshot -> characterization stats dict."""
        return self.resilient(
            "raw_snapshot",
            lambda: self._http("/api/raw/capture?width=%d" % width, timeout=timeout),
            retries=retries, per_call_timeout=timeout)

    def save_raw(self, n_frames=1, label="raw", retries=2, timeout=60):
        """Capture + persist N RAW16 frames on the board (uint16 .npy + JSON sidecar)."""
        return self.resilient(
            "raw_snapshot",
            lambda: self._http("/api/raw/save", method="POST",
                               body={"n_frames": n_frames, "label": label}, timeout=timeout),
            retries=retries, per_call_timeout=timeout)

    # ---- checkpointed sweep --------------------------------------------------
    def sweep(self, points, capture_fn, checkpoint_path, label="sweep",
              settle_s=0.0):
        """Run capture_fn(point) for each point, resiliently, resuming from a checkpoint.

        capture_fn(point) -> JSON-serializable result (it should itself use the resilient
        helpers above). Completed points are appended to checkpoint_path (JSONL); on
        restart, already-done points (by their string key) are skipped. A reboot mid-sweep
        costs one point + recovery, never the whole run.
        """
        done = {}
        if os.path.exists(checkpoint_path):
            with open(checkpoint_path, encoding="utf-8") as f:
                for line in f:
                    try:
                        rec = json.loads(line)
                        done[str(rec.get("point"))] = rec
                    except Exception:
                        pass
            if done:
                self._log("resuming %s: %d/%d points already done"
                          % (label, len(done), len(points)))
        results = []
        for i, pt in enumerate(points):
            key = str(pt)
            if key in done:
                results.append(done[key])
                continue
            if settle_s:
                time.sleep(settle_s)
            self._log("sweep %s point %d/%d: %s" % (label, i + 1, len(points), key))
            res = capture_fn(pt)
            rec = {"point": pt, "t": time.strftime("%Y-%m-%dT%H:%M:%S"), "result": res}
            with open(checkpoint_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec) + "\n")
            results.append(rec)
        return results


# --- DMX illuminant helper (PC-side agent) -----------------------------------
def set_dmx(d65=None, tungsten=None, agent=DMX_AGENT, timeout=6):
    """Set lab illuminant via the PC-side DMX agent. Returns True on success."""
    body = {}
    if d65 is not None:
        body["d65"] = int(d65)
    if tungsten is not None:
        body["tungsten"] = int(tungsten)
    try:
        req = urllib.request.Request(agent + "/set", data=json.dumps(body).encode(),
                                     method="POST",
                                     headers={"Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=timeout).read()
        return True
    except Exception:
        return False


# --- CLI ----------------------------------------------------------------------
def _main(argv):
    cmd = argv[1] if len(argv) > 1 else "health"
    iq9 = IQ9()
    if cmd == "health":
        print(json.dumps(iq9.health(), indent=2))
    elif cmd == "arm":
        iq9.arm_raw(); print(json.dumps(iq9.health(), indent=2))
    elif cmd == "disarm":
        iq9.disarm_raw(); print(json.dumps(iq9.health(), indent=2))
    elif cmd == "capture":
        r = iq9.capture_raw()
        print(json.dumps(r.get("stats", r) if isinstance(r, dict) else r, indent=2))
    elif cmd == "lightsweep":
        # DMX light sweep at fixed (uncontrolled) exposure -> checkpointed RAW capture.
        # This is the viable characterization path given exposure isn't settable on RDI.
        levels = [0, 16, 32, 64, 96, 128, 160, 192, 224, 255]
        ckpt = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lightsweep.jsonl")
        iq9.arm_raw()

        def cap(level):
            set_dmx(d65=level, tungsten=level)
            time.sleep(1.5)                       # let the lamp + scene settle
            r = iq9.save_raw(n_frames=4, label="d65_%03d" % level)
            return {"level": level, "file": r.get("file"),
                    "stats": r.get("stats")} if isinstance(r, dict) else {"level": level}

        res = iq9.sweep(levels, cap, ckpt, label="lightsweep", settle_s=0.5)
        print(json.dumps([x.get("result", x) for x in res], indent=2))
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv))
