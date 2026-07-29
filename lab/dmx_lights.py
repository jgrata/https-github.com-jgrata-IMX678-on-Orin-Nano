"""ENTTEC Open DMX USB (FTDI FT232R) controller for the Waveform Lighting 3082
LED illuminant panels -- PC-side lab lighting control.

Verified 2026-07-29 (camera-in-the-loop: scene brightness tracked ch5 0<->255).
The COM-port/pyserial break did NOT drive this rig; the working path is the FTDI
D2XX API (`ftd2xx`), the same QLC+ uses in "Open TX" mode.

DMX frame = BREAK (>=88us) + Mark-After-Break + start code 0x00 + 512 channels, at
250000 baud 8-N-2, streamed at ~11 Hz (the 3082 flashes at 44 Hz; QLC+ runs it at
11 Hz). The panel LATCHES the last frame, so a short burst sets a level and holds.

Channels are 1-indexed DMX addresses. This rig (3082 at DMX addr 001, Run1):
  ch4 = D65 (~6500K) panel,  ch5 = Tungsten (3200K) panel.

Exclusive with QLC+ -- only one process can own the FTDI at a time.

CLI:  python dmx_lights.py --tungsten 200 --d65 0
API:  with OpenDMX() as d: d.set_many({CH_D65:0, CH_TUNGSTEN:200}); time.sleep(1.5)
"""
import threading
import time

import ftd2xx

CH_D65 = 4
CH_TUNGSTEN = 5


class OpenDMX:
    def __init__(self, index=0):
        self.dev = ftd2xx.open(index)
        self.dev.setBaudRate(250000)
        self.dev.setDataCharacteristics(8, 2, 0)     # 8 data bits, 2 stop bits, no parity
        self.dev.setFlowControl(0, 0, 0)
        self.dev.setTimeouts(50, 50)
        try:
            self.dev.setLatencyTimer(1)               # tighten USB latency for frame timing
        except Exception:
            pass
        self.dev.purge(3)                             # RX | TX
        self.data = bytearray(513)                    # [0]=start code (0x00); [1..512]=channels
        self._lock = threading.Lock()
        self._run = False
        self._thr = None

    def set(self, channel, value):
        with self._lock:
            self.data[int(channel)] = max(0, min(255, int(value)))

    def set_many(self, mapping):
        with self._lock:
            for ch, v in mapping.items():
                self.data[int(ch)] = max(0, min(255, int(v)))

    def get(self, channel):
        with self._lock:
            return self.data[int(channel)]

    def _tx(self):
        self.dev.setBreakOn()
        time.sleep(0.0012)                            # break (>=88us)
        self.dev.setBreakOff()
        time.sleep(0.00015)                           # mark-after-break
        with self._lock:
            pkt = bytes(self.data)
        self.dev.write(pkt)

    def start(self, rate_hz=11.0):
        if self._run:
            return self
        self._run = True
        period = 1.0 / float(rate_hz)

        def loop():
            while self._run:
                t0 = time.monotonic()
                try:
                    self._tx()
                except Exception:
                    break
                dt = period - (time.monotonic() - t0)
                if dt > 0:
                    time.sleep(dt)
        self._thr = threading.Thread(target=loop, daemon=True)
        self._thr.start()
        return self

    def stop(self):
        self._run = False
        if self._thr:
            self._thr.join(timeout=1.0)

    def close(self):
        self.stop()
        try:
            self.dev.close()
        except Exception:
            pass

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Set Waveform-3082 illuminant levels (ENTTEC Open DMX USB)")
    ap.add_argument("--d65", type=int, default=None, help="channel 4 (D65) level 0-255")
    ap.add_argument("--tungsten", type=int, default=None, help="channel 5 (Tungsten) level 0-255")
    ap.add_argument("--hold", type=float, default=1.5, help="seconds to stream (panel latches the last frame)")
    ap.add_argument("--rate", type=float, default=11.0, help="DMX refresh Hz (44 flickers this rig; 11 is stable)")
    a = ap.parse_args()
    d = OpenDMX().start(rate_hz=a.rate)
    if a.d65 is not None:
        d.set(CH_D65, a.d65)
    if a.tungsten is not None:
        d.set(CH_TUNGSTEN, a.tungsten)
    time.sleep(a.hold)
    print("set D65(ch4)=%d  Tungsten(ch5)=%d  (held %.1fs; panel latches last frame)"
          % (d.get(CH_D65), d.get(CH_TUNGSTEN), a.hold))
    d.close()
