#!/usr/bin/env python3
"""
image_server.py — True 10-bit RAW server for e-CAM86_CUONX
Jetson 10.70.0.27 → MATLAB port 9000
raw_capture --server on localhost port 9001
"""
import socket, struct, subprocess, numpy as np
import threading, time, os, signal, sys, json, re

SERVER_HOST      = '0.0.0.0'
SERVER_PORT      = 9000
RAW_CAPTURE_BIN  = '/home/metro/raw_capture/raw_capture'
RAW_CAPTURE_PORT = 9001
CAPTURE_DIR      = '/tmp/ecam_captures'

DEFAULT_SENSOR_MODE = 1
DEFAULT_FPS         = 30
DEFAULT_EXPOSURE_NS = 33000000
DEFAULT_GAIN        = 0.0
DEFAULT_BIT_DEPTH   = 10
DEFAULT_LOSSLESS    = True    # lossless-by-default; True => RAW10 packed

# Frame wire dtypes (status 0x00 frame header: [0x00][H u32][W u32][dtype u8])
DTYPE_U16      = 0x10   # uint16 unpacked  (16.6 MB / 4K frame)
DTYPE_RAW10    = 0x11   # RAW10 bit-packed (9.5 MB / 4K frame, lossless)

HDR_SAT_THRESHOLD  = 0.95
HDR_EXPOSURE_RATIO = 4

SENSOR_MODES = {
    0: {'width':3840,'height':2160,'bpp':12,'hdr':False},
    1: {'width':3840,'height':2160,'bpp':10,'hdr':False},
    2: {'width':1920,'height':1080,'bpp':12,'hdr':False},
    3: {'width':3840,'height':2160,'bpp':10,'hdr':True},
}

CMD_CAPTURE    = 0x01
CMD_STREAM_ON  = 0x02
CMD_STREAM_OFF = 0x03
CMD_SET_PARAMS = 0x04
CMD_GET_INFO   = 0x05
CMD_PING       = 0x06


def ensure_dir(p):
    os.makedirs(p, exist_ok=True)

def send_exactly(sock, data):
    total = 0; mv = memoryview(data)
    while total < len(data):
        n = sock.send(mv[total:])
        if n == 0: raise ConnectionError("closed")
        total += n

def recv_exactly(sock, n):
    buf = bytearray()
    while len(buf) < n:
        c = sock.recv(n - len(buf))
        if not c: raise ConnectionError("closed")
        buf.extend(c)
    return bytes(buf)

def nvargus_restart():
    print("[nvargus] Restarting...")
    subprocess.run(['sudo','systemctl','restart','nvargus-daemon'],
                   capture_output=True, timeout=15)
    time.sleep(3)
    r = subprocess.run(['systemctl','is-active','nvargus-daemon'],
                       capture_output=True, text=True)
    print("[nvargus] " + r.stdout.strip())
    return r.stdout.strip() == 'active'

def read_actual_sensor_params():
    try:
        r = subprocess.run(
            ['v4l2-ctl','-d','/dev/video0','--get-ctrl=exposure,gain'],
            capture_output=True, text=True, timeout=3)
        exp_ns = 0; gain = 0.0
        for line in r.stdout.splitlines():
            low = line.lower().strip()
            if low.startswith('exposure:'):
                m = re.search(r'(\d+)', low)
                if m: exp_ns = int(m.group(1)) * 1000
            elif low.startswith('gain:'):
                m = re.search(r'(\d+)', low)
                if m:
                    gain_db = int(m.group(1)) * 0.1
                    gain = 10.0 ** (gain_db / 20.0)
        return exp_ns, gain
    except Exception:
        return 0, 0.0


# ── RAW10 packing (reduces 16.6MB → 9.5MB) ───────────────────────────────────

def pack_raw10(frame_u16):
    flat = frame_u16.flatten().astype(np.uint16)
    rem  = len(flat) % 4
    if rem: flat = np.append(flat, np.zeros(4-rem, dtype=np.uint16))
    px       = flat.reshape(-1, 4)
    out      = np.empty((len(px), 5), dtype=np.uint8)
    out[:,0] = (px[:,0] >> 2).astype(np.uint8)
    out[:,1] = (px[:,1] >> 2).astype(np.uint8)
    out[:,2] = (px[:,2] >> 2).astype(np.uint8)
    out[:,3] = (px[:,3] >> 2).astype(np.uint8)
    out[:,4] = ((px[:,0]&3) | ((px[:,1]&3)<<2) |
                ((px[:,2]&3)<<4) | ((px[:,3]&3)<<6)).astype(np.uint8)
    return out.tobytes()


# ── RawCaptureProcess ─────────────────────────────────────────────────────────

import struct as _struct

# raw_capture localhost request protocol (must match ReqHdr in raw_capture.cpp)
_REQ_FMT        = '<IQfI'          # cmd u32, want_exp u64, want_gain f32, pad u32
_REQ_FRAME      = 1
_REQ_SET_EXPGAIN= 2
_REQ_PING       = 3


class RawCaptureProcess:
    """Manages raw_capture --server as a persistent background process,
    and holds ONE persistent localhost connection to it (reused for every
    frame) instead of reconnecting per capture."""

    def __init__(self):
        self.proc        = None
        self.lock        = threading.Lock()
        self.ready       = False
        self.sensor_mode = DEFAULT_SENSOR_MODE
        self.fps         = DEFAULT_FPS
        self.exposure_ns = DEFAULT_EXPOSURE_NS
        self.gain        = DEFAULT_GAIN
        self._sock       = None        # persistent connection to raw_capture

    def _build_cmd(self):
        # Pass the FULL current state every launch so a restart for one
        # parameter (e.g. sensormode) never silently resets the others.
        cmd = [RAW_CAPTURE_BIN,'--server',
               '--port', str(RAW_CAPTURE_PORT),
               '--mode', str(self.sensor_mode),
               '--fps',  str(self.fps)]
        if self.exposure_ns > 0:
            cmd += ['--exposure', str(int(self.exposure_ns))]
        if self.gain > 0.0:
            cmd += ['--gain', '{:.6f}'.format(self.gain)]
        return cmd

    def start(self):
        self.stop()
        self.ready = False
        cmd = self._build_cmd()
        print("[RCP] Starting: " + ' '.join(cmd))
        self.proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True)
        threading.Thread(target=self._log, daemon=True).start()
        print("[RCP] Waiting for Argus session init (~7s)...")
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            time.sleep(0.5)
            if not self.is_alive():
                print("[RCP] Process died during init")
                return False
            if self.ready:
                print("[RCP] Ready (detected from stdout)")
                return True
        if self.is_alive():
            print("[RCP] Assuming ready after timeout (process alive)")
            self.ready = True
            return True
        return False

    def _log(self):
        if self.proc and self.proc.stdout:
            for line in self.proc.stdout:
                s = line.rstrip()
                if s: print("  [rcp] " + s)
                if ('fast captures' in s.lower() or
                        'ready' in s.lower() and 'port' in s.lower()):
                    self.ready = True

    def stop(self):
        self.ready = False
        self._close_sock()
        if self.proc:
            try: self.proc.terminate(); self.proc.wait(timeout=5)
            except:
                try: self.proc.kill()
                except: pass
            self.proc = None

    def is_alive(self):
        return self.proc is not None and self.proc.poll() is None

    def _close_sock(self):
        if self._sock is not None:
            try: self._sock.close()
            except: pass
            self._sock = None

    def _connect(self):
        """Open (or reopen) the persistent localhost connection."""
        self._close_sock()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8*1024*1024)
        except: pass
        s.settimeout(60)
        s.connect(('127.0.0.1', RAW_CAPTURE_PORT))
        self._sock = s

    def restart_with_params(self):
        """Restart raw_capture carrying the FULL current state. Used only for
        changes Argus can't apply live (sensor mode, fps)."""
        with self.lock:
            nvargus_restart()
            time.sleep(2)
            if not self.start():
                raise RuntimeError("raw_capture restart failed")

    def set_expgain_live(self, exposure_ns, gain):
        """Push a live exposure/gain change to raw_capture over the persistent
        connection — no process restart. Safe for exp/gain-only changes."""
        with self.lock:
            self.exposure_ns = int(exposure_ns)
            self.gain        = float(gain)
            if self._sock is None:
                # No live channel yet; values are stashed and will apply
                # on next (re)connect / launch args.
                return
            req = _struct.pack(_REQ_FMT, _REQ_SET_EXPGAIN,
                               int(exposure_ns), float(gain), 0)
            try:
                self._sock.sendall(req)
                ack = self._recv_fast(self._sock, 4)  # 4-byte ack
                _ = _struct.unpack('<I', ack)[0]
            except Exception as e:
                # Live channel broke — drop it, next capture reconnects.
                self._close_sock()
                print("[RCP] live set failed, will reconnect: " + str(e))

    def capture(self):
        """Request one frame over the persistent connection. Reconnects
        transparently if the socket is not yet open or was dropped."""
        with self.lock:
            if not self.is_alive() or not self.ready:
                print("[RCP] Not ready — restarting...")
                nvargus_restart()
                time.sleep(2)
                if not self.start():
                    raise RuntimeError("raw_capture server failed to start")

            # (Re)establish persistent connection if needed.
            if self._sock is None:
                try:
                    self._connect()
                except Exception as e:
                    self.ready = False
                    raise RuntimeError("Cannot connect to raw_capture: " + str(e))

            try:
                return self._request_frame()
            except (ConnectionError, OSError) as e:
                # One transparent retry after reconnect.
                print("[RCP] frame req failed (" + str(e) + "), reconnecting...")
                try:
                    self._connect()
                    return self._request_frame()
                except Exception as e2:
                    self._close_sock()
                    self.ready = False
                    raise RuntimeError("raw_capture frame failed: " + str(e2))

    def _request_frame(self):
        s = self._sock
        s.sendall(_struct.pack(_REQ_FMT, _REQ_FRAME, 0, 0.0, 0))

        # NetHdr: 36 bytes — magic(4) w(4) h(4) bpp(4) nf(4)
        #                     exp_ns(8) gain_x1000(4) pad(4)
        hdr_bytes = self._recv_fast(s, 36)
        magic, w, h, bpp, nf, exp_ns, gain_x1000, pad = \
            struct.unpack('<IIIIIQII', hdr_bytes)

        if magic == 0xDEADBEEF:
            self.ready = False
            raise RuntimeError("raw_capture returned error")
        if magic != 0x52413130:
            raise RuntimeError("Bad magic: " + hex(magic))

        total_px = w * h * nf
        n_bytes  = total_px * 2

        pixel_arr = np.empty(total_px, dtype=np.uint16)
        view = pixel_arr.view(np.uint8)
        received = 0
        while received < n_bytes:
            got = s.recv_into(view[received:], n_bytes - received)
            if got == 0: raise ConnectionError("raw_capture closed")
            received += got

        return pixel_arr, w, h, bpp, nf, exp_ns, gain_x1000

    @staticmethod
    def _recv_fast(sock, n):
        buf = bytearray(n)
        view = memoryview(buf)
        total = 0
        while total < n:
            got = sock.recv_into(view[total:], n - total)
            if got == 0: raise ConnectionError("closed")
            total += got
        return bytes(buf)


# ── Camera ────────────────────────────────────────────────────────────────────

class Camera:

    def __init__(self, rcp,
                 sensor_mode=DEFAULT_SENSOR_MODE,
                 fps=DEFAULT_FPS,
                 exposure_ns=DEFAULT_EXPOSURE_NS,
                 gain=DEFAULT_GAIN,
                 bit_depth=DEFAULT_BIT_DEPTH,
                 lossless=DEFAULT_LOSSLESS):
        self.rcp         = rcp
        self.sensor_mode = sensor_mode
        self.fps         = fps
        self.exposure_ns = exposure_ns
        self.gain        = gain
        self.bit_depth   = bit_depth
        self.lossless    = lossless
        self.actual_exp  = 0
        self.actual_gain = 0.0
        self._ae_lock    = threading.Lock()
        self._refresh()
        ensure_dir(CAPTURE_DIR)
        threading.Thread(target=self._ae_monitor, daemon=True).start()

    def _refresh(self):
        m = SENSOR_MODES[self.sensor_mode]
        self.width      = m['width']
        self.height     = m['height']
        self.native_bpp = m['bpp']
        self.hdr        = m['hdr']

    def _ae_monitor(self):
        time.sleep(8)
        while True:
            time.sleep(3)
            exp, g = read_actual_sensor_params()
            if exp > 0:
                with self._ae_lock: self.actual_exp = exp
            if g > 0:
                with self._ae_lock: self.actual_gain = g
            elif self.gain > 0:
                with self._ae_lock: self.actual_gain = self.gain
            if self.exposure_ns > 0:
                with self._ae_lock: self.actual_exp = self.exposure_ns

    def capture_frame(self):
        if self.hdr:
            return self._capture_hdr()
        pixels, w, h, bpp, nf, ae, ag = self.rcp.capture()
        with self._ae_lock:
            if ae > 0: self.actual_exp  = ae
            if ag > 0: self.actual_gain = ag / 1000.0
        frame = pixels[:w*h].reshape(h, w)
        return self._scale(frame)

    def _scale(self, frame):
        bd = self.bit_depth
        if bd == 8:
            return ((frame.astype(np.float32)*255/1023)
                    .clip(0,255).astype(np.uint16))
        if bd == 12:
            return (frame.astype(np.uint32)*4).clip(0,4095).astype(np.uint16)
        return frame  # bd==10: no change

    def _capture_hdr(self):
        long_ns  = self.exposure_ns if self.exposure_ns > 0 else 16000000
        short_ns = max(450000, long_ns // HDR_EXPOSURE_RATIO)
        orig = self.rcp.exposure_ns
        self.rcp.exposure_ns = long_ns
        px,w,h,bpp,_,_,_ = self.rcp.capture()
        long_frame = px[:w*h].reshape(h,w)
        self.rcp.exposure_ns = short_ns
        px,w,h,bpp,_,_,_ = self.rcp.capture()
        short_frame = px[:w*h].reshape(h,w)
        self.rcp.exposure_ns = orig
        l = long_frame.astype(np.float32)
        s = short_frame.astype(np.float32)
        lm = float(l.max()) if l.max()>0 else 1.0
        sm = float(s.max()) if s.max()>0 else 1.0
        sat = (l/lm) >= HDR_SAT_THRESHOLD
        merged = (np.where(sat,s/sm,l/lm)*1023).clip(0,1023).astype(np.uint16)
        return self._scale(merged)

    def set_params(self, params):
        changed = []
        restart = False          # only True for mode/fps (needs session rebuild)
        exp_gain_touched = False # exposure/gain changed -> live update
        if 'sensormode' in params:
            m = int(params['sensormode'])
            if m not in SENSOR_MODES: raise ValueError("bad sensormode")
            self.sensor_mode = m; self._refresh()
            self.rcp.sensor_mode = m; changed.append("sensormode="+str(m))
            restart = True
        if 'fps' in params:
            self.fps = int(params['fps']); self.rcp.fps = self.fps
            changed.append("fps="+str(self.fps)); restart = True
        if 'exposure_ns' in params:
            self.exposure_ns = int(params['exposure_ns'])
            self.rcp.exposure_ns = self.exposure_ns
            with self._ae_lock: self.actual_exp = 0
            changed.append("exp_ns="+str(self.exposure_ns))
            exp_gain_touched = True
        if 'gain' in params:
            self.gain = float(params['gain']); self.rcp.gain = self.gain
            with self._ae_lock: self.actual_gain = 0.0
            changed.append("gain="+'{:.3f}'.format(self.gain))
            exp_gain_touched = True
        if 'bit_depth' in params:
            bd = int(params['bit_depth'])
            if bd not in (8,10,12): raise ValueError("bit_depth 8/10/12")
            self.bit_depth = bd; changed.append("bit_depth="+str(bd))
        if 'lossless' in params:
            self.lossless = bool(params['lossless'])
            changed.append("lossless="+str(self.lossless))
        if 'sat_threshold' in params:
            global HDR_SAT_THRESHOLD
            HDR_SAT_THRESHOLD = float(params['sat_threshold'])
            changed.append("sat="+'{:.2f}'.format(HDR_SAT_THRESHOLD))
        if 'hdr_exp_ratio' in params:
            global HDR_EXPOSURE_RATIO
            HDR_EXPOSURE_RATIO = int(params['hdr_exp_ratio'])
            changed.append("ratio="+str(HDR_EXPOSURE_RATIO))

        # Only the requested setting changes:
        #  - mode/fps  -> full restart, but restart carries ALL persisted params
        #  - exp/gain  -> live update on the running pipeline (no restart),
        #                 unless a restart is already happening for mode/fps
        #                 (in which case the new exp/gain go in via launch args)
        if restart and changed:
            def do_restart():
                print("[Camera] Restarting pipeline: " + str(changed))
                self.rcp.restart_with_params()
            threading.Thread(target=do_restart, daemon=True).start()
        elif exp_gain_touched:
            # Live path — apply immediately without disturbing mode/fps.
            self.rcp.set_expgain_live(self.exposure_ns, self.gain)
        return changed

    def info(self):
        self._refresh()
        with self._ae_lock: ae = self.actual_exp; ag = self.actual_gain
        exp_d = ae if (self.exposure_ns==0 and ae>0) else self.exposure_ns
        return {
            'width':              self.width,
            'height':             self.height,
            'fps':                self.fps,
            'bit_depth':          self.bit_depth,
            'lossless':           self.lossless,
            'native_bpp':         self.native_bpp,
            'hdr':                self.hdr,
            'sensormode':         self.sensor_mode,
            'exposure_ns':        exp_d,
            'exposure_ns_set':    self.exposure_ns,
            'actual_exposure_ns': ae,
            'gain':               self.gain,
            'gain_set':           self.gain,
            'actual_gain':        round(ag,4),
            'hdr_exp_ratio':      HDR_EXPOSURE_RATIO,
            'sat_threshold':      HDR_SAT_THRESHOLD,
            'sensor':             'e-CAM86_CUONX (IMX678)',
            'bayer':              'RGGB',
            'method':             'CUDA EGL RAW16 TRUE 10-bit (persistent session)',
            'note':               str(self.bit_depth)+'-bit RAW | ~0.5s/frame',
            'pipeline_running':   self.rcp.is_alive(),
        }


# ── ClientHandler ─────────────────────────────────────────────────────────────

class ClientHandler(threading.Thread):

    def __init__(self, conn, addr, camera):
        super().__init__(daemon=True)
        self.conn = conn; self.addr = addr
        self.camera = camera; self.streaming = False

    def run(self):
        print("[Server] Connected: " + str(self.addr))
        try:
            while True:
                hdr = recv_exactly(self.conn, 8)
                cmd, pl = struct.unpack('<II', hdr)
                pay = recv_exactly(self.conn, pl) if pl else b''
                self._dispatch(cmd, pay)
        except (ConnectionError, OSError) as e:
            print("[Server] Disconnected: "+str(self.addr)+" ("+str(e)+")")
        finally:
            self.streaming = False
            try: self.conn.close()
            except: pass

    def _dispatch(self, cmd, pay):
        {
            CMD_PING:       lambda: self._resp(b'PONG'),
            CMD_GET_INFO:   lambda: self._resp(
                json.dumps(self.camera.info()).encode()),
            CMD_CAPTURE:    self._capture,
            CMD_SET_PARAMS: lambda: self._set_params(pay),
            CMD_STREAM_ON:  self._stream_on,
            CMD_STREAM_OFF: self._stream_off,
        }.get(cmd, lambda: self._err("unknown "+hex(cmd)))()

    def _set_params(self, pay):
        try:
            changed = self.camera.set_params(json.loads(pay.decode()))
            if changed: print("[Server] Params: "+str(changed))
            self._resp(b'OK')
        except Exception as e:
            self._err(str(e))

    def _capture(self):
        try:
            t0    = time.monotonic()
            frame = self.camera.capture_frame()
            t_cap = time.monotonic()-t0
            self._send_frame(frame)
            t_tot = time.monotonic()-t0
            print("[Server] cap={:.2f}s net={:.2f}s total={:.2f}s".format(
                t_cap, t_tot-t_cap, t_tot))
        except Exception as e:
            print("[Server] Capture error: "+str(e))
            self._err(str(e))

    def _stream_on(self):
        self.streaming = True
        self._resp(b'STREAMING')
        period = 1.0 / self.camera.fps
        while self.streaming:
            t0 = time.monotonic()
            try:
                self._send_frame(self.camera.capture_frame())
            except Exception as e:
                self._err(str(e)); break
            time.sleep(max(0.0, period-(time.monotonic()-t0)))

    def _stream_off(self):
        self.streaming = False
        self._resp(b'STOPPED')

    def _send_frame(self, frame):
        """Send one frame. Lossless-by-default:
          lossless + 10-bit  -> RAW10 bit-packed (dtype 0x11, 9.5MB/4K)
          otherwise          -> uint16 unpacked  (dtype 0x10, 16.6MB/4K)
        RAW10 packing is only valid for true 10-bit data (0-1023); for 8/12-bit
        scaled output we fall back to lossless uint16 so no bits are lost."""
        h, w = frame.shape
        use_raw10 = bool(getattr(self.camera, 'lossless', True)) and \
                    self.camera.bit_depth == 10
        if use_raw10:
            packed = pack_raw10(frame)
            header = struct.pack('<BIIB', 0x00, h, w, DTYPE_RAW10)
            send_exactly(self.conn, header + packed)
        else:
            pixels = frame.astype(np.uint16).tobytes()
            header = struct.pack('<BIIB', 0x00, h, w, DTYPE_U16)
            send_exactly(self.conn, header + pixels)

    def _resp(self, data):
        send_exactly(self.conn,
                     struct.pack('<BI', 0x00, len(data)) + data)

    def _err(self, msg):
        d = msg.encode()
        send_exactly(self.conn,
                     struct.pack('<BI', 0xFF, len(d)) + d)
        print("[Server] Error -> "+str(self.addr)+": "+msg)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ensure_dir(CAPTURE_DIR)
    try:
        subprocess.run(['fuser','-k',str(SERVER_PORT)+'/tcp'],
                       capture_output=True, timeout=5)
        time.sleep(0.5)
    except: pass

    print("[Init] Restarting nvargus-daemon...")
    subprocess.run(['sudo','systemctl','restart','nvargus-daemon'],
                   capture_output=True, timeout=15)
    time.sleep(3)
    r = subprocess.run(['systemctl','is-active','nvargus-daemon'],
                       capture_output=True, text=True)
    print("[Init] nvargus-daemon: "+r.stdout.strip())

    rcp = RawCaptureProcess()
    rcp.sensor_mode  = DEFAULT_SENSOR_MODE
    rcp.fps          = DEFAULT_FPS
    rcp.exposure_ns  = DEFAULT_EXPOSURE_NS
    rcp.gain         = DEFAULT_GAIN

    print("[Init] Starting raw_capture --server (first init ~7s)...")
    if not rcp.start():
        print("[Init] WARNING: raw_capture not ready — will retry on first capture")

    camera = Camera(rcp,
        sensor_mode = DEFAULT_SENSOR_MODE,
        fps         = DEFAULT_FPS,
        exposure_ns = DEFAULT_EXPOSURE_NS,
        gain        = DEFAULT_GAIN,
        bit_depth   = DEFAULT_BIT_DEPTH)

    print("\n[Server] Camera info:")
    for k,v in camera.info().items():
        print("  "+str(k).ljust(22)+": "+str(v))

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try: srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except: pass

    for attempt in range(5):
        try: srv.bind((SERVER_HOST, SERVER_PORT)); break
        except OSError:
            if attempt<4: time.sleep(2)
            else: raise

    srv.listen(4)
    print("\n[Server] Listening on "+SERVER_HOST+":"+str(SERVER_PORT))
    print("[Server] Ready.\n")

    def _stop(s,f):
        rcp.stop(); srv.close(); sys.exit(0)
    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    while True:
        conn, addr = srv.accept()
        try:
            conn.setsockopt(socket.SOL_SOCKET,
                            socket.SO_SNDBUF, 4*1024*1024)
        except: pass
        ClientHandler(conn, addr, camera).start()


if __name__ == '__main__':
    main()
