"""Thin client to image_server.py's TCP protocol (the one transport-aware module;
port this for the IQ9). Mirrors ECamHDRClient's wire format:

  request  = <II>(cmd, payload_len) + payload
  _resp    = <BI>(status, len) + data           (info / OK / errors)
  capture  = <BIIB>(status, H, W, dtype) + packed frame bytes
"""
import json
import socket
import struct

import numpy as np

CMD_CAPTURE = 0x01
CMD_SET_PARAMS = 0x04
CMD_GET_INFO = 0x05
CMD_PING = 0x06

DTYPE_U16 = 0x10
DTYPE_RAW10 = 0x11
DTYPE_RAW12 = 0x12


def _recv(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(1 << 20, n - len(buf)))
        if not chunk:
            raise ConnectionError("socket closed with %d/%d bytes" % (len(buf), n))
        buf += chunk
    return bytes(buf)


def unpack_raw12(packed, h, w):
    n = h * w
    ng = (n + 1) // 2
    b = np.frombuffer(packed[:ng * 3], np.uint8).reshape(ng, 3).astype(np.uint16)
    p0 = (b[:, 0] << 4) | (b[:, 2] & 0x0F)
    p1 = (b[:, 1] << 4) | ((b[:, 2] >> 4) & 0x0F)
    flat = np.empty(ng * 2, np.uint16)
    flat[0::2] = p0
    flat[1::2] = p1
    return flat[:n].reshape(h, w)


def unpack_raw10(packed, h, w):
    n = h * w
    ng = (n + 3) // 4
    b = np.frombuffer(packed[:ng * 5], np.uint8).reshape(ng, 5).astype(np.uint16)
    lo = b[:, 4]
    flat = np.empty(ng * 4, np.uint16)
    for i in range(4):
        flat[i::4] = (b[:, i] << 2) | ((lo >> (2 * i)) & 0x03)
    return flat[:n].reshape(h, w)


class CameraClient:
    """One connection per instance. Not thread-safe: give each stream/request its
    own client (image_server handles many concurrent connections)."""

    def __init__(self, host="127.0.0.1", port=9000, timeout=20.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        return self

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()

    def _read_resp(self):
        status, ln = struct.unpack("<BI", _recv(self.sock, 5))
        data = _recv(self.sock, ln) if ln else b""
        return status, data

    def info(self):
        self.sock.sendall(struct.pack("<II", CMD_GET_INFO, 0))
        status, data = self._read_resp()
        return json.loads(data.decode())

    def set_params(self, params):
        pay = json.dumps(params).encode()
        self.sock.sendall(struct.pack("<II", CMD_SET_PARAMS, len(pay)) + pay)
        status, data = self._read_resp()          # server replies b'OK'
        return status == 0x00

    def capture(self):
        """Return (frame uint16 [H,W] Bayer, maxv). Handles RAW10/RAW12/U16."""
        self.sock.sendall(struct.pack("<II", CMD_CAPTURE, 0))
        status, H, W, dtype = struct.unpack("<BIIB", _recv(self.sock, 10))
        if dtype == DTYPE_RAW12:
            frame = unpack_raw12(_recv(self.sock, ((H * W + 1) // 2) * 3), H, W)
            return frame, 4095.0
        if dtype == DTYPE_RAW10:
            frame = unpack_raw10(_recv(self.sock, ((H * W + 3) // 4) * 5), H, W)
            return frame, 1023.0
        if dtype == DTYPE_U16:
            frame = np.frombuffer(_recv(self.sock, H * W * 2), np.uint16).reshape(H, W).copy()
            return frame, 65535.0
        raise ValueError("unexpected frame dtype 0x%02x" % dtype)
