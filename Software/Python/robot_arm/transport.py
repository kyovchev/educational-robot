"""
Transport layer for STS3215Driver.

Both transports expose the same interface so the driver is agnostic
to the physical link (USB serial vs. Wi-Fi TCP socket).

Interface
---------
  connect()        -> bool
  disconnect()
  connected        -> bool  (property)
  write(data)
  read(n)          -> bytes
  reset_input_buffer()
"""

import socket
import time
from typing import Optional

import serial


# ─────────────────────────────────────────────────────────────────────────────
class SerialTransport:
    """USB / UART serial transport."""

    def __init__(self):
        self._ser: Optional[serial.Serial] = None

    def connect(self, port: str, baud: int = 1_000_000,
                timeout: float = 0.1) -> bool:
        try:
            self._ser = serial.Serial()
            self._ser.port = port
            self._ser.baudrate = baud
            self._ser.timeout = timeout
            self._ser.dtr = False   # must be set before open() to prevent Arduino reset
            self._ser.rts = False
            self._ser.open()
            time.sleep(0.5)             # wait for Arduino to finish booting
            self._ser.reset_input_buffer()
            return True
        except Exception as e:
            print(f"[SerialTransport] Connect error: {e}")
            return False

    def disconnect(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def write(self, data: bytes):
        if self._ser and self._ser.is_open:
            self._ser.write(data)

    def read(self, n: int) -> bytes:
        if self._ser and self._ser.is_open:
            return self._ser.read(n)
        return b""

    def reset_input_buffer(self):
        if self._ser and self._ser.is_open:
            self._ser.reset_input_buffer()

    def __repr__(self):
        return f"SerialTransport({self._ser.port if self._ser else 'disconnected'})"


# ─────────────────────────────────────────────────────────────────────────────
class TcpTransport:
    """Wi-Fi TCP transport — connects to an ESP32 running a raw TCP bridge."""

    DEFAULT_PORT = 8888
    RECV_TIMEOUT = 0.1

    def __init__(self):
        self._sock:      Optional[socket.socket] = None
        self._host:      str = ""
        self._tcp_port:  int = self.DEFAULT_PORT

    def connect(self, host: str, port: int = DEFAULT_PORT,
                timeout: float = 3.0) -> bool:
        try:
            self._host = host
            self._tcp_port = port
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            sock.connect((host, port))
            sock.settimeout(self.RECV_TIMEOUT)
            self._sock = sock
            return True
        except Exception as e:
            print(f"[TcpTransport] Connect error: {e}")
            self._sock = None
            return False

    def disconnect(self):
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def write(self, data: bytes):
        if self._sock:
            try:
                self._sock.sendall(data)
            except Exception as e:
                print(f"[TcpTransport] Write error: {e}")
                self._sock = None

    def read(self, n: int) -> bytes:
        if not self._sock:
            return b""
        buf = b""
        try:
            while len(buf) < n:
                chunk = self._sock.recv(n - len(buf))
                if not chunk:
                    break
                buf += chunk
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[TcpTransport] Read error: {e}")
            self._sock = None
        return buf

    def reset_input_buffer(self):
        if not self._sock:
            return
        try:
            self._sock.setblocking(False)
            while True:
                chunk = self._sock.recv(256)
                if not chunk:
                    break
        except (BlockingIOError, socket.error):
            pass
        finally:
            try:
                self._sock.settimeout(self.RECV_TIMEOUT)
            except Exception:
                pass

    def __repr__(self):
        if self._sock:
            return f"TcpTransport({self._host}:{self._tcp_port})"
        return "TcpTransport(disconnected)"
