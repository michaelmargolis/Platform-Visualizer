# easyip_receiver.py ver 1
"""Receive POC/FestoSender EasyIP pressure writes (UDP 995), without replying.

Only the six commanded muscle pressures are used for motion.  An optional
seventh FLAG_WORD is preserved as ``prop_command_word``: POC currently sends
0 for prop engage and 1 for prop disengage.  It is a *command*, not feedback.

The wire layout is the 20-byte little-endian EasyIP header used by POC v149's
output/fstlib/easyip.py.  Keeping this read-only decoder self-contained avoids
copying the complete Festo controller/emulator into the visualizer.
"""
from __future__ import annotations

import socket
import struct
import threading
import time

HEADER = struct.Struct("<BBHHBBHHBBHHH")
FLAG_WORD = 1
RESPONSE_FLAG = 0x80
MAX_PRESSURE_MBAR = 6000


def decode_pressure_write(data: bytes) -> dict:
    """Decode a complete EasyIP send FLAG_WORD packet; reject other messages."""
    if len(data) < HEADER.size:
        raise ValueError("EasyIP datagram shorter than 20-byte header")
    (flags, error, counter, _index1, _spare1, send_type, send_size,
     send_offset, _spare2, _req_type, _req_size, _req_offset, _client_offset) = HEADER.unpack_from(data)
    if flags & RESPONSE_FLAG:
        raise ValueError("EasyIP response packet (not a pressure command)")
    if error:
        raise ValueError(f"EasyIP error field is {error}")
    if send_type != FLAG_WORD or send_offset != 0 or send_size not in (6, 7):
        raise ValueError(
            f"not a six/seven-word EasyIP pressure write: "
            f"type={send_type}, offset={send_offset}, count={send_size}"
        )
    expected_length = HEADER.size + send_size * 2
    if len(data) != expected_length:
        raise ValueError(f"EasyIP packet length {len(data)}, expected {expected_length}")
    words = struct.unpack_from("<" + "H" * send_size, data, HEADER.size)
    pressures = list(words[:6])
    if any(p > MAX_PRESSURE_MBAR for p in pressures):
        raise ValueError("EasyIP commanded muscle pressure exceeds 6000 mbar")
    prop_word = words[6] if send_size == 7 else None
    if prop_word is not None and prop_word not in (0, 1):
        raise ValueError(f"EasyIP prop command word must be 0 or 1: {prop_word}")
    return {
        "command_pressure_mbar": pressures,
        "prop_command_word": prop_word,
        "easyip_counter": counter,
    }


class EasyIpReceiver:
    """TwinUdpReceiver-compatible latest-packet interface; no EasyIP replies."""

    def __init__(self, port=995):
        self.port = int(port)
        self.latest = None
        self.latest_arrival = 0.0
        self.packet_count = 0
        self.datagram_count = 0
        self.rejected_count = 0
        self.last_sender = None
        self.last_reject = ""
        self.last_error = ""
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.25)
        try:
            self._sock.bind(("0.0.0.0", self.port))
        except OSError:
            self._sock.close()
            raise
        self._thread = threading.Thread(target=self._loop, name="EasyIP UDP listener", daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                data, sender = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            with self._lock:
                self.datagram_count += 1
                self.last_sender = sender
            try:
                msg = decode_pressure_write(data)
            except (ValueError, struct.error) as exc:
                with self._lock:
                    self.rejected_count += 1
                    self.last_error = str(exc)
                    self.last_reject = str(exc)
                continue
            with self._lock:
                self.latest = msg
                self.latest_arrival = time.monotonic()
                self.packet_count += 1
                self.last_error = ""
                self.last_reject = ""

    def snapshot(self):
        with self._lock:
            return self.latest, self.latest_arrival, self.packet_count, self.last_error

    def diagnostics(self):
        with self._lock:
            return {
                "datagram_count": self.datagram_count,
                "valid_count": self.packet_count,
                "rejected_count": self.rejected_count,
                "last_sender": self.last_sender,
                "last_reject": self.last_reject,
            }

    def close(self):
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass
        self._thread.join(timeout=0.5)
