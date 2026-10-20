# twin_udp.py ver 5
"""POC_TWIN UDP receiver. Keeps only the newest valid complete packet."""
from __future__ import annotations
import json
import socket
import threading
import time
import numpy as np


class TwinUdpReceiver:
    def __init__(self, port=10020):
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
        self._sock.bind(("0.0.0.0", self.port))
        self._thread = threading.Thread(target=self._loop, name="POC Twin UDP", daemon=True)
        self._thread.start()

    @staticmethod
    def _six_finite(msg, field):
        values = msg.get(field)
        if not isinstance(values, list) or len(values) != 6:
            raise ValueError(f"{field} must contain six values")
        values = np.asarray(values, dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError(f"{field} contains a non-finite value")
        return values

    def _loop(self):
        while not self._stop.is_set():
            try:
                data, sender = self._sock.recvfrom(2048)
                with self._lock:
                    self.datagram_count += 1
                    self.last_sender = sender
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                if not data:
                    raise ValueError("empty UDP datagram")
                if len(data) > 1400:
                    raise ValueError(f"UDP datagram too large: {len(data)} bytes")
                msg = json.loads(data.decode("utf-8"))
                if msg.get("protocol") != "POC_TWIN":
                    raise ValueError(f"unexpected protocol: {msg.get('protocol')!r}")
                if msg.get("version") != 1:
                    raise ValueError(f"unsupported POC_TWIN version: {msg.get('version')!r}")
                msg["muscle_length_mm"] = self._six_finite(msg, "muscle_length_mm")
                msg["command_pressure_mbar"] = self._six_finite(msg, "command_pressure_mbar")
                if msg.get("applied_transform_real") is not None:
                    msg["applied_transform_real"] = self._six_finite(msg, "applied_transform_real")
                if "parking_state" in msg and msg["parking_state"] not in (
                    "PARKED", "UNPARKED", "PARKING", "UNPARKING"
                ):
                    raise ValueError("invalid parking_state")
                with self._lock:
                    self.latest = msg
                    self.latest_arrival = time.monotonic()
                    self.packet_count += 1
                    self.last_error = ""
                    self.last_reject = ""
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                with self._lock:
                    self.rejected_count += 1
                    self.last_error = str(exc)
                    self.last_reject = str(exc)

    def snapshot(self):
        with self._lock:
            return self.latest, self.latest_arrival, self.packet_count, self.last_error

    def diagnostics(self):
        """Return receiver-level counters without changing snapshot() compatibility."""
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


def _raw_port_test(port=10020, truncate=220):
    """Bind the twin UDP port and print the start of every received datagram."""
    port = int(port)
    truncate = max(20, int(truncate))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    print(f"Listening for UDP datagrams on 0.0.0.0:{port}")
    print("Press Ctrl+C to stop.")
    try:
        while True:
            data, sender = sock.recvfrom(65535)
            text = data.decode("utf-8", errors="replace")
            text = text.replace("\r", "\\r").replace("\n", "\\n")
            if len(text) > truncate:
                text = text[:truncate] + "..."
            stamp = time.strftime("%H:%M:%S")
            print(f"{stamp} {sender[0]}:{sender[1]} {len(data)} bytes  {text}", flush=True)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Listen on the POC_TWIN UDP port and print the start of each raw datagram."
    )
    parser.add_argument("--port", type=int, default=10020, help="UDP port to listen on (default: 10020)")
    parser.add_argument(
        "--truncate", type=int, default=220,
        help="Maximum decoded characters to print per datagram (default: 220)",
    )
    args = parser.parse_args()
    _raw_port_test(args.port, args.truncate)


if __name__ == "__main__":
    main()
