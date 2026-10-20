# test_udp_diagnostics.py ver 1
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from udp_diagnostics import PacketDiagnostics, sequence_delta_u32

assert sequence_delta_u32(0, 0xFFFFFFFF) == 1
assert sequence_delta_u32(12, 10) == 2

d = PacketDiagnostics()
s = d.update({"sequence": 100, "timestamp_ms": 1000}, 10.000, 1)
assert s.sequence_delta is None and s.receiver_count_delta is None
s = d.update({"sequence": 101, "timestamp_ms": 1050}, 10.051, 2)
assert s.sequence_delta == 1
assert s.receiver_count_delta == 1
assert abs(s.local_dt_s - 0.051) < 1e-9
assert abs(s.source_dt_s - 0.050) < 1e-9
assert s.render_skipped_packets == 0 and s.source_missing_packets == 0
# Three packets reached the receiver since the last processed snapshot, so two
# were superseded before the render thread got to them.
s = d.update({"sequence": 104, "timestamp_ms": 1200}, 10.201, 5)
assert s.sequence_delta == 3 and s.receiver_count_delta == 3
assert s.render_skipped_packets == 2 and s.source_missing_packets == 0
# Only one new datagram received but POC sequence advanced by two: one missing
# upstream/network packet.
s = d.update({"sequence": 106, "timestamp_ms": 1300}, 10.301, 6)
assert s.sequence_delta == 2 and s.receiver_count_delta == 1
assert s.source_missing_packets == 1
print("PASS: UDP packet timing/sequence diagnostics")
