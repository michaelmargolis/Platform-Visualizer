# udp_diagnostics.py ver 1
"""Pure-Python timing/sequence diagnostics for processed POC_TWIN packets."""
from __future__ import annotations

from dataclasses import dataclass

UINT32_MOD = 1 << 32


def sequence_delta_u32(current, previous):
    """Return unsigned 32-bit sequence advance, including wrap, or None."""
    if current is None or previous is None:
        return None
    return (int(current) - int(previous)) % UINT32_MOD


@dataclass
class PacketDiagnosticsSnapshot:
    sequence: int | None
    sequence_delta: int | None
    receiver_count_delta: int | None
    local_dt_s: float | None
    source_dt_s: float | None
    render_skipped_packets: int
    source_missing_packets: int
    render_skipped_total: int
    source_missing_total: int


class PacketDiagnostics:
    """Track gaps and timing between packets actually processed by the visualizer.

    TwinUdpReceiver intentionally retains only the newest complete snapshot.  If
    rendering/solving falls behind, receiver_count_delta > 1 reveals that one or
    more already-received packets were superseded before processing.  Comparing
    sequence_delta with receiver_count_delta also exposes sequence gaps upstream
    of the visualizer receiver.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.previous_sequence = None
        self.previous_receiver_count = None
        self.previous_arrival = None
        self.previous_timestamp_ms = None
        self.render_skipped_total = 0
        self.source_missing_total = 0

    def update(self, msg, arrival, receiver_count):
        raw_sequence = msg.get("sequence") if isinstance(msg, dict) else None
        try:
            sequence = None if raw_sequence is None else int(raw_sequence) & 0xFFFFFFFF
        except (TypeError, ValueError):
            sequence = None

        raw_timestamp = msg.get("timestamp_ms") if isinstance(msg, dict) else None
        try:
            timestamp_ms = None if raw_timestamp is None else int(raw_timestamp)
        except (TypeError, ValueError):
            timestamp_ms = None

        count = int(receiver_count)
        arrival = float(arrival)

        seq_delta = sequence_delta_u32(sequence, self.previous_sequence)
        count_delta = None if self.previous_receiver_count is None else max(0, count - self.previous_receiver_count)
        local_dt = None if self.previous_arrival is None else max(0.0, arrival - self.previous_arrival)
        source_dt = None
        if timestamp_ms is not None and self.previous_timestamp_ms is not None:
            source_dt = max(0.0, (timestamp_ms - self.previous_timestamp_ms) / 1000.0)

        render_skipped = 0 if count_delta is None else max(0, count_delta - 1)
        # If POC sequence advanced farther than the number of datagrams received,
        # those extra sequence positions were not observed by this receiver.
        source_missing = 0
        if seq_delta is not None and count_delta is not None:
            source_missing = max(0, seq_delta - count_delta)

        self.render_skipped_total += render_skipped
        self.source_missing_total += source_missing

        self.previous_sequence = sequence
        self.previous_receiver_count = count
        self.previous_arrival = arrival
        self.previous_timestamp_ms = timestamp_ms

        return PacketDiagnosticsSnapshot(
            sequence=sequence,
            sequence_delta=seq_delta,
            receiver_count_delta=count_delta,
            local_dt_s=local_dt,
            source_dt_s=source_dt,
            render_skipped_packets=render_skipped,
            source_missing_packets=source_missing,
            render_skipped_total=self.render_skipped_total,
            source_missing_total=self.source_missing_total,
        )
