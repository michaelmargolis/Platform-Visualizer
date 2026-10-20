# test_easyip_receiver.py ver 1
"""Read-only EasyIP parser/UDP regression tests, independent of Panda3D."""
from __future__ import annotations

import importlib.util
import socket
import struct
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from easyip_receiver import EasyIpReceiver, decode_pressure_write, HEADER

# Import actual POC v149 EasyIP Factory to catch wire-format drift.
EASYIP_SOURCE = Path(__file__).resolve().parents[2] / 'poc' / 'output' / 'fstlib' / 'easyip.py'
if EASYIP_SOURCE.is_file():
    spec = importlib.util.spec_from_file_location('poc_v149_easyip', EASYIP_SOURCE)
    easyip = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(easyip)
else:
    easyip = None


def packet(words, counter=3):
    if easyip is not None:
        return easyip.Factory.send_flagword(counter, words, offset=0).pack()
    return HEADER.pack(0, 0, counter, 0, 0, 1, len(words), 0, 0, 0, 0, 0, 0) + struct.pack('<' + 'H' * len(words), *words)


def test_header_matches_poc_version():
    assert HEADER.size == 20
    if easyip is not None:
        assert HEADER.format == easyip.Packet.HEADER_FORMAT.replace(' ', '')


def test_six_pressure_words():
    msg = decode_pressure_write(packet([0, 100, 200, 300, 400, 6000], 42))
    assert msg == {'command_pressure_mbar': [0, 100, 200, 300, 400, 6000],
                   'prop_command_word': None, 'easyip_counter': 42}


@pytest.mark.parametrize('prop', [0, 1])
def test_seventh_prop_word_is_command_not_pressure(prop):
    msg = decode_pressure_write(packet([1000] * 6 + [prop]))
    assert msg['command_pressure_mbar'] == [1000] * 6
    assert msg['prop_command_word'] == prop


@pytest.mark.parametrize('data', [
    b'', b'\0' * 19,
    packet([1] * 5),
    packet([1] * 8),
    packet([6001] * 6),
    packet([100] * 6 + [2]),
    packet([100] * 6)[:-1],
    packet([100] * 6) + b'\0',
])
def test_rejects_non_matching_payloads(data):
    with pytest.raises(ValueError):
        decode_pressure_write(data)


def test_rejects_response_and_unexpected_register_offset():
    data = bytearray(packet([100] * 6))
    data[0] |= 0x80
    with pytest.raises(ValueError, match='response'):
        decode_pressure_write(bytes(data))
    data = bytearray(packet([100] * 6))
    data[10:12] = struct.pack('<H', 1)  # senddata_offset
    with pytest.raises(ValueError, match='offset'):
        decode_pressure_write(bytes(data))


def test_read_only_network_receiver_tracks_prop_and_rejects():
    receiver = EasyIpReceiver(0)
    port = receiver._sock.getsockname()[1]
    assert receiver._sock.getsockname()[0] == '0.0.0.0'
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.settimeout(0.25)
    try:
        tx.sendto(packet([100, 200, 300, 400, 500, 600, 1]), ('127.0.0.1', port))
        deadline = time.monotonic() + 1.5
        while receiver.snapshot()[2] < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        msg, arrival, count, err = receiver.snapshot()
        assert count == 1 and arrival > 0 and not err
        assert msg['command_pressure_mbar'] == [100, 200, 300, 400, 500, 600]
        assert msg['prop_command_word'] == 1
        # The visualizer listener must never send EasyIP replies.
        with pytest.raises(socket.timeout):
            tx.recvfrom(2048)
        tx.sendto(b'bad', ('127.0.0.1', port))
        deadline = time.monotonic() + 1.5
        while receiver.diagnostics()['rejected_count'] < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert receiver.diagnostics()['rejected_count'] == 1
        assert receiver.snapshot()[2] == 1
        tx.sendto(packet([0] * 6), ('127.0.0.1', port))
        deadline = time.monotonic() + 1.5
        while receiver.snapshot()[2] < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert receiver.snapshot()[0]['prop_command_word'] is None
        assert receiver.snapshot()[2] == 2
    finally:
        tx.close()
        receiver.close()
