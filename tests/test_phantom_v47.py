# test_phantom_v47.py ver 2
"""The pose-difference overlay requires an explicitly unparked, finite real POC pose."""
import numpy as np
from test_easyip_visualizer_integration import import_visualizer_with_panda_stubs


def test_only_confirmed_unparked_transform_is_drawable(monkeypatch):
    mod = import_visualizer_with_panda_stubs(monkeypatch)
    packet = {"parking_state": "UNPARKED", "applied_transform_real": [1, 2, 3, .1, -.2, .3]}
    assert np.allclose(mod.pose_diff_pose_from_packet(packet), [1, 2, 3, .1, -.2, .3])
    for state in ("PARKED", "PARKING", "UNPARKING", "UNKNOWN", None):
        packet["parking_state"] = state
        assert mod.pose_diff_pose_from_packet(packet) is None
    packet["parking_state"] = "UNPARKED"
    packet["applied_transform_real"] = [0, 0, 0]
    assert mod.pose_diff_pose_from_packet(packet) is None
    packet["applied_transform_real"] = [0, 0, 0, 0, 0, np.nan]
    assert mod.pose_diff_pose_from_packet(packet) is None
