# test_parked_camera_v45.py ver 4
"""Regression checks for v45 parked transition and camera reset."""
from types import SimpleNamespace
import numpy as np

from test_easyip_visualizer_integration import import_visualizer_with_panda_stubs


def test_demo_to_twin_length_displays_cached_park_and_waits(monkeypatch):
    mod = import_visualizer_with_panda_stubs(monkeypatch)
    scene_updates = []
    class Receiver:
        def snapshot(self):
            return None, None, 7, None
    target_pose = np.array([12., 0., -40., 0., 0., 0.])
    obj = SimpleNamespace(
        receiver=Receiver(), input_mode='twin', mode='DEMO',
        parked_pose=target_pose, parked_solver_rms=2.3,
        pose=np.zeros(6), solver_rms=None, udp_state='IGNORED',
        _update_dynamic_scene=lambda: scene_updates.append(True),
        _update_pose_diff_scene=lambda: None,
        _refresh_touch_controls=lambda: None,
        last_processed_packet_count=5,
    )
    mod.PlatformVisualizer._set_mode(obj, 'LENGTH')
    assert obj.mode == 'LENGTH'
    assert obj._waiting_for_live_packet
    assert obj._mode_entry_packet_count == 7
    assert obj.udp_state == 'WAITING'
    assert np.array_equal(obj.pose, target_pose)
    assert obj.pose is not target_pose
    assert obj.solver_rms == 2.3
    assert scene_updates == [True]
    assert obj.last_processed_packet_count is None


def test_reset_uses_display_specific_camera_default(monkeypatch):
    mod = import_visualizer_with_panda_stubs(monkeypatch)
    applied = []
    obj = SimpleNamespace(
        camera_default_azimuth=-33., camera_default_radius=3805.,
        camera_default_elevation=18., camera_azimuth=42., camera_radius=2800.,
        camera_elevation=75., camera_view_z_offset=50.,
        ortho_default_height=2000., ortho_height=1000., projection="orthographic",
        cam=SimpleNamespace(node=lambda: SimpleNamespace(setLens=lambda lens: None)),
        perspective_lens=object(),
        _apply_camera=lambda: applied.append(True),
    )
    mod.PlatformVisualizer._reset_camera(obj)
    assert obj.camera_radius == 3805.
    assert obj.camera_azimuth == -33.
    assert obj.camera_elevation == 18.
    assert obj.projection == "perspective"
    assert applied == [True]
