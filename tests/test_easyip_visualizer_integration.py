# test_easyip_visualizer_integration.py ver 5
"""Headless visualizer input-mode checks; Panda3D window is not required."""
import importlib
import sys
import time
import types
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def import_visualizer_with_panda_stubs(monkeypatch):
    for name in ('direct', 'direct.showbase', 'direct.gui', 'panda3d'):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    showbase = types.ModuleType('direct.showbase.ShowBase')
    showbase.ShowBase = type('ShowBase', (), {})
    monkeypatch.setitem(sys.modules, showbase.__name__, showbase)
    onscreen = types.ModuleType('direct.gui.OnscreenText')
    onscreen.OnscreenText = type('OnscreenText', (), {})
    monkeypatch.setitem(sys.modules, onscreen.__name__, onscreen)
    gui = types.ModuleType('direct.gui.DirectGui')
    gui.DirectFrame = type('DirectFrame', (), {})
    gui.DirectButton = type('DirectButton', (), {})
    monkeypatch.setitem(sys.modules, gui.__name__, gui)
    core = types.ModuleType('panda3d.core')
    for name in ('AmbientLight', 'DirectionalLight', 'Geom', 'GeomNode', 'GeomTriangles',
                 'GeomVertexData', 'GeomVertexFormat', 'GeomVertexWriter', 'LineSegs',
                 'Point3', 'Filename', 'TextNode', 'Vec3', 'OrthographicLens', 'WindowProperties', 'loadPrcFileData'):
        setattr(core, name, type(name, (), {}))
    monkeypatch.setitem(sys.modules, core.__name__, core)
    sys.modules.pop('platform_visualizer', None)
    return importlib.import_module('platform_visualizer')


def test_easyip_l_key_does_not_enable_unavailable_lengths(monkeypatch):
    mod = import_visualizer_with_panda_stubs(monkeypatch)
    obj = SimpleNamespace(input_mode='easyip', mode='DEMO', receiver=object())
    mod.PlatformVisualizer._set_mode(obj, 'LENGTH')
    assert obj.mode == 'DEMO'


def test_easyip_pressure_packet_reaches_existing_model_and_captures_prop(monkeypatch):
    mod = import_visualizer_with_panda_stubs(monkeypatch)
    calls = []
    class FakeReceiver:
        def snapshot(self):
            return {'command_pressure_mbar':[100,200,300,400,500,600],
                    'prop_command_word':0}, time.monotonic(), 1, ''
    class FakePressure:
        def estimate(self, values, load):
            calls.append((list(values),load))
            return SimpleNamespace(contractions_mm=[20.0]*6,
                                   directions=[0]*6, saturated=[False]*6)
    class FakeDynamics:
        enabled = False
        contraction_rate_mm_s=100
        extension_rate_mm_s=100
        def step(self, targets, dt): return targets
    monkeypatch.setattr(mod, 'solve_pose_from_muscle_lengths',
                        lambda *a, **kw: (np.zeros(6), 0.1, {'iterations':1}))
    obj = SimpleNamespace(
        mode='FESTO', input_mode='easyip', receiver=FakeReceiver(),
        _update_pose_diff_scene=lambda: None,
        udp_stale_seconds=0.5, platform=SimpleNamespace(
            cfg=SimpleNamespace(MUSCLE_MAX_LENGTH=800, MUSCLE_MIN_LENGTH=600),
            kin=object(), bounds=[]),
        udp_state='WAITING', last_processed_packet_count=None,
        packet_diagnostics=mod.PacketDiagnostics(), packet_diag=None,
        prop_command_word=None, pressure_model=FakePressure(),
        pressure_load=40.0, muscle_dynamics=FakeDynamics(),
        update_hz=20, last_pressure_packet_arrival=None,
        solver_peak_length_delta_mm=0, previous_solver_lengths=None,
        pose=np.zeros(6), solver_rms=None, solver_iterations=None,
        _last_easyip_solve_time=0.0, easyip_solve_interval_s=0.1)
    mod.PlatformVisualizer._update_mode(obj, time.monotonic())
    assert obj.udp_state == 'LIVE'
    assert calls == [([100,200,300,400,500,600], 40.0)]
    assert obj.prop_command_word == 0
    assert obj.last_processed_packet_count == 1
    mod.PlatformVisualizer._update_mode(obj, time.monotonic())
    assert len(calls) == 1  # solver/pressure model runs once per new packet


def test_twin_mode_still_checks_platform_identity(monkeypatch):
    mod = import_visualizer_with_panda_stubs(monkeypatch)
    obj = SimpleNamespace(
        mode='FESTO', input_mode='twin',
        _update_pose_diff_scene=lambda: None,
        receiver=SimpleNamespace(snapshot=lambda: ({'platform':'WRONG'},time.monotonic(),1,'')),
        platform=SimpleNamespace(accepts_platform_id=lambda value: value=='chair_v2'),
        udp_stale_seconds=0.5, udp_state='WAITING')
    mod.PlatformVisualizer._update_mode(obj,time.monotonic())
    assert obj.udp_state == 'WRONG PLATFORM'
