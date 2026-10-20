# test_touch_controls.py ver 6
"""Headless tests for the separated Panda3D screen-space UI."""
import importlib
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class FakeWidget:
    created = []

    def __init__(self, **kwargs):
        self.opts = dict(kwargs)
        self.visible = True
        self.destroyed = False
        self.display_text = kwargs.get("text", "")
        FakeWidget.created.append(self)

    def __getitem__(self, key):
        return self.opts[key]

    def __setitem__(self, key, value):
        self.opts[key] = value

    def show(self):
        self.visible = True

    def hide(self):
        self.visible = False

    def destroy(self):
        self.destroyed = True

    def setText(self, text):
        self.display_text = text


@pytest.fixture
def modules(monkeypatch):
    for name in ("direct", "direct.showbase", "direct.gui", "panda3d"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    showbase = types.ModuleType("direct.showbase.ShowBase")
    showbase.ShowBase = type("ShowBase", (), {"userExit": lambda self: None})
    monkeypatch.setitem(sys.modules, showbase.__name__, showbase)
    onscreen = types.ModuleType("direct.gui.OnscreenText")
    onscreen.OnscreenText = FakeWidget
    monkeypatch.setitem(sys.modules, onscreen.__name__, onscreen)
    gui = types.ModuleType("direct.gui.DirectGui")
    gui.DirectFrame = FakeWidget
    gui.DirectButton = FakeWidget
    monkeypatch.setitem(sys.modules, gui.__name__, gui)
    core = types.ModuleType("panda3d.core")
    for name in ("AmbientLight", "DirectionalLight", "Geom", "GeomNode", "GeomTriangles",
                 "GeomVertexData", "GeomVertexFormat", "GeomVertexWriter", "LineSegs",
                 "Point3", "Filename", "Vec3", "OrthographicLens", "WindowProperties"):
        setattr(core, name, type(name, (), {}))
    core.TextNode = types.SimpleNamespace(ACenter=0, ALeft=1)
    core.loadPrcFileData = lambda *args: None
    monkeypatch.setitem(sys.modules, core.__name__, core)
    sys.modules.pop("visualizer_ui", None)
    sys.modules.pop("platform_visualizer", None)
    FakeWidget.created.clear()
    return importlib.import_module("platform_visualizer"), importlib.import_module("visualizer_ui")


def make_ui(modules, mode="DEMO", input_mode="twin"):
    _, ui_mod = modules
    anchors = types.SimpleNamespace(aspect=object(), top=object(), bottom=object())
    called = []
    ui = ui_mod.VisualizerUI(
        aspect2d=anchors.aspect,
        top_right=anchors.top,
        bottom_right=anchors.bottom,
        on_mode=lambda m: called.append(("mode", m)),
        on_input=lambda m: called.append(("input", m)),
        on_toggle_text=lambda: called.append(("text",)),
        on_quit=lambda: called.append(("quit",)),
        mode=mode,
        input_mode=input_mode,
        diagnostics_visible=True,
    )
    return ui, anchors, called


def test_screen_space_anchors_callbacks_and_exact_v41_dimensions(modules):
    ui, anchors, called = make_ui(modules)
    assert len(ui.touch_buttons) == 6
    for name in ("DEMO", "FESTO", "LENGTH", "TEXT", "POSE_DIFF"):
        assert ui.touch_buttons[name]["parent"] is anchors.top
    assert ui.touch_buttons["QUIT"]["parent"] is anchors.bottom
    assert ui.touch_buttons["FESTO"]["extraArgs"] == ["FESTO"]
    assert ui.touch_buttons["LENGTH"].visible
    assert ui.touch_buttons["DEMO"]["frameColor"] != ui.touch_buttons["FESTO"]["frameColor"]
    for button in ui.touch_buttons.values():
        assert button["relief"] == "flat"
        assert button["frameSize"] == (-0.140, 0.205, -0.052, 0.052)
        assert button["text_scale"] == 0.056
        assert button["text_pos"] == (0.020, -0.018)
        assert button["pos"][0] == -0.225
    ui.touch_buttons["FESTO"]["command"]("FESTO")
    ui.touch_buttons["TEXT"]["command"]()
    ui.touch_buttons["QUIT"]["command"]()
    assert called == [("mode", "FESTO"), ("text",), ("quit",)]


def test_easyip_lengths_hidden_highlight_and_diagnostics_independent(modules):
    ui, _, _ = make_ui(modules, mode="FESTO", input_mode="easyip")
    assert not ui.touch_buttons["LENGTH"].visible
    assert not ui.touch_buttons["POSE_DIFF"].visible
    assert ui.touch_buttons["FESTO"].visible
    assert ui.touch_buttons["FESTO"]["frameColor"] != ui.touch_buttons["DEMO"]["frameColor"]
    ui.refresh_touch_controls("DEMO", "twin", False)
    assert all(btn.visible for btn in ui.touch_buttons.values())
    assert ui.touch_buttons["TEXT"]["frameColor"] != ui.touch_buttons["QUIT"]["frameColor"]
    ui.set_status("TEST")
    assert ui.status.display_text == "TEST"
    ui.show_help(True)
    assert ui.help.visible
    ui.show_help(False)
    assert not ui.help.visible


def test_linux_lists_all_active_ipv4_interfaces_read_only(modules, monkeypatch):
    _, ui_mod = modules
    monkeypatch.setattr(ui_mod.sys, "platform", "linux")
    calls = []
    def fake_run(cmd, **kwargs):
        calls.append((cmd, kwargs))
        return types.SimpleNamespace(returncode=0, stdout=(
            "1: lo inet 127.0.0.1/8 scope host lo\n"
            "2: eth0 inet 192.168.0.10/24 brd 192.168.0.255 scope global eth0\n"
            "3: eth1 inet 192.168.1.125/24 brd 192.168.1.255 scope global eth1\n"
        ))
    monkeypatch.setattr(ui_mod.subprocess, "run", fake_run)
    assert ui_mod.current_listening_ipv4() == "eth0: 192.168.0.10\neth1: 192.168.1.125"
    assert calls[0][0] == ["ip", "-4", "-o", "addr", "show", "up"]
    ui, _, called = make_ui(modules)
    ui.show_input_chooser()
    chooser = ui._input_chooser
    assert chooser["parent"] is not None
    assert any("eth1: 192.168.1.125" in w.opts.get("text", "") for w in FakeWidget.created)
    assert len([w for w in FakeWidget.created if w.opts.get("extraArgs") in (["easyip"], ["twin"])]) == 2
    button = next(w for w in FakeWidget.created if w.opts.get("extraArgs") == ["twin"])
    button["command"]("twin")
    assert called == [("input", "twin")]
    ui.show_input_error("easyip", OSError("permission denied"))
    assert "permission denied" in chooser["text"]
    ui.hide_input_chooser()
    assert chooser.destroyed
    assert ui._input_chooser is None


def test_windows_hostname_ipv4_standard_library_no_interface_dependency(modules, monkeypatch):
    _, ui_mod = modules
    monkeypatch.setattr(ui_mod.sys, "platform", "win32")
    monkeypatch.setattr(ui_mod.socket, "gethostname", lambda: "VIS-DEV")
    calls = []
    def fake_addresses(hostname, port, family):
        calls.append((hostname, port, family))
        return [
            (None, None, None, "", ("192.168.1.27", 0)),
            (None, None, None, "", ("127.0.0.1", 0)),
            (None, None, None, "", ("192.168.0.4", 0)),
            (None, None, None, "", ("192.168.1.27", 0)),
        ]
    monkeypatch.setattr(ui_mod.socket, "getaddrinfo", fake_addresses)
    assert ui_mod.current_listening_ipv4() == "192.168.0.4\n192.168.1.27"
    assert calls == [("VIS-DEV", None, ui_mod.socket.AF_INET)]


def test_missing_or_unresolvable_interfaces_are_nonfatal(modules, monkeypatch):
    _, ui_mod = modules
    monkeypatch.setattr(ui_mod.sys, "platform", "linux")
    monkeypatch.setattr(ui_mod.subprocess, "run", lambda *a, **kw: (_ for _ in ()).throw(FileNotFoundError()))
    assert ui_mod.current_listening_ipv4() == "IPv4 addresses unavailable"
    monkeypatch.setattr(ui_mod.subprocess, "run", lambda *a, **kw: types.SimpleNamespace(returncode=0, stdout=""))
    assert ui_mod.current_listening_ipv4() == "No active IPv4 addresses"
    monkeypatch.setattr(ui_mod.sys, "platform", "win32")
    monkeypatch.setattr(ui_mod.socket, "getaddrinfo", lambda *a, **kw: (_ for _ in ()).throw(OSError("no host")))
    assert ui_mod.current_listening_ipv4() == "IPv4 addresses unavailable"


def test_application_ui_is_only_presentation_layer(modules):
    controller, _ = modules
    app = object.__new__(controller.PlatformVisualizer)
    app.aspect2d = object()
    app.a2dTopRight = object()
    app.a2dBottomRight = object()
    app.mode = "DEMO"
    app.input_mode = "twin"
    app._last_status_time = 1.0
    app.pose_diff_enabled = False
    app._build_ui()
    assert app.ui.touch_buttons["TEXT"]["parent"] is app.a2dTopRight
    app._toggle_diagnostics()
    assert not app.diagnostics_visible
    app._toggle_help()
    assert app.ui.help.visible


def test_scene_swipes_match_arrow_directions_with_one_step(modules):
    ui, _, _ = make_ui(modules)
    # Panda normalized mouse coords; 800x480 pixel displacement.
    for ending, direction in (
        ((0.2, 0.0), "right"), ((-0.2, 0.0), "left"),
        ((0.0, 0.2), "up"), ((0.0, -0.2), "down"),
    ):
        ui.begin_scene_swipe((0.0, 0.0), 800 / 480)
        assert ui.end_scene_swipe(ending, (800, 480)) == direction
        assert ui.end_scene_swipe(ending, (800, 480)) is None


def test_scene_swipes_ignore_taps_diagonals_buttons_and_chooser(modules):
    ui, _, _ = make_ui(modules)
    ratio = 800 / 480
    ui.begin_scene_swipe((0.0, 0.0), ratio)
    assert ui.end_scene_swipe((0.05, 0.04), (800, 480)) is None
    ui.begin_scene_swipe((0.0, 0.0), ratio)
    assert ui.end_scene_swipe((0.2, 0.35), (800, 480)) is None
    # Button at upper right starts around mouse x=0.78 on an 800x480 view.
    ui.begin_scene_swipe((0.85, 0.2), ratio)
    assert ui.end_scene_swipe((0.2, 0.2), (800, 480)) is None
    # Window-aspect-aware exclusion also works on square windows.
    ui.begin_scene_swipe((0.7, 0.2), 1.0)
    assert ui.end_scene_swipe((-0.1, 0.2), (480, 480)) is None
    ui._input_chooser = object()
    ui.begin_scene_swipe((0.0, 0.0), ratio)
    assert ui.end_scene_swipe((0.2, 0.0), (800, 480)) is None
    # Missing pointer location when released is harmless.
    ui._input_chooser = None
    ui.begin_scene_swipe((0.0, 0.0), ratio)
    assert ui.end_scene_swipe(None, (800, 480)) is None


def test_swipe_controller_calls_same_camera_methods_as_arrow_keys(modules):
    controller, _ = modules
    calls = []
    app = types.SimpleNamespace(
        camera_azimuth_step=5.0, camera_elevation_step=5.0,
        _orbit=lambda delta: calls.append(("orbit", delta)),
        _camera_vertical=lambda delta: calls.append(("height", delta)),
    )
    class FakeUI:
        direction = None
        def end_scene_swipe(self, xy, wh):
            return self.direction
    ui = FakeUI()
    app.ui = ui
    app._pointer_xy = lambda: (0.0, 0.0)
    app.win = types.SimpleNamespace(getXSize=lambda: 800, getYSize=lambda: 480)
    for direction in ("right", "left", "up", "down", None):
        ui.direction = direction
        controller.PlatformVisualizer._swipe_end(app)
    assert calls == [
        ("orbit", 5.0), ("orbit", -5.0),
        ("height", 5.0), ("height", -5.0),
    ]


def test_swipe_press_event_forwards_position_and_window_aspect(modules):
    controller, _ = modules
    calls = []
    app = types.SimpleNamespace(
        ui=types.SimpleNamespace(begin_scene_swipe=lambda *args: calls.append(args)),
        win=types.SimpleNamespace(getXSize=lambda: 800, getYSize=lambda: 480),
        _pointer_xy=lambda: (0.5, -0.25),
    )
    controller.PlatformVisualizer._swipe_start(app)
    assert calls == [((0.5, -0.25), 800 / 480)]


def test_requested_v44_rendering_constants(modules):
    controller, _ = modules
    assert controller.UPRIGHT_COLOUR == controller.FRAME_COLOUR
    assert controller.FIXED_JOINT_COLOUR == controller.FRAME_COLOUR
