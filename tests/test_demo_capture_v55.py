# test_demo_capture_v55.py ver 2
"""12-second demo and fixed-simulation-time GIF capture tests."""
from pathlib import Path
import math
import sys

import numpy as np
import pytest
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from demo_motion import CannedMotion
from poc_platform import PocPlatform
from tools.capture_demo_gif import FPS, frame_times, frame_durations_ms, demo_heading, default_output_path


@pytest.mark.parametrize("platform_name", ["platform_v2", "chair_v2", "Falcon"])
def test_demo_uses_actual_platform_config_limits(platform_name):
    platform = PocPlatform(ROOT / "bundled_poc", selected_platform=platform_name)
    demo = CannedMotion(platform.bounds)
    assert demo.period == 12.0
    assert len(demo.stages) == 6
    for stage_i, (name, idx, amplitude, duration) in enumerate(demo.stages):
        assert duration == 2.0
        assert amplitude == pytest.approx(platform.cfg.LIMITS_1DOF_TRANFORM[idx])
        peak, active_name = demo.pose(2.0 * stage_i + 0.5)
        assert active_name == name
        assert peak[idx] == pytest.approx(amplitude)
        assert np.count_nonzero(peak) == 1
        negative_peak, _ = demo.pose(2.0 * stage_i + 1.5)
        assert negative_peak[idx] == pytest.approx(-amplitude)
    assert np.allclose(demo.pose(12.0)[0], demo.pose(0.0)[0])


def test_invalid_limits_cannot_create_demo():
    for limits in ([1, 2, 3], [1, 2, 3, 4, 5, -1], [1, 2, 3, math.inf, 5, 6]):
        with pytest.raises(ValueError, match="six finite positive"):
            CannedMotion(limits)


@pytest.mark.parametrize("fps", [4, 6, 8, 10, 12, 15, 20])
def test_gif_timing_exact_for_common_fps(fps):
    samples = frame_times(12.0, fps)
    durations = frame_durations_ms(len(samples), fps)
    assert len(samples) == 12 * fps
    assert samples[0] == 0
    assert samples[-1] == pytest.approx(12 - 1 / fps)
    assert all(d > 0 and d % 10 == 0 for d in durations)
    assert sum(durations) == 12000


def test_fps_setting_and_pillow_gif_framing(tmp_path):
    assert FPS == 12
    frames = [Image.new("RGB", (20, 10), (i * 50, 0, 0)) for i in range(3)]
    palette = frames[0].quantize(colors=128)
    colors = [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in frames]
    gif = tmp_path / "demo.gif"
    colors[0].save(gif, format="GIF", save_all=True, append_images=colors[1:],
                   duration=frame_durations_ms(3, 3), loop=0, optimize=True, disposal=1)
    with Image.open(gif) as img:
        assert img.format == "GIF"
        assert img.info["loop"] == 0
        assert img.n_frames >= 1


@pytest.mark.parametrize("custom_output", [True, False])
def test_capture_writes_full_gif_without_network_or_desktop_capture(tmp_path, monkeypatch, custom_output):
    """Exercise capture plumbing with Panda3D-shaped fakes (renderer unavailable in CI)."""
    import json
    import types
    from tools import capture_demo_gif

    config = tmp_path / "app_config.json"
    config.write_text(json.dumps({"selected_platform": "chair_v2", "input_mode": "twin"}))
    screenshots = []
    headings = []
    instances = []

    class FakeFilename:
        @staticmethod
        def fromOsSpecific(value):
            return value

    class FakeWindowProperties:
        def setSize(self, width, height):
            self.size = (width, height)

    class FakeWindow:
        def requestProperties(self, props):
            self.size = props.size
        def getXSize(self):
            return self.size[0]
        def getYSize(self):
            return self.size[1]
        def saveScreenshot(self, path):
            # Test the actual PNG/Pillow/GIF stages with a changing fake frame.
            screenshot = Image.new("RGB", self.size, "blue")
            from PIL import ImageDraw
            painter = ImageDraw.Draw(screenshot)
            colors = ["red", "green", "yellow"]
            for j, color in enumerate(colors):
                x = (j + len(screenshots) % 3) % 3
                painter.rectangle((x * 15, 0, x * 15 + 14, 9), fill=color)
            screenshot.save(path)
            screenshots.append(path)
            assert instances[0].ui.status == demo_heading(instances[0].demo_stage)
            headings.append(instances[0].ui.status)
            return True

    class FakeNode:
        def hide(self):
            self.hidden = True

    class FakeUi:
        def __init__(self):
            self.status = ""
            self.help_hidden = False
        def show_help(self, visible):
            self.help_hidden = not visible
        def set_status(self, text):
            self.status = text

    class FakeTasks:
        def remove(self, name):
            assert name == "platform-update"
        def step(self):
            pass

    class FakeVisualizer:
        def __init__(self, root, config_path, **kwargs):
            assert kwargs["demo_only"] is True
            assert kwargs["selected_platform"] == "chair_v2"
            self.win = FakeWindow()
            self.aspect2d = FakeNode()
            self.labels_root = FakeNode()
            self.ui = FakeUi()
            self.taskMgr = FakeTasks()
            self.demo = CannedMotion([1, 2, 3, 4, 5, 6])
            self.platform = types.SimpleNamespace(selected_platform="chair_v2")
            instances.append(self)
        def _update_dynamic_scene(self):
            pass
        def destroy(self):
            self.destroyed = True

    core = types.ModuleType("panda3d.core")
    core.Filename = FakeFilename
    core.WindowProperties = FakeWindowProperties
    pkg = types.ModuleType("panda3d")
    pkg.core = core
    visualizer = types.ModuleType("platform_visualizer")
    visualizer.PlatformVisualizer = FakeVisualizer
    visualizer.resolve_poc_root = lambda *args: tmp_path
    with monkeypatch.context() as m:
        m.setattr(capture_demo_gif, "ROOT", tmp_path)
        m.setitem(sys.modules, "panda3d", pkg)
        m.setitem(sys.modules, "panda3d.core", core)
        m.setitem(sys.modules, "platform_visualizer", visualizer)
        capture_args = ["--config", str(config), "--width", "48", "--height", "24", "--fps", "2"]
        if custom_output:
            capture_args += ["--output", str(tmp_path / "docs" / "custom.gif")]
        capture_demo_gif.main(capture_args)
    assert len(screenshots) == 24
    assert instances[0].ui.help_hidden
    assert instances[0].labels_root.hidden
    assert not hasattr(instances[0].aspect2d, "hidden"), "Capture must retain the onscreen buttons"
    assert headings == [demo_heading(instances[0].demo.pose(i / 2)[1]) for i in range(24)]
    assert headings[0] == "PLATFORM TWIN DEMO: HEAVE"
    gif_path = tmp_path / "docs" / ("custom.gif" if custom_output else "demo-chair_v2.gif")
    with Image.open(gif_path) as gif:
        assert gif.size == (48, 24)
        assert gif.info["loop"] == 0
        assert gif.n_frames == 24
        assert sum(gif.seek(i) or gif.info["duration"] for i in range(gif.n_frames)) == 12000


def test_platform_name_output_and_heading():
    assert default_output_path("chair_v2").name == "demo-chair_v2.gif"
    assert default_output_path("platform_v2").name == "demo-platform_v2.gif"
    assert default_output_path("Falcon").name == "demo-Falcon.gif"
    assert default_output_path("chair v2").name == "demo-chair_v2.gif"
    with pytest.raises(ValueError, match="Selected platform"):
        default_output_path("...")
    assert demo_heading("SWAY") == "PLATFORM TWIN DEMO: SWAY"
