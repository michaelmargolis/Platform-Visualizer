# test_camera_snap_v54.py ver 1
"""Static regression checks for platform-aligned camera snap presets."""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_snap_angles_and_key_bindings():
    src = (ROOT / "platform_visualizer.py").read_text()
    tree = ast.parse(src)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "PlatformVisualizer")
    snap = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_snap_camera_view")
    angles = next(n.value for n in snap.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "angles" for t in n.targets))
    assert ast.literal_eval(angles) == {"top": (-90.0, 90.0), "front": (0.0, 0.0), "side": (-90.0, 0.0)}
    for key, view in (("1", "top"), ("2", "front"), ("3", "side")):
        assert f'self.accept("{key}", self._snap_camera_view, ["{view}"])' in src
    assert "self._apply_camera()" in ast.get_source_segment(src, snap)
    assert "self.projection =" not in ast.get_source_segment(src, snap)
    assert "self.camera_radius =" not in ast.get_source_segment(src, snap)


def test_help_lists_snap_keys():
    assert "1/2/3: top/front/side views" in (ROOT / "visualizer_ui.py").read_text()
