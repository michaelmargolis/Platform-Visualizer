# tests/test_chair_profiles_v52.py ver 1
"""Check platform-specific support inheritance, heights and optional seat models."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_seat_support_heights_and_common_model():
    config = json.loads((ROOT / "app_config.json").read_text())
    common = config["chair"]
    profiles = config["platform_profiles"]
    assert profiles["platform_v2"]["chair"]["support"]["height_mm"] == 300.0
    assert profiles["chair_v2"]["chair"]["support"]["height_mm"] == 250.0
    for name in ("platform_v2", "chair_v2"):
        specific = profiles[name]["chair"]
        assert (ROOT / specific["model"]).is_file()
        assert specific["model"] == common["model"]
        effective_support = {**common["support"], **specific["support"]}
        assert effective_support["width_mm"] == common["support"]["width_mm"]
        assert effective_support["depth_mm"] == common["support"]["depth_mm"]
        assert effective_support["colour"] == common["support"]["colour"]


def test_support_zero_height_and_model_override_supported_in_source():
    source = (ROOT / "platform_visualizer.py").read_text()
    assert 'if self.chair_support_height_mm > 0.0:' in source
    assert 'self.chair_support_height_mm < 0.0' in source
    assert 'cfg.get("model", "assets/seat_lowpoly.egg")' in source
    assert 'cfg.get("z_offset_mm", 0.0)' in source
