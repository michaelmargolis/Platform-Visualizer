# test_bundled_poc_v48.py ver 2
"""Standalone POC bundle, explicit external overrides and source comparisons."""
import os
from pathlib import Path
import shutil
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from poc_platform import BUNDLED_POC_FILES, PocPlatform, report_poc_bundle_differences, are_files_identical


def _resolve_module(monkeypatch):
    # The executable imports Panda3D; reuse the headless testing stubs.
    from test_easyip_visualizer_integration import import_visualizer_with_panda_stubs
    return import_visualizer_with_panda_stubs(monkeypatch)


def test_bundled_root_is_default_even_when_cwd_contains_poc(monkeypatch, tmp_path):
    mod = _resolve_module(monkeypatch)
    for name in ('poc_config.json', 'config/platforms.py', 'kinematics/kinematics_V3.py'):
        dest = tmp_path / name
        dest.parent.mkdir(exist_ok=True)
        dest.write_bytes((ROOT / 'bundled_poc' / name).read_bytes())
    monkeypatch.chdir(tmp_path)
    assert mod.resolve_poc_root(None, None, ROOT) == (ROOT / 'bundled_poc').resolve()


def test_explicit_root_precedence_and_invalid_root_does_not_silently_fallback(monkeypatch, tmp_path):
    mod = _resolve_module(monkeypatch)
    configured = ROOT / 'bundled_poc'
    external = tmp_path / 'external'
    for name in ('poc_config.json', 'config/platforms.py', 'kinematics/kinematics_V3.py'):
        dst = external / name
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(configured / name, dst)
    assert mod.resolve_poc_root(str(external), 'bundled_poc', ROOT) == external.resolve()
    assert mod.resolve_poc_root(None, 'bundled_poc', ROOT) == configured.resolve()
    assert mod.resolve_poc_root(None, str(external), ROOT) == external.resolve()
    with pytest.raises(SystemExit, match='Invalid POC root'):
        mod.resolve_poc_root(str(tmp_path / 'does-not-exist'), 'bundled_poc', ROOT)
    with pytest.raises(SystemExit, match='Invalid app_config.json poc_root'):
        mod.resolve_poc_root(None, 'absent', ROOT)


def test_bundled_both_supported_platforms_load():
    # Both profiles use the same kinematics module and distinct config classes.
    for name in ('platform_v2', 'chair_v2'):
        model = PocPlatform(ROOT / 'bundled_poc', selected_platform=name)
        assert model.selected_platform == name
        assert model.base_points.shape == (6, 3)
        assert model.platform_local_points.shape == (6, 3)


def test_chunk_comparison_and_only_content_differences_are_reported(tmp_path):
    bundle = ROOT / 'bundled_poc'
    ext = tmp_path / 'external'
    for name in BUNDLED_POC_FILES:
        dst = ext / name
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(bundle / name, dst)
    output = []
    assert report_poc_bundle_differences(ext, bundle, printer=output.append) == []
    assert output == []

    # Changing timestamps alone must not report a difference.
    reg = ext / 'config/platforms.py'
    os.utime(reg, (reg.stat().st_mtime + 3600, reg.stat().st_mtime + 3600))
    output = []
    assert report_poc_bundle_differences(ext, bundle, printer=output.append) == []
    assert output == []

    # Same size and timestamp but different contents must report a difference.
    cfg = ext / 'kinematics/cfg_SuspendedChair.py'
    oldmtime = cfg.stat().st_mtime
    data = cfg.read_bytes()
    cfg.write_bytes(data[:-1] + (b'!' if data[-1:] != b'!' else b'?'))
    os.utime(cfg, (oldmtime, oldmtime))
    assert not are_files_identical(cfg, bundle / 'kinematics/cfg_SuspendedChair.py', chunk_size=8)
    output = []
    report = report_poc_bundle_differences(ext, bundle, printer=output.append)
    assert len(report) == 1
    assert len(output) == 2
    assert 'cfg_SuspendedChair.py' in report[0]
    assert 'Bundled:' in report[0] and 'External:' in report[0]
    assert 'platforms.py' not in '\n'.join(output)
    assert are_files_identical(cfg, cfg)

    # Missing counterpart reports missing, without raising an exception.
    cfg.unlink()
    output = []
    report = report_poc_bundle_differences(ext, bundle, printer=output.append)
    assert len(report) == 1 and 'MISSING in external root' in report[0]
