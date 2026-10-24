# poc_platform.py ver 9
"""Load the currently selected POC platform geometry/kinematics."""
from __future__ import annotations

import importlib
import importlib.util
import inspect
import json
from pathlib import Path
import sys

import numpy as np



# The standalone distribution contains these unmodified POC files. Compare
# against an explicitly selected external POC tree, without changing either.
BUNDLED_POC_FILES = (
    "poc_config.json",
    "config/platforms.py",
    "kinematics/kinematics_V3.py",
    "kinematics/cfg_SuspendedPlatformMdx.py",
    "kinematics/cfg_SuspendedChair.py",
    "kinematics/cfg_SuspendedChairFalcon.py",
)


def are_files_identical(file1_path, file2_path, chunk_size=8192) -> bool:
    """Compare file contents in chunks, without relying on timestamps."""
    file1_path, file2_path = Path(file1_path), Path(file2_path)
    if file1_path.resolve() == file2_path.resolve():
        return True
    if file1_path.stat().st_size != file2_path.stat().st_size:
        return False
    with file1_path.open("rb") as first, file2_path.open("rb") as second:
        while True:
            first_chunk = first.read(chunk_size)
            second_chunk = second.read(chunk_size)
            if first_chunk != second_chunk:
                return False
            if not first_chunk:
                return True


def report_poc_bundle_differences(external_root, bundled_root, printer=print):
    """Report only differing/missing bundled POC files; never modify them."""
    from datetime import datetime

    external_root = Path(external_root).resolve()
    bundled_root = Path(bundled_root).resolve()
    if external_root == bundled_root:
        return []

    def modified(path):
        return datetime.fromtimestamp(path.stat().st_mtime).astimezone().strftime("%Y-%m-%d %H:%M:%S %z")

    reports = []
    for relative in BUNDLED_POC_FILES:
        bundled = bundled_root / relative
        external = external_root / relative
        if not bundled.is_file() or not external.is_file():
            message = f"  {relative}: MISSING in {'bundle' if not bundled.is_file() else 'external root'}"
        elif are_files_identical(bundled, external):
            continue
        else:
            message = (f"  {relative}\n"
                       f"    Bundled:  {modified(bundled)}\n"
                       f"    External: {modified(external)}")
        reports.append(message)
    if reports:
        printer("POC file differences:")
        for message in reports:
            printer(message)
    return reports


class PocPlatform:
    """Adapter exposing the POC-selected platform to the visualizer."""

    def __init__(self, poc_root, selected_platform=None):
        self.poc_root = Path(poc_root).expanduser().resolve()
        self._validate_root()

        if selected_platform is None:
            # Backwards compatibility for app_config.json without selected_platform.
            poc_config = json.loads((self.poc_root / "poc_config.json").read_text(encoding="utf-8"))
            selected_platform = poc_config.get("selected_platform", "platform_v2")
        self.selected_platform = str(selected_platform)
        platforms = self._load_platform_registry()
        if self.selected_platform not in platforms:
            raise ValueError(
                f"Selected platform {self.selected_platform!r} is not present in config/platforms.py"
            )

        metadata = dict(platforms[self.selected_platform])
        self.platform_metadata = metadata
        self.config_module = str(metadata["sop_config_module"])

        root_text = str(self.poc_root)
        sys.path.insert(0, root_text)
        try:
            cfg_module = importlib.import_module(self.config_module)
            kin_module = importlib.import_module("kinematics.kinematics_V3")
        finally:
            if sys.path and sys.path[0] == root_text:
                sys.path.pop(0)

        self.cfg = cfg_module.PlatformConfig()
        self._kin_module = kin_module

        platform_coords = getattr(self.cfg, "platform_coords", None)
        if platform_coords is None:
            platform_coords = getattr(self.cfg, "platform_coords_xy", None)
        if platform_coords is None:
            raise AttributeError(
                f"{self.config_module}.PlatformConfig has neither platform_coords nor platform_coords_xy"
            )

        self._configure_geometry(self.cfg.base_coords, platform_coords)

        ids = {
            "",
            self.selected_platform,
            str(metadata.get("name", "")),
            str(metadata.get("platform_name", "")),
            str(getattr(self.cfg, "PLATFORM_NAME", "")),
        }
        # Historical POC_TWIN aliases retained for compatibility with captures/tools.
        if self.selected_platform == "platform_v2":
            ids.update(("wheelchair", "wheelchair_platform", "MDX Suspended Platform"))
        self.accepted_platform_ids = frozenset(v for v in ids if v is not None)

        print("POC platform:")
        print(f"  selected key   : {self.selected_platform}")
        print(f"  display name   : {metadata.get('name', '?')}")
        print(f"  config module  : {self.config_module}")
        if self.extension_reserve is not None:
            print(f"  extension reserve: {self.extension_reserve:.1f} mm")
        print(f"  neutral Z      : {self.mid_z:.2f} mm")

    def _configure_geometry(self, base_coords, platform_coords):
        """Configure the selected POC kinematics from explicit attachment geometry."""
        PlatformParams = self._kin_module.PlatformParams
        params = PlatformParams(
            self.cfg.MUSCLE_MIN_LENGTH,
            self.cfg.MUSCLE_MAX_LENGTH,
            self.cfg.FIXED_HARDWARE_LENGTH,
            self.cfg.LIMITS_1DOF_TRANFORM,
        )
        self.kin = self._kin_module.Kinematics()
        # Respect an explicit platform-specific axis convention. Without it,
        # retain the historic POC default used by the chair configurations.
        axis_flip_mask = getattr(self.cfg, "AXIS_FLIP_MASK", None)
        if axis_flip_mask is not None:
            self.kin.set_axis_flip_mask(axis_flip_mask)
        clearance_offset = float(getattr(self.cfg, "PLATFORM_CLEARANCE_OFFSET", 0.0))
        extension_reserve = getattr(self.cfg, "MUSCLE_EXTENSION_RESERVE", None)
        self.extension_reserve = None if extension_reserve is None else float(extension_reserve)
        set_geometry_params = inspect.signature(self.kin.set_geometry).parameters
        if extension_reserve is not None:
            if "muscle_extension_reserve" not in set_geometry_params:
                raise RuntimeError(
                    "The selected POC platform uses MUSCLE_EXTENSION_RESERVE, but its "
                    "kinematics_V3.set_geometry() does not support muscle_extension_reserve. "
                    "Use POC v146 or later."
                )
            self.kin.set_geometry(
                base_coords,
                platform_coords,
                params,
                clearance_offset=clearance_offset,
                muscle_extension_reserve=self.extension_reserve,
            )
        else:
            self.kin.set_geometry(
                base_coords,
                platform_coords,
                params,
                clearance_offset=clearance_offset,
            )

        self.base_points = np.asarray(self.kin.base_coords, dtype=float)
        self.platform_local_points = np.asarray(self.kin.platform_local_coords, dtype=float)
        self.bounds = np.asarray(self.cfg.LIMITS_1DOF_TRANFORM, dtype=float)
        self.mid_z = float(self.kin.PLATFORM_MID_HEIGHT)

    def apply_visual_geometry(self, fixed_attachments, moving_attachments):
        """Use CAD-derived visual attachments as the emulator solver geometry.

        This is intentionally opt-in per visualizer platform profile.  It is useful
        while a CAD-corrected platform is being validated before the same geometry
        is promoted into the POC platform configuration.
        """
        fixed = np.asarray(fixed_attachments, dtype=float)
        moving = np.asarray(moving_attachments, dtype=float)
        if fixed.shape != (6, 3) or moving.shape != (6, 3):
            raise ValueError("Visual geometry override requires six fixed and six moving XYZ points")
        self._configure_geometry(fixed.tolist(), moving[:, :2].tolist())
        print("  solver geometry: CAD visual override")
        print(f"  neutral Z      : {self.mid_z:.2f} mm")

    def _validate_root(self):
        required = (
            Path("poc_config.json"),
            Path("config/platforms.py"),
            Path("kinematics/kinematics_V3.py"),
        )
        missing = [str(p) for p in required if not (self.poc_root / p).is_file()]
        if missing:
            raise FileNotFoundError(
                f"POC root is missing {', '.join(missing)}: {self.poc_root}"
            )

    def _load_platform_registry(self):
        path = self.poc_root / "config" / "platforms.py"
        spec = importlib.util.spec_from_file_location("visualizer_poc_platforms", path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Could not load POC platform registry: {path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        platforms = getattr(module, "PLATFORMS", None)
        if not isinstance(platforms, dict):
            raise ValueError(f"POC platform registry has no PLATFORMS dictionary: {path}")
        return platforms

    def accepts_platform_id(self, platform_id):
        return str(platform_id) in self.accepted_platform_ids

    def moving_points(self, pose):
        return np.asarray(self.kin.inverse_kinematics(pose), dtype=float)

    def platform_pose_basis(self, pose):
        """Return world origin/forward/up matching the current POC IK convention."""
        pose = np.asarray(pose, dtype=float)
        translation = pose[:3] * self.kin.FLIP_TRANSLATION
        rpy = pose[3:6] * self.kin.FLIP_ROTATION
        rotation = self.kin.calc_rotation(rpy)
        local_origin = np.array([0.0, 0.0, self.mid_z], dtype=float)
        origin = rotation @ local_origin + translation
        forward = rotation @ np.array([0.0, 1.0, 0.0], dtype=float)
        up = rotation @ np.array([0.0, 0.0, 1.0], dtype=float)
        return origin, forward, up

    def transform_xy_paths(self, paths, pose):
        """Transform visual paths drawn in the nominal moving reference plane."""
        pose = np.asarray(pose, dtype=float)
        translation = pose[:3] * self.kin.FLIP_TRANSLATION
        rpy = pose[3:6] * self.kin.FLIP_ROTATION
        rotation = self.kin.calc_rotation(rpy)
        out = []
        for path in paths:
            local = np.array([[x, y, self.mid_z] for x, y in path], dtype=float)
            out.append((rotation @ local.T).T + translation)
        return out


# Compatibility alias for older external scripts that imported the MDX-specific name.
PocPlatformMdx = PocPlatform
