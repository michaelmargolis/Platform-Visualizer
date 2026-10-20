# platform_visualizer.py ver 48
"""800x480 Panda3D proof-of-concept for the POC suspended platform emulator."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path
import numpy as np

from visual_geometry import load_visual_geometry
from forward_solver import solve_pose_from_muscle_lengths
from twin_udp import TwinUdpReceiver
from easyip_receiver import EasyIpReceiver
from demo_motion import CannedMotion
from poc_platform import PocPlatform, report_poc_bundle_differences
from pressure_model import EmpiricalPressureModel, MuscleDynamics
from udp_diagnostics import PacketDiagnostics

try:
    from direct.showbase.ShowBase import ShowBase
    from visualizer_ui import VisualizerUI
    from panda3d.core import (
        AmbientLight, DirectionalLight, Geom, GeomNode, GeomTriangles,
        GeomVertexData, GeomVertexFormat, GeomVertexWriter, LineSegs, Point3, OrthographicLens,
        Filename, TextNode, Vec3, WindowProperties, loadPrcFileData,
    )
except ImportError as exc:
    raise SystemExit("Panda3D is required: python -m pip install panda3d numpy") from exc

BG = (0.0, 0.0, 1.0, 1.0)
FRAME_COLOUR = (0.17, 0.17, 0.17, 1.0)
UPRIGHT_COLOUR = FRAME_COLOUR
FIXED_JOINT_COLOUR = FRAME_COLOUR
PLATFORM_COLOUR = (0.55, 0.07, 0.05, 1.0)
PLATFORM_EDGE_COLOUR = (1.00, 0.30, 0.22, 1.0)
MUSCLE_COLOUR = (0.0, 0.0, 0.0, 1.0)
MOVING_JOINT_COLOUR = (1.00, 0.45, 0.32, 1.0)
NUMBER_COLOUR = (1.00, 0.95, 0.32, 1.0)
CHAIR_COLOUR = (0.82, 0.05, 0.03, 1.0)
POSE_DIFF_COLOUR = (0.15, 0.95, 0.95, 1.0)


def pose_diff_pose_from_packet(msg):
    """Return applied POC real pose only while confirmed unparked."""
    if not isinstance(msg, dict) or msg.get("parking_state") != "UNPARKED":
        return None
    values = msg.get("applied_transform_real")
    if values is None:
        return None
    try:
        pose = np.asarray(values, dtype=float)
        if pose.shape == (6,) and np.all(np.isfinite(pose)):
            return pose.copy()
    except (TypeError, ValueError):
        pass
    return None


def line_node(name, paths, colour, thickness=3.0, closed_flags=None):
    segs = LineSegs(name)
    segs.setColor(*colour)
    segs.setThickness(thickness)
    if closed_flags is None:
        closed_flags = [False] * len(paths)
    for path, closed in zip(paths, closed_flags):
        if len(path) < 2:
            continue
        p0 = path[0]
        segs.moveTo(float(p0[0]), float(p0[1]), float(p0[2]))
        for p in path[1:]:
            segs.drawTo(float(p[0]), float(p[1]), float(p[2]))
        if closed:
            segs.drawTo(float(p0[0]), float(p0[1]), float(p0[2]))
    return segs.create()


def make_cylinder_node(name, radius, height, colour, sides=12):
    fmt = GeomVertexFormat.getV3c4()
    vdata = GeomVertexData(name, fmt, Geom.UHStatic)
    vdata.setNumRows(2 * sides + 2)
    vw, cw = GeomVertexWriter(vdata, "vertex"), GeomVertexWriter(vdata, "color")
    for z in (0.0, height):
        for i in range(sides):
            a = 2.0 * math.pi * i / sides
            vw.addData3(radius * math.cos(a), radius * math.sin(a), z)
            cw.addData4(*colour)
    bc, tc = 2 * sides, 2 * sides + 1
    vw.addData3(0, 0, 0); cw.addData4(*colour)
    vw.addData3(0, 0, height); cw.addData4(*colour)
    tris = GeomTriangles(Geom.UHStatic)
    for i in range(sides):
        j = (i + 1) % sides
        bi, bj, ti, tj = i, j, sides + i, sides + j
        tris.addVertices(bi, bj, tj); tris.addVertices(bi, tj, ti)
        tris.addVertices(bc, bj, bi); tris.addVertices(tc, ti, tj)
    tris.closePrimitive()
    geom = Geom(vdata); geom.addPrimitive(tris)
    node = GeomNode(name); node.addGeom(geom)
    return node




def make_box_node(name, width, depth, height, colour):
    """Create an axis-aligned box centred on local X/Y with its base at Z=0."""
    w = float(width) * 0.5
    d = float(depth) * 0.5
    h = float(height)
    verts = [
        (-w, -d, 0.0), (w, -d, 0.0), (w, d, 0.0), (-w, d, 0.0),
        (-w, -d, h),   (w, -d, h),   (w, d, h),   (-w, d, h),
    ]
    faces = [
        (0, 2, 1), (0, 3, 2),
        (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4),
        (1, 2, 6), (1, 6, 5),
        (2, 3, 7), (2, 7, 6),
        (3, 0, 4), (3, 4, 7),
    ]
    fmt = GeomVertexFormat.getV3c4()
    vdata = GeomVertexData(name, fmt, Geom.UHStatic)
    vdata.setNumRows(len(verts))
    vw = GeomVertexWriter(vdata, "vertex")
    cw = GeomVertexWriter(vdata, "color")
    for x, y, z in verts:
        vw.addData3(x, y, z)
        cw.addData4(*colour)
    tris = GeomTriangles(Geom.UHStatic)
    for a, b, c in faces:
        tris.addVertices(a, b, c)
    tris.closePrimitive()
    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode(name)
    node.addGeom(geom)
    return node


def make_pyramid_node(name, base_size, height, colour):
    """Square clevis stand-in: local tip Z=0, square base at Z=-height."""
    half = float(base_size) / 2.0
    z = -float(height)
    vertices = [(-half, -half, z), (half, -half, z),
                (half, half, z), (-half, half, z), (0.0, 0.0, 0.0)]
    fmt = GeomVertexFormat.getV3c4()
    vdata = GeomVertexData(name, fmt, Geom.UHStatic)
    vdata.setNumRows(5)
    vw = GeomVertexWriter(vdata, "vertex")
    cw = GeomVertexWriter(vdata, "color")
    for v in vertices:
        vw.addData3(*v)
        cw.addData4(*colour)
    tris = GeomTriangles(Geom.UHStatic)
    for i in range(4):
        tris.addVertices(i, (i + 1) % 4, 4)
    tris.addVertices(0, 2, 1)
    tris.addVertices(0, 3, 2)
    tris.closePrimitive()
    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode(name)
    node.addGeom(geom)
    return node


def parked_pose_for_platform(platform):
    """Return a parked pose in POC coordinates, without changing active limits.

    MDX CAD moving points are already local offsets from the parked reference
    plane.  With the POC translation sign, move that reference plane to Z=0.
    Other platform variants retain the existing constrained best-fit behaviour.
    """
    target = np.full(6, float(platform.cfg.MUSCLE_MAX_LENGTH))
    if platform.selected_platform != "platform_v2":
        return solve_pose_from_muscle_lengths(
            platform.kin, target, np.zeros(6), platform.bounds
        )
    z_sign = float(platform.kin.FLIP_TRANSLATION[2])
    if abs(z_sign) < 1e-12:
        raise ValueError("MDX parked pose requires nonzero Z translation axis")
    pose = np.zeros(6, dtype=float)
    pose[2] = -platform.mid_z / z_sign
    _, actuator_lengths = platform.kin.inverse_kinematics(pose, return_lengths=True)
    muscle_lengths = np.asarray(actuator_lengths) - float(platform.cfg.FIXED_HARDWARE_LENGTH)
    rms = float(np.sqrt(np.mean((muscle_lengths - target) ** 2)))
    return pose, rms


def mdx_deck_top_z(local_attachments, clevis_height=17.0):
    """Place the MDX deck under all six 17 mm clevis bases (short pads fill gaps)."""
    return float(np.min(np.asarray(local_attachments, dtype=float)[:, 2]) - clevis_height)


def polygon_area(points):
    if len(points) < 3:
        return 0.0
    return 0.5 * sum(
        points[i][0] * points[(i + 1) % len(points)][1]
        - points[(i + 1) % len(points)][0] * points[i][1]
        for i in range(len(points))
    )


def _point_in_triangle(p, a, b, c):
    def cross(u, v): return u[0] * v[1] - u[1] * v[0]
    ab, bc, ca = b - a, c - b, a - c
    ap, bp, cp = p - a, p - b, p - c
    c1, c2, c3 = cross(ab, ap), cross(bc, bp), cross(ca, cp)
    return (c1 >= -1e-9 and c2 >= -1e-9 and c3 >= -1e-9) or (c1 <= 1e-9 and c2 <= 1e-9 and c3 <= 1e-9)


def triangulate_simple_polygon(points2):
    """Ear clipping for one simple closed 2D polygon; returns triangle indices."""
    pts = np.asarray(points2, dtype=float)
    if len(pts) < 3:
        return []
    # Remove repeated final vertex if present.
    if np.linalg.norm(pts[0] - pts[-1]) < 1e-9:
        pts = pts[:-1]
    n = len(pts)
    if n < 3:
        return []
    ccw = polygon_area([tuple(p) for p in pts]) > 0.0
    idx = list(range(n))
    triangles = []
    guard = 0
    while len(idx) > 3 and guard < n * n:
        guard += 1
        clipped = False
        for k in range(len(idx)):
            ia, ib, ic = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = pts[ia], pts[ib], pts[ic]
            cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
            if (ccw and cross <= 1e-9) or ((not ccw) and cross >= -1e-9):
                continue
            if any(_point_in_triangle(pts[j], a, b, c) for j in idx if j not in (ia, ib, ic)):
                continue
            triangles.append((ia, ib, ic)); del idx[k]; clipped = True; break
        if not clipped:
            return []
    if len(idx) == 3:
        triangles.append(tuple(idx))
    return triangles


def filled_polygon_node(name, world_points, local_points2, colour):
    triangles = triangulate_simple_polygon(local_points2)
    if not triangles:
        return None
    pts = np.asarray(world_points, dtype=float)
    if len(pts) > len(local_points2):
        pts = pts[:len(local_points2)]
    fmt = GeomVertexFormat.getV3c4()
    vdata = GeomVertexData(name, fmt, Geom.UHDynamic)
    vdata.setNumRows(len(pts))
    vw, cw = GeomVertexWriter(vdata, "vertex"), GeomVertexWriter(vdata, "color")
    for p in pts:
        vw.addData3(float(p[0]), float(p[1]), float(p[2])); cw.addData4(*colour)
    prim = GeomTriangles(Geom.UHStatic)
    for a, b, c in triangles:
        prim.addVertices(a, b, c); prim.addVertices(a, c, b)
    prim.closePrimitive()
    geom = Geom(vdata); geom.addPrimitive(prim)
    node = GeomNode(name); node.addGeom(geom)
    return node


def report_attachment_mismatch(label, visual_points, poc_points, platform, visual_path):
    """Print indexed XYZ differences without modifying the geometry or tolerance."""
    visual_points = np.asarray(visual_points, dtype=float)
    poc_points = np.asarray(poc_points, dtype=float)
    print(f"ERROR: {label} attachment coordinates do not match")
    print(f"  POC selected platform: {platform.selected_platform}")
    print(f"  POC config module    : {platform.config_module}")
    print(f"  Visual geometry file : {visual_path}")
    print("  Units: mm; delta = visual - POC; mismatch tolerance = 0.01 mm")
    if visual_points.shape != poc_points.shape:
        print(f"  Shape mismatch: visual={visual_points.shape}, POC={poc_points.shape}")
        return
    for i, (v, q) in enumerate(zip(visual_points, poc_points)):
        delta = v - q
        if not np.allclose(v, q, atol=0.01):
            print(f"  {label} {i}: visual=({v[0]:.3f}, {v[1]:.3f}, {v[2]:.3f}) "
                  f"POC=({q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}) "
                  f"delta=({delta[0]:+.3f}, {delta[1]:+.3f}, {delta[2]:+.3f})")


class PlatformVisualizer(ShowBase):
    def __init__(self, poc_root, config_path, visual_override=None, fullscreen_override=None, selected_platform=None, demo_only=False):
        self.config_path = Path(config_path).resolve()
        self.app_config = json.loads(self.config_path.read_text())
        self.update_hz = float(self.app_config.get("update_hz", 20))
        wcfg = self.app_config["window"]
        width, height = int(wcfg["width"]), int(wcfg["height"])
        if width <= 0 or height <= 0:
            raise ValueError("window.width and window.height must be positive")
        self.base_window_size = (width, height)
        self.window_scale_step = 0
        fullscreen = bool(wcfg.get("fullscreen", False)) if fullscreen_override is None else fullscreen_override
        self.fullscreen_800x480 = bool(fullscreen and (width, height) == (800, 480))

        loadPrcFileData("", "sync-video false")
        loadPrcFileData("", "show-frame-rate-meter false")
        loadPrcFileData("", "textures-power-2 none")
        loadPrcFileData("", f"clock-mode limited\nclock-frame-rate {self.update_hz:g}")
        super().__init__()
        props = WindowProperties(); props.setTitle("MDX Platform Twin")
        props.setSize(width, height); props.setFullscreen(fullscreen)
        self.win.requestProperties(props)
        self.setBackgroundColor(*BG); self.disableMouse()
        self.camLens.setFov(44.0); self.camLens.setNearFar(10.0, 10000.0)
        self.perspective_lens = self.camLens
        self.orthographic_lens = OrthographicLens()
        self.orthographic_lens.setNearFar(10.0, 10000.0)
        self.projection = "perspective"

        self.platform = PocPlatform(poc_root, selected_platform=selected_platform)
        profiles = self.app_config.get("platform_profiles", {})
        self.platform_profile = dict(profiles.get(self.platform.selected_platform, {}))
        if self.platform_profile:
            print(f"Visualizer profile: {self.platform.selected_platform}")
        else:
            print(f"Visualizer profile: default ({self.platform.selected_platform})")

        def merged_section(name):
            merged = dict(self.app_config.get(name, {}))
            merged.update(self.platform_profile.get(name, {}))
            return merged

        self.platform_visual_cfg = merged_section("platform_visual")
        self.moving_platform_cfg = merged_section("moving_platform")
        self.fixed_platform_cfg = merged_section("fixed_platform")
        self.fixed_frame_cfg = merged_section("fixed_frame")
        self.chair_cfg = merged_section("chair")
        # A platform may override only support height without losing the global
        # width, depth or colour settings.
        self.chair_cfg["support"] = {
            **self.app_config.get("chair", {}).get("support", {}),
            **self.platform_profile.get("chair", {}).get("support", {}),
        }
        self.geometry_cfg = merged_section("geometry")

        self.pose = np.zeros(6, dtype=float)
        self.mode = "DEMO"
        self.demo = CannedMotion(self.platform.bounds); self.demo_t0 = time.monotonic(); self.demo_stage = "HEAVE"
        self.solver_rms = None

        # A missing/invalid mode requires an explicit in-window selection.
        # Never silently bind UDP 995 when the operator has not chosen EasyIP.
        configured_mode = None if demo_only else self.app_config.get("input_mode")
        self.input_mode = (configured_mode.strip().lower()
                           if isinstance(configured_mode, str) else None)
        if self.input_mode not in ("twin", "easyip"):
            self.input_mode = None
        ucfg = self.app_config["udp"]
        self.udp_stale_seconds = float(ucfg.get("stale_seconds", 0.5))
        self.receiver = None
        self.easyip_solve_interval_s = 1.0 / max(1.0, float(self.app_config.get("easyip_solve_hz", 10.0)))
        self._last_easyip_solve_time = 0.0
        self.easyip_max_solver_iterations = max(1, min(35, int(self.app_config.get("easyip_max_solver_iterations", 8))))
        self._last_twin_rejection = None
        if self.input_mode is not None:
            self._start_receiver(self.input_mode)
        self.prop_command_word = None  # Last received prop COMMAND, not measured state.
        self.udp_state = "WAITING"

        pcfg = self.app_config.get("pressure_model", {})
        pressure_files = []
        for item in pcfg.get("files", ["pressure_data/PtoD_40.csv", "pressure_data/PtoD_80.csv"]):
            path = Path(item)
            if not path.is_absolute():
                path = self.config_path.parent / path
            pressure_files.append(path.resolve())
        self.pressure_model = EmpiricalPressureModel(
            pressure_files,
            direction_deadband_mbar=float(pcfg.get("direction_deadband_mbar", 25.0)),
        )
        self.pressure_load = float(pcfg.get("empirical_load", 40.0))
        self.muscle_dynamics = MuscleDynamics(
            enabled=bool(pcfg.get("slew_enabled", True)),
            contraction_rate_mm_s=float(pcfg.get("max_contraction_rate_mm_s", 100.0)),
            extension_rate_mm_s=float(pcfg.get("max_extension_rate_mm_s", 100.0)),
        )
        self.last_processed_packet_count = None
        self.previous_solver_lengths = None
        self.solver_iterations = None
        self.solver_solve_ms = None
        self.solver_max_length_delta_mm = None
        self.solver_peak_length_delta_mm = 0.0
        self.solver_slew_cap_mm = None
        self.packet_diagnostics = PacketDiagnostics()
        self.packet_diag = None

        vcfg = self.platform_visual_cfg
        visual_path = Path(visual_override) if visual_override else Path(vcfg["file"])
        if not visual_path.is_absolute():
            visual_path = self.config_path.parent / visual_path
        self.visual = load_visual_geometry(visual_path)
        if bool(self.geometry_cfg.get("use_visual_geometry_for_solver", False)):
            if self.visual.moving_attachments is None:
                raise ValueError(
                    f"Visual geometry {visual_path} cannot override solver geometry without moving attachments"
                )
            self.platform.apply_visual_geometry(
                self.visual.fixed_attachments, self.visual.moving_attachments
            )
        if not np.allclose(self.visual.fixed_attachments, self.platform.base_points, atol=0.01):
            report_attachment_mismatch(
                "fixed", self.visual.fixed_attachments, self.platform.base_points,
                self.platform, visual_path,
            )
            raise ValueError(
                f"Compiled visual geometry {visual_path} does not match the current POC fixed attachment coordinates; "
                "see the indexed coordinate differences above"
            )
        if self.visual.moving_attachments is not None and not np.allclose(
            self.visual.moving_attachments, self.platform.platform_local_points, atol=0.01
        ):
            report_attachment_mismatch(
                "moving", self.visual.moving_attachments, self.platform.platform_local_points,
                self.platform, visual_path,
            )
            raise ValueError(
                f"Compiled visual geometry {visual_path} does not match the current POC moving attachment coordinates; "
                "see the indexed coordinate differences above"
            )
        # MDX uses the actual CAD parked reference plane (outside the normal
        # +-80 mm operational heave range); other platforms keep the old solve.
        self.parked_target_lengths = np.full(6, float(self.platform.cfg.MUSCLE_MAX_LENGTH))
        self.parked_pose, self.parked_solver_rms = parked_pose_for_platform(self.platform)
        if self.parked_solver_rms > 1.0:
            print(f"WARNING: parked muscle lengths differ from full extension "
                  f"(RMS error: {self.parked_solver_rms:.1f} mm)")
        if self.platform.selected_platform == "platform_v2":
            print(f"MDX CAD parked pose: heave={self.parked_pose[2]:.1f} mm, "
                  f"length RMS={self.parked_solver_rms:.2f} mm")
        self.visual_path = visual_path.resolve()
        self.geometry_name = self.visual_path.name
        self.source_name = self.visual.source_file
        self.source_hash = self.visual.source_sha256
        self.source_path = self.visual_path.parent / self.source_name
        self.source_hash_state = "not checked"
        if self.source_path.is_file() and self.source_hash:
            actual_hash = hashlib.sha256(self.source_path.read_bytes()).hexdigest()
            self.source_hash_state = "OK" if actual_hash == self.source_hash else "MISMATCH"
            if actual_hash != self.source_hash:
                print("WARNING: compiled visual geometry is stale:")
                print(f"  source file: {self.source_path}")
                print(f"  JSON hash  : {self.source_hash}")
                print(f"  source hash: {actual_hash}")
        elif self.source_path.is_file():
            self.source_hash_state = "JSON has no hash"
        else:
            self.source_hash_state = "source not found"

        print(f"Visualizer geometry: {self.visual_path} (source hash: {self.source_hash_state})")

        ccfg = self.app_config["camera"]
        # Open one right-arrow step clockwise from the configured camera angle.
        # Reset Camera/Home returns to this same starting view.
        self.camera_default_azimuth = float(ccfg["azimuth_deg"]) + float(ccfg["azimuth_step_deg"])
        self.camera_azimuth = self.camera_default_azimuth
        # Convert the legacy horizontal radius / absolute camera height to a
        # true spherical orbit. This reproduces the old starting eye position.
        horizontal_radius = float(ccfg["radius_mm"])
        initial_dz = float(ccfg["height_mm"]) - float(ccfg["target_xyz_mm"][2])
        self.camera_default_radius = math.hypot(horizontal_radius, initial_dz)
        self.camera_radius = self.camera_default_radius
        self.camera_default_elevation = math.degrees(math.atan2(initial_dz, horizontal_radius))
        self.camera_elevation = self.camera_default_elevation
        self.camera_target = np.asarray(ccfg["target_xyz_mm"], dtype=float)
        self.camera_azimuth_step = float(ccfg["azimuth_step_deg"])
        self.camera_elevation_step = float(ccfg.get("elevation_step_deg", 5.0))
        self.camera_elevation_min = max(-90.0, float(ccfg.get("elevation_min_deg", -90.0)))
        self.camera_elevation_max = min(90.0, float(ccfg.get("elevation_max_deg", 90.0)))
        self.camera_radius_step = float(ccfg.get("radius_step_mm", 150.0))
        self.camera_radius_min = float(ccfg.get("radius_min_mm", 1200.0))
        self.camera_radius_max = float(ccfg.get("radius_max_mm", 6000.0))
        # Keep the existing fullscreen Pi camera-distance adjustment.
        if self.fullscreen_800x480:
            self.camera_default_radius = min(self.camera_radius_max,
                max(self.camera_radius_min, self.camera_default_radius + 5 * self.camera_radius_step))
            self.camera_radius = self.camera_default_radius
        # Match perspective framing when first switching to orthographic.
        self.ortho_default_height = 2.0 * self.camera_default_radius * math.tan(
            math.radians(self.perspective_lens.getFov()[1]) / 2.0)
        self.ortho_height = self.ortho_default_height
        self.ortho_zoom_step = float(ccfg.get("ortho_zoom_step", 1.1))
        self.camera_view_z_offset = 0.0
        self.camera_view_z_step = float(ccfg.get("view_z_step_mm", 50.0))
        self.camera_view_z_min = float(ccfg.get("view_z_min_mm", -600.0))
        self.camera_view_z_max = float(ccfg.get("view_z_max_mm", 600.0))

        self.fixed_root = self.render.attachNewNode("fixed")
        self.dynamic_root = self.render.attachNewNode("dynamic")
        self.labels_root = self.render.attachNewNode("labels")
        # One platform-local transform for the visual deck and its clevises.
        # This remains independent of camera/screen UI and POC kinematics.
        self.platform_model_mount_np = self.dynamic_root.attachNewNode("moving-platform-mount")
        self.mdx_deck_top_z = (mdx_deck_top_z(self.platform.platform_local_points)
                               if self.platform.selected_platform == "platform_v2" else 0.0)
        self.platform_surface_np = None; self.platform_outline_np = None; self.muscles_np = None
        self.pose_diff_np = None
        self.pose_diff_enabled = False
        self.pose_diff_pose = None
        self.platform_model_np = None
        self.use_platform_model = False
        self.fixed_model_np = None
        self.use_fixed_model = False
        self.moving_markers = []
        self.numbers_visible = False  # Start with labels hidden; N toggles them
        self.chair_np = None
        self.chair_mount_np = None
        self.chair_support_np = None
        self.chair_offset_np = None
        self.chair_local_origin = None

        self._setup_lights(); self._build_fixed_model(); self._build_fixed_scene(); self._build_labels(); self._build_moving_markers(); self._build_moving_platform_model(); self._build_chair()
        self._build_ui(); self._setup_keys(); self._apply_camera(); self._update_dynamic_scene()
        if self.receiver is None and not demo_only:
            self.ui.show_input_chooser()
        if not self.numbers_visible: self.labels_root.hide()
        self._last_status_time = 0.0; self._frames = 0; self._fps = 0.0; self._fps_t0 = time.monotonic()
        self.taskMgr.add(self._update_task, "platform-update", sort=20)
        self.accept("window-event", self._window_event)

    def _setup_lights(self):
        amb = AmbientLight("ambient"); amb.setColor((0.58, 0.58, 0.62, 1.0)); self.render.setLight(self.render.attachNewNode(amb))
        sun = DirectionalLight("sun"); sun.setColor((0.76, 0.76, 0.76, 1.0)); sn = self.render.attachNewNode(sun); sn.setHpr(-35, -55, 0); self.render.setLight(sn)

    def _build_fixed_model(self):
        cfg = self.fixed_platform_cfg
        if not bool(cfg.get("enabled", False)):
            return

        model_path = Path(cfg.get("model", ""))
        if not model_path.is_absolute():
            model_path = self.config_path.parent / model_path
        model_path = model_path.resolve()
        fallback = bool(cfg.get("procedural_fallback", True))
        if not model_path.is_file():
            if fallback:
                print(f"WARNING: fixed-platform model not found; using procedural frame: {model_path}")
                return
            raise FileNotFoundError(f"Fixed-platform model not found: {model_path}")

        try:
            self.fixed_model_np = self.loader.loadModel(Filename.fromOsSpecific(str(model_path)))
        except (OSError, IOError, RuntimeError) as exc:
            self.fixed_model_np = None
            if fallback:
                print(
                    "WARNING: Panda3D could not load fixed-platform model; "
                    f"using procedural frame: {model_path} ({exc})"
                )
                return
            raise RuntimeError(f"Panda3D could not load fixed-platform model: {model_path}") from exc

        if self.fixed_model_np.isEmpty():
            self.fixed_model_np = None
            if fallback:
                print(f"WARNING: Panda3D returned an empty fixed-platform model; using procedural frame: {model_path}")
                return
            raise RuntimeError(f"Panda3D returned an empty fixed-platform model: {model_path}")

        self.fixed_model_np.reparentTo(self.fixed_root)
        # The imported CAD model includes the base and uprights; tint both to
        # match the frame instead of changing only procedural fallback parts.
        self.fixed_model_np.setColor(*FRAME_COLOUR)
        self.fixed_model_np.setDepthTest(True)
        self.fixed_model_np.setDepthWrite(True)
        self.use_fixed_model = True
        print(f"Fixed-platform model: {model_path.name}")

    def _build_fixed_scene(self):
        rail_cfg = self.fixed_frame_cfg
        rail_width = float(rail_cfg.get("width_mm", 50.0))
        rail_height = float(rail_cfg.get("height_mm", 70.0))
        # MDX CAD upright bases are Z=-18/-17 mm in POC coordinates.  Keep
        # procedural rails aligned with them rather than the former Z=0 top.
        rail_top_z = float(rail_cfg.get("top_z_mm", -18.0 if self.platform.selected_platform == "platform_v2" else 0.0))
        if rail_width <= 0.0 or rail_height <= 0.0:
            raise ValueError("fixed_frame width_mm and height_mm must both be > 0")

        if not self.use_fixed_model:
            # CAD frame lines are centre lines in the nominal attachment-plane XY
            # system. Fall back to the legacy closed outline when loading a v1 file.
            frame_lines = list(self.visual.fixed_frame_lines)
            if not frame_lines:
                outline = np.asarray(self.visual.fixed_outline, dtype=float)
                frame_lines = [
                    np.asarray([outline[i], outline[(i + 1) % len(outline)]], dtype=float)
                    for i in range(len(outline))
                ]

            for i, line in enumerate(frame_lines):
                a, b = line
                dx, dy = float(b[0] - a[0]), float(b[1] - a[1])
                length = math.hypot(dx, dy)
                if length <= 1e-9:
                    continue
                rail = self.fixed_root.attachNewNode(
                    make_box_node(f"ground-rail-{i}", length, rail_width, rail_height, FRAME_COLOUR)
                )
                rail.setPos(float((a[0] + b[0]) * 0.5), float((a[1] + b[1]) * 0.5), rail_top_z - rail_height)
                rail.setH(math.degrees(math.atan2(dy, dx)))
                rail.setDepthTest(True); rail.setDepthWrite(True)

            # Procedural fallback represents uprights as cylinders.  CAD-model
            # profiles can instead load the true rectangular/extruded uprights.
            ztop = float(np.max(self.platform.base_points[:, 2]))
            for ui, upright in enumerate(self.visual.uprights):
                x, y = upright.center_xy
                u = self.fixed_root.attachNewNode(
                    make_cylinder_node(f"upright-{ui}", upright.radius, ztop - rail_top_z, UPRIGHT_COLOUR)
                )
                u.setPos(float(x), float(y), rail_top_z)
                for ch in upright.channels:
                    path = [[[float(x), float(y), ztop], self.platform.base_points[ch].tolist()]]
                    support = self.fixed_root.attachNewNode(
                        line_node(f"support-{ui}-{ch}", path, FIXED_JOINT_COLOUR, 3.0)
                    )
                    support.setDepthTest(True)

        # Attachment markers remain visible even when the CAD base model is used.
        for i, p in enumerate(self.platform.base_points):
            marker = self.fixed_root.attachNewNode(
                make_cylinder_node(f"fixed-joint-{i}", 18.0, 24.0, FIXED_JOINT_COLOUR, 10)
            )
            marker.setPos(float(p[0]), float(p[1]), float(p[2] - 12.0))

    def _build_moving_markers(self):
        if self.platform.selected_platform == "platform_v2":
            # Each 17 mm high square pyramid has its tip at the actual CAD
            # joint centre.  CAD joint Z varies from -8 to -2 mm, so 0-6 mm
            # deck-coloured pads make every pyramid base meet local deck top.
            deck_z = self.mdx_deck_top_z
            for i, (x, y, z) in enumerate(self.platform.platform_local_points):
                pyramid = self.platform_model_mount_np.attachNewNode(
                    make_pyramid_node(f"moving-clevis-{i}", 17.0, 17.0,
                                      (0.72, 0.75, 0.78, 1.0))
                )
                pyramid.setPos(float(x), float(y), float(z))
                pyramid.setTwoSided(True)
                self.moving_markers.append(pyramid)
                pad_height = float(z) - 17.0 - deck_z
                if pad_height > 0.001:
                    pad = self.platform_model_mount_np.attachNewNode(
                        make_box_node(f"moving-deck-pad-{i}", 17.0, 17.0,
                                      pad_height, PLATFORM_COLOUR)
                    )
                    pad.setPos(float(x), float(y), deck_z)
                    pad.setTwoSided(True)
            return
        for i in range(6):
            m = self.dynamic_root.attachNewNode(make_cylinder_node(f"moving-joint-{i}", 14.0, 20.0, MOVING_JOINT_COLOUR, 10)); self.moving_markers.append(m)

    def _build_labels(self):
        self.label_nps = []
        for i, p in enumerate(self.platform.base_points):
            text = TextNode(f"channel-{i}"); text.setText(str(i)); text.setTextColor(*NUMBER_COLOUR); text.setAlign(TextNode.ACenter)
            nptext = self.labels_root.attachNewNode(text); nptext.setScale(55.0); nptext.setPos(float(p[0]), float(p[1]), float(p[2] + 42.0)); nptext.setBillboardPointEye(); self.label_nps.append(nptext)

    def _build_moving_platform_model(self):
        cfg = self.moving_platform_cfg
        if not bool(cfg.get("enabled", True)):
            return

        model_path = Path(cfg.get("model", "assets/mdx_moving_platform.egg"))
        if not model_path.is_absolute():
            model_path = self.config_path.parent / model_path
        model_path = model_path.resolve()
        fallback = bool(cfg.get("procedural_fallback", True))
        if not model_path.is_file():
            if fallback:
                print(f"WARNING: moving-platform model not found; using procedural outline: {model_path}")
                return
            raise FileNotFoundError(f"Moving-platform model not found: {model_path}")

        try:
            self.platform_model_np = self.loader.loadModel(Filename.fromOsSpecific(str(model_path)))
        except (OSError, IOError, RuntimeError) as exc:
            self.platform_model_np = None
            if fallback:
                print(
                    "WARNING: Panda3D could not load moving-platform model; "
                    f"using procedural outline: {model_path} ({exc})"
                )
                return
            raise RuntimeError(
                f"Panda3D could not load moving-platform model: {model_path}"
            ) from exc

        if self.platform_model_np.isEmpty():
            self.platform_model_np = None
            if fallback:
                print(f"WARNING: Panda3D returned an empty moving-platform model; using procedural outline: {model_path}")
                return
            raise RuntimeError(f"Panda3D returned an empty moving-platform model: {model_path}")

        self.platform_model_np.reparentTo(self.platform_model_mount_np)
        thickness = float(cfg.get("thickness_mm", 40.0))
        if self.platform.selected_platform == "platform_v2":
            # Align CAD-deck top and underside without changing the source EGG
            # or the kinematic attachment points.  POC MDX underside is Z=-80.
            underside_z = float(self.platform.cfg.PLATFORM_UNDERSIDE_Z)
            display_depth = self.mdx_deck_top_z - underside_z
            if thickness <= 0 or display_depth <= 0:
                raise ValueError("MDX moving-platform thickness/underside geometry is invalid")
            self.platform_model_np.setSz(display_depth / thickness)
            self.platform_model_np.setZ(self.mdx_deck_top_z)
            print(f"MDX visual deck: top Z={self.mdx_deck_top_z:.1f} mm, "
                  f"underside Z={underside_z:.1f} mm (parked POC frame)")
        self.platform_model_np.setDepthTest(True)
        self.platform_model_np.setDepthWrite(True)
        self.use_platform_model = True
        if self.platform.selected_platform != "platform_v2":
            print(f"Moving-platform model: {model_path.name} (underside Z={-thickness:g} mm)")

    def _update_moving_platform_model_pose(self):
        # The local mount also carries MDX clevises and small deck pads, even
        # if the moving-platform EGG is unavailable (procedural fallback).
        origin, forward, up = self.platform.platform_pose_basis(self.pose)
        self.platform_model_mount_np.setPos(*map(float, origin))
        self.platform_model_mount_np.lookAt(
            Point3(*map(float, origin + forward)),
            Vec3(*map(float, up)),
        )

    def _build_chair(self):
        cfg = self.chair_cfg
        if not bool(cfg.get("enabled", False)):
            return

        model_path = Path(cfg.get("model", "assets/seat_lowpoly.egg"))
        if not model_path.is_absolute():
            model_path = self.config_path.parent / model_path
        model_path = model_path.resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f"Chair model not found: {model_path}")

        # Mount origin is the centroid of the three fixed upright centres, projected
        # onto the neutral moving-platform plane.  Local mount axes remain POC axes,
        # so chair x_offset_mm is unambiguously positive toward POC +X (platform front).
        upright_xy = np.asarray([u.center_xy for u in self.visual.uprights], dtype=float)
        centroid_xy = np.mean(upright_xy, axis=0)
        # Seat support rests on the visually corrected MDX deck top.
        self.chair_local_origin = np.array([
            centroid_xy[0], centroid_xy[1],
            self.platform.mid_z + self.mdx_deck_top_z,
        ], dtype=float)

        self.chair_x_offset_mm = float(cfg.get("x_offset_mm", 0.0))
        self.chair_y_offset_mm = float(cfg.get("y_offset_mm", 0.0))
        self.chair_yaw_offset_deg = float(cfg.get("yaw_offset_deg", 180.0))
        self.chair_z_offset_mm = float(cfg.get("z_offset_mm", 0.0))

        support_cfg = cfg.get("support", {})
        self.chair_support_width_mm = float(support_cfg.get("width_mm", 325.661))
        self.chair_support_depth_mm = float(support_cfg.get("depth_mm", 303.876))
        self.chair_support_height_mm = float(support_cfg.get("height_mm", 300.0))
        if self.chair_support_width_mm <= 0.0 or self.chair_support_depth_mm <= 0.0 or self.chair_support_height_mm < 0.0:
            raise ValueError("chair.support width_mm and depth_mm must be > 0; height_mm must be >= 0")
        support_colour = support_cfg.get("colour", CHAIR_COLOUR)
        if len(support_colour) != 4:
            raise ValueError("chair.support.colour must contain four RGBA values")

        self.chair_mount_np = self.dynamic_root.attachNewNode("chair-mount")
        # Offsets and yaw are applied to one shared node so the procedural support
        # and chair remain coincident and move as one payload assembly.
        self.chair_offset_np = self.chair_mount_np.attachNewNode("chair-offset")
        self.chair_offset_np.setPos(self.chair_x_offset_mm, self.chair_y_offset_mm, 0.0)
        self.chair_offset_np.setH(self.chair_yaw_offset_deg)

        self.chair_support_np = None
        if self.chair_support_height_mm > 0.0:
            self.chair_support_np = self.chair_offset_np.attachNewNode(
                make_box_node(
                    "chair-support",
                    self.chair_support_width_mm,
                    self.chair_support_depth_mm,
                    self.chair_support_height_mm,
                    tuple(map(float, support_colour)),
                )
            )
            self.chair_support_np.setDepthTest(True)
            self.chair_support_np.setDepthWrite(True)

        self.chair_np = self.loader.loadModel(Filename.fromOsSpecific(str(model_path)))
        if self.chair_np.isEmpty():
            raise RuntimeError(f"Panda3D could not load chair model: {model_path}")
        self.chair_np.reparentTo(self.chair_offset_np)
        # seat_lowpoly.egg is normalized so XY=0 is the former support centroid
        # and Z=0 is the lowest remaining chair geometry. It therefore sits
        # directly on the configurable procedural support.
        self.chair_np.setPos(0.0, 0.0, self.chair_support_height_mm + self.chair_z_offset_mm)
        # Preserve colours/materials embedded in the EGG (dark shell, red padding).
        # An optional explicit override remains available for simple single-colour assets.
        colour_override = cfg.get("colour_override")
        if colour_override is not None:
            if len(colour_override) != 4:
                raise ValueError("chair.colour_override must contain four RGBA values")
            self.chair_np.setColor(*map(float, colour_override))
        self.chair_np.setDepthTest(True)
        self.chair_np.setDepthWrite(True)

        print(f"Chair visual: {model_path.name} (offset X={self.chair_x_offset_mm:+.1f}, "
              f"Y={self.chair_y_offset_mm:+.1f} mm; yaw={self.chair_yaw_offset_deg:+.1f} deg)")

    def _update_chair_pose(self):
        if self.chair_np is None:
            return
        pose = np.asarray(self.pose, dtype=float)
        translation = pose[:3] * self.platform.kin.FLIP_TRANSLATION
        rpy = pose[3:6] * self.platform.kin.FLIP_ROTATION
        rotation = self.platform.kin.calc_rotation(rpy)

        origin = rotation @ self.chair_local_origin + translation
        # Keep the chair mount aligned with the POC platform axes.  This means
        # child X/Y offsets remain platform-relative even while the platform moves.
        forward = rotation @ np.array([0.0, 1.0, 0.0], dtype=float)
        up = rotation @ np.array([0.0, 0.0, 1.0], dtype=float)

        self.chair_mount_np.setPos(*map(float, origin))
        target = origin + forward
        self.chair_mount_np.lookAt(
            Point3(*map(float, target)),
            Vec3(*map(float, up)),
        )

    def _build_ui(self):
        self.help_visible = False
        self.diagnostics_visible = True
        self.ui = VisualizerUI(
            aspect2d=self.aspect2d,
            top_right=self.a2dTopRight,
            bottom_right=self.a2dBottomRight,
            on_mode=self._set_mode,
            on_input=self._choose_input,
            on_toggle_text=self._toggle_diagnostics,
            on_quit=self.userExit,
            on_toggle_pose_diff=self._toggle_pose_diff,
            mode=self.mode,
            input_mode=self.input_mode,
            diagnostics_visible=self.diagnostics_visible,
        )

    def _start_receiver(self, mode):
        if mode == "easyip":
            receiver = EasyIpReceiver(int(self.app_config.get("easyip_port", 995)))
        else:
            receiver = TwinUdpReceiver(int(self.app_config["udp"].get("port", 10020)))
        self.input_mode = mode
        self.receiver = receiver
        print(f"Visualizer input: {mode.upper()} UDP {receiver.port}")

    def _choose_input(self, mode):
        try:
            self._start_receiver(mode)
        except OSError as exc:
            print(f"Cannot start {mode} input: {exc}")
            self.ui.show_input_error(mode, exc)
            return
        self.ui.hide_input_chooser()
        self._last_status_time = 0.0
        self._refresh_touch_controls()

    def _refresh_touch_controls(self):
        self.ui.refresh_touch_controls(self.mode, self.input_mode, self.diagnostics_visible)
        self.ui.set_pose_diff_enabled(self.pose_diff_enabled)

    def _toggle_pose_diff(self):
        self.pose_diff_enabled = not self.pose_diff_enabled
        self.ui.set_pose_diff_enabled(self.pose_diff_enabled)
        self._update_pose_diff_scene()

    def _update_pose_diff_scene(self):
        """One lightweight outline; never duplicate muscles, clevises or chair."""
        if self.pose_diff_np is not None:
            self.pose_diff_np.removeNode()
            self.pose_diff_np = None
        if not self.pose_diff_enabled or self.pose_diff_pose is None:
            return
        local_outline = self.visual.moving_outline.tolist()
        outline = self.platform.transform_xy_paths([local_outline], self.pose_diff_pose)[0]
        if self.mdx_deck_top_z:
            _origin, _forward, up = self.platform.platform_pose_basis(self.pose_diff_pose)
            outline = outline + self.mdx_deck_top_z * up
        self.pose_diff_np = self.dynamic_root.attachNewNode(
            line_node("pose_diff-outline", [outline], POSE_DIFF_COLOUR, 3.0, [True])
        )
        self.pose_diff_np.setDepthTest(False)
        self.pose_diff_np.setBin("fixed", 40)

    def _setup_keys(self):
        self.accept("escape", self.userExit); self.accept("q", self.userExit)
        self.accept("arrow_left", self._orbit, [-self.camera_azimuth_step]); self.accept("a", self._orbit, [-self.camera_azimuth_step])
        self.accept("arrow_right", self._orbit, [self.camera_azimuth_step]); self.accept("d", self._orbit, [self.camera_azimuth_step])
        self.accept("arrow_up", self._camera_vertical, [self.camera_elevation_step]); self.accept("w", self._camera_vertical, [self.camera_elevation_step])
        self.accept("arrow_down", self._camera_vertical, [-self.camera_elevation_step]); self.accept("s", self._camera_vertical, [-self.camera_elevation_step])
        self.accept("+", self._camera_zoom, [-self.camera_radius_step]); self.accept("=", self._camera_zoom, [-self.camera_radius_step])
        self.accept("-", self._camera_zoom, [self.camera_radius_step]); self.accept("_", self._camera_zoom, [self.camera_radius_step])
        self.accept("page_up", self._camera_view_vertical, [self.camera_view_z_step])
        self.accept("page_down", self._camera_view_vertical, [-self.camera_view_z_step])
        self.accept("home", self._reset_camera); self.accept("c", self._reset_camera)
        self.accept("o", self._toggle_projection)
        self.accept("1", self._snap_camera_view, ["top"])
        self.accept("2", self._snap_camera_view, ["front"])
        self.accept("3", self._snap_camera_view, ["side"])
        self.accept("t", self._set_mode, ["DEMO"])
        self.accept("l", self._set_mode, ["LENGTH"])
        self.accept("f", self._set_mode, ["FESTO"])
        self.accept("r", self._hold_neutral)
        self.accept("n", self._toggle_numbers)
        self.accept("h", self._toggle_diagnostics)
        self.accept("f1", self._toggle_help)
        # Touchscreens report a swipe as a mouse press and release. One gesture
        # gives one movement step, identical to the corresponding arrow key.
        self.accept("mouse1", self._swipe_start)
        self.accept("mouse1-up", self._swipe_end)
        # Window resizing is distinct from the existing +/- camera zoom.
        for key in ("control-+", "control-=", "control-shift-="):
            self.accept(key, self._resize_window_steps, [1])
        for key in ("control--", "control-_"):
            self.accept(key, self._resize_window_steps, [-1])
        self.accept("control-0", self._reset_window_size)

    def _resize_window_steps(self, delta):
        if self.win is None or self.win.getProperties().getFullscreen():
            return
        # Each step is 25% of the dimensions in app_config.json.
        self.window_scale_step = max(0, min(12, self.window_scale_step + delta))
        self._apply_window_size()

    def _reset_window_size(self):
        if self.win is None or self.win.getProperties().getFullscreen():
            return
        self.window_scale_step = 0
        self._apply_window_size()

    def _apply_window_size(self):
        factor = 1.0 + 0.25 * self.window_scale_step
        width = round(self.base_window_size[0] * factor)
        height = round(self.base_window_size[1] * factor)
        props = WindowProperties()
        props.setSize(width, height)
        self.win.requestProperties(props)
        # Panda3D's aspect2d/OnscreenText uses window-relative coordinates.
        # Text size and positioning therefore scale naturally in pixels as
        # the aspect-preserving window grows; don't multiply font scale again.

    def _pointer_xy(self):
        if not self.mouseWatcherNode.hasMouse():
            return None
        pos = self.mouseWatcherNode.getMouse()
        return (float(pos.getX()), float(pos.getY()))

    def _swipe_start(self):
        width, height = self.win.getXSize(), self.win.getYSize()
        self.ui.begin_scene_swipe(self._pointer_xy(), width / max(height, 1))

    def _swipe_end(self):
        direction = self.ui.end_scene_swipe(
            self._pointer_xy(), (self.win.getXSize(), self.win.getYSize())
        )
        if direction == "right":
            self._orbit(self.camera_azimuth_step)
        elif direction == "left":
            self._orbit(-self.camera_azimuth_step)
        elif direction == "up":
            self._camera_vertical(self.camera_elevation_step)
        elif direction == "down":
            self._camera_vertical(-self.camera_elevation_step)

    def _orbit(self, delta): self.camera_azimuth = (self.camera_azimuth + delta) % 360.0; self._apply_camera()
    def _camera_vertical(self, delta): self.camera_elevation = min(self.camera_elevation_max, max(self.camera_elevation_min, self.camera_elevation + delta)); self._apply_camera()
    def _camera_zoom(self, delta):
        if self.projection == "orthographic":
            # delta < 0 is zoom in, as in perspective mode.
            factor = self.ortho_zoom_step if delta > 0 else 1.0 / self.ortho_zoom_step
            self.ortho_height = min(self.ortho_default_height * 8.0,
                                    max(self.ortho_default_height / 8.0, self.ortho_height * factor))
        else:
            self.camera_radius = min(self.camera_radius_max,
                                     max(self.camera_radius_min, self.camera_radius + delta))
        self._apply_camera()

    def _toggle_projection(self):
        self.projection = "orthographic" if self.projection == "perspective" else "perspective"
        self.cam.node().setLens(self.orthographic_lens if self.projection == "orthographic"
                                else self.perspective_lens)
        self._apply_camera()
    def _snap_camera_view(self, view):
        # World axes: front looks from +X; side from -Y; top from +Z.
        # At the top pole, azimuth -90 gives screen-right +X and up +Y.
        angles = {"top": (-90.0, 90.0),
                  "front": (0.0, 0.0),
                  "side": (-90.0, 0.0)}
        self.camera_azimuth, self.camera_elevation = angles[view]
        self._apply_camera()

    def _camera_view_vertical(self, delta): self.camera_view_z_offset = min(self.camera_view_z_max, max(self.camera_view_z_min, self.camera_view_z_offset + delta)); self._apply_camera()
    def _reset_camera(self):
        self.camera_azimuth = self.camera_default_azimuth
        self.camera_radius = self.camera_default_radius
        self.camera_elevation = self.camera_default_elevation
        self.ortho_height = self.ortho_default_height
        self.projection = "perspective"
        self.cam.node().setLens(self.perspective_lens)
        self.camera_view_z_offset = 0.0
        self._apply_camera()
    def _set_mode(self, mode):
        if self.receiver is None:
            print("Select EasyIP or Twin input in the startup dialog first")
            return
        if mode == "LENGTH" and self.input_mode == "easyip":
            print("LENGTH mode requires POC_TWIN packets; select input_mode=twin")
            return
        previous = self.mode
        self.mode = mode
        self.pose_diff_pose = None
        self._update_pose_diff_scene()
        if mode in ("LENGTH", "FESTO") and previous == "DEMO":
            # A packet buffered during Demo is not a live command for this mode.
            # Display the fully extended, zero-pressure platform until a NEW
            # valid packet arrives.  This is a visual command model, not a
            # physical parked/prop-engagement confirmation.
            _message, _arrival, count, _error = self.receiver.snapshot()
            self._waiting_for_live_packet = True
            self._mode_entry_packet_count = count
            self.pose = self.parked_pose.copy()
            self.solver_rms = self.parked_solver_rms
            self.udp_state = "WAITING"
            # Apply the parked pose now, even when the next frame has no new
            # packet or the scene-update task sees an unchanged pose.
            self._update_dynamic_scene()
        elif mode in ("LENGTH", "FESTO") and previous != mode:
            self._waiting_for_live_packet = False
        if mode == "DEMO":
            self.demo_t0 = time.monotonic()
        if mode in ("LENGTH", "FESTO") and previous != mode:
            # Process the current/latest UDP snapshot once on entry, then only
            # process subsequent newly received packets.
            self.last_processed_packet_count = None
        if mode == "FESTO" and previous != "FESTO":
            self.pressure_model.reset()
            self.muscle_dynamics.reset()
            self.previous_solver_lengths = None
            self.solver_iterations = None
            self.solver_solve_ms = None
            self.solver_max_length_delta_mm = None
            self.solver_peak_length_delta_mm = 0.0
            self.solver_slew_cap_mm = None
            self.packet_diagnostics.reset()
            self.packet_diag = None
        self._refresh_touch_controls()

    def _hold_neutral(self):
        self.mode = "HOLD"
        self.pose_diff_pose = None
        self._update_pose_diff_scene()
        self.pose[:] = 0.0
        self.solver_rms = None
        self.pressure_model.reset()
        self.muscle_dynamics.reset()
        self.last_processed_packet_count = None
        self.previous_solver_lengths = None
        self.solver_slew_cap_mm = None
        self.packet_diagnostics.reset()
        self.packet_diag = None
        self._refresh_touch_controls()
    def _toggle_numbers(self): self.numbers_visible = not self.numbers_visible; self.labels_root.show() if self.numbers_visible else self.labels_root.hide()
    def _toggle_diagnostics(self):
        self.diagnostics_visible = not self.diagnostics_visible
        self._last_status_time = 0.0  # Refresh the display on the next frame
        self._refresh_touch_controls()

    def _toggle_help(self):
        self.help_visible = not self.help_visible
        self.ui.show_help(self.help_visible)

    def _apply_camera(self):
        a = math.radians(self.camera_azimuth)
        e = math.radians(self.camera_elevation)
        t = self.camera_target.copy()
        t[2] += self.camera_view_z_offset
        cos_e = math.cos(e)
        eye = [
            t[0] + self.camera_radius * cos_e * math.cos(a),
            t[1] + self.camera_radius * cos_e * math.sin(a),
            t[2] + self.camera_radius * math.sin(e),
        ]
        self.camera.setPos(*map(float, eye))
        # Tangent to the orbit is a stable screen-up direction at BOTH poles.
        up = Vec3(-math.sin(e) * math.cos(a),
                  -math.sin(e) * math.sin(a), math.cos(e))
        self.camera.lookAt(Point3(*map(float, t)), up)
        if self.projection == "orthographic":
            aspect = (float(self.win.getXSize()) / max(1.0, float(self.win.getYSize())))
            self.orthographic_lens.setFilmSize(self.ortho_height * aspect, self.ortho_height)

    def _update_mode(self, now):
        if self.mode == "DEMO":
            self.pose_diff_pose = None
            self._update_pose_diff_scene()
            self.pose, self.demo_stage = self.demo.pose(now - self.demo_t0)
            self.udp_state = "IGNORED"
            return
        if self.mode not in ("LENGTH", "FESTO"):
            self.udp_state = "IGNORED"
            return

        if self.receiver is None:
            self.udp_state = "SELECT INPUT"
            return
        msg, arrival, packet_count, _err = self.receiver.snapshot()
        age = now - arrival if arrival else 999.0
        if msg is None:
            self.pose_diff_pose = None
            self._update_pose_diff_scene()
            self.udp_state = "WAITING"
            return
        if age > self.udp_stale_seconds:
            self.pose_diff_pose = None
            self._update_pose_diff_scene()
            self.udp_state = "STALE"
            return
        if getattr(self, "_waiting_for_live_packet", False):
            if packet_count <= self._mode_entry_packet_count:
                self.udp_state = "WAITING"
                return

        # EasyIP pressure writes carry no platform identifier.  The visualizer's
        # selected_platform chooses the pressure model/geometry for this source.
        if self.input_mode == "twin":
            platform = str(msg.get("platform", ""))
            if not self.platform.accepts_platform_id(platform):
                self.pose_diff_pose = None
                self._update_pose_diff_scene()
                self.udp_state = "WRONG PLATFORM"
                if platform != getattr(self, "_last_twin_rejection", None):
                    print(f"TWIN platform mismatch: packet={platform!r}; visualizer={getattr(self.platform, 'selected_platform', '?')!r}; "
                          f"accepted={sorted(getattr(self.platform, 'accepted_platform_ids', ()))!r}", flush=True)
                    self._last_twin_rejection = platform
                return
            self._last_twin_rejection = None

        self.udp_state = "LIVE"
        self._waiting_for_live_packet = False

        # Rendering may run faster than POC's UDP stream. Do the pressure/length
        # conversion and nonlinear solve exactly once for each newly received packet.
        if packet_count == self.last_processed_packet_count:
            return
        if self.input_mode == "easyip":
            self.prop_command_word = msg.get("prop_command_word")
        # CPU-limited Pi: keep only the latest packet, bound nonlinear solver
        # frequency in EasyIP mode, and render between solves.  Twin unchanged.
        if (self.input_mode == "easyip" and self.mode == "FESTO"
                and self._last_easyip_solve_time
                and now - self._last_easyip_solve_time < self.easyip_solve_interval_s):
            return
        self.last_processed_packet_count = packet_count
        self.pose_diff_pose = (pose_diff_pose_from_packet(msg)
                             if self.input_mode == "twin" else None)
        self._update_pose_diff_scene()
        if self.input_mode == "easyip":
            self._last_easyip_solve_time = now
        self.packet_diag = self.packet_diagnostics.update(msg, arrival, packet_count)

        if self.mode == "LENGTH":
            target_lengths = np.asarray(msg["muscle_length_mm"], dtype=float)
            self.pose, self.solver_rms = solve_pose_from_muscle_lengths(
                self.platform.kin, target_lengths, self.pose, self.platform.bounds
            )
        else:
            estimate = self.pressure_model.estimate(msg["command_pressure_mbar"], self.pressure_load)
            max_contraction = float(self.platform.cfg.MUSCLE_MAX_LENGTH - self.platform.cfg.MUSCLE_MIN_LENGTH)
            bounded = [min(max_contraction, max(0.0, float(v))) for v in estimate.contractions_mm]

            if self.packet_diag is not None and self.packet_diag.local_dt_s is not None:
                dt = self.packet_diag.local_dt_s
            else:
                dt = 1.0 / self.update_hz
            max_slew_rate = max(
                float(self.muscle_dynamics.contraction_rate_mm_s),
                float(self.muscle_dynamics.extension_rate_mm_s),
            )
            self.solver_slew_cap_mm = max(0.0, max_slew_rate) * dt if self.muscle_dynamics.enabled else None
            contractions = self.muscle_dynamics.step(bounded, dt)
            target_lengths = np.asarray(
                [float(self.platform.cfg.MUSCLE_MAX_LENGTH) - c for c in contractions],
                dtype=float,
            )

            if self.previous_solver_lengths is None:
                self.solver_max_length_delta_mm = 0.0
            else:
                self.solver_max_length_delta_mm = float(
                    np.max(np.abs(target_lengths - self.previous_solver_lengths))
                )
            self.previous_solver_lengths = target_lengths.copy()
            self.solver_peak_length_delta_mm = max(
                self.solver_peak_length_delta_mm, self.solver_max_length_delta_mm
            )

            solve_started = time.perf_counter()
            self.pose, self.solver_rms, solver_diag = solve_pose_from_muscle_lengths(
                self.platform.kin, target_lengths, self.pose, self.platform.bounds,
                max_iter=(getattr(self, "easyip_max_solver_iterations", 8) if self.input_mode == "easyip" else 35),
                return_diagnostics=True,
            )
            self.solver_solve_ms = (time.perf_counter() - solve_started) * 1000.0
            self.solver_iterations = int(solver_diag["iterations"])

    def _replace(self, old_np, node):
        if old_np is not None: old_np.removeNode()
        if node is None: return None
        n = self.dynamic_root.attachNewNode(node); n.setDepthTest(True); n.setDepthWrite(True); return n

    def _update_dynamic_scene(self):
        moving = self.platform.moving_points(self.pose)
        self._update_moving_platform_model_pose()

        if self.use_platform_model:
            if self.platform_surface_np is not None:
                self.platform_surface_np.removeNode(); self.platform_surface_np = None
            if self.platform_outline_np is not None:
                self.platform_outline_np.removeNode(); self.platform_outline_np = None
        else:
            local_outline = self.visual.moving_outline.tolist()
            world_outline = self.platform.transform_xy_paths([local_outline], self.pose)[0]
            if self.mdx_deck_top_z:
                _origin, _forward, up = self.platform.platform_pose_basis(self.pose)
                world_outline = world_outline + self.mdx_deck_top_z * up
            surface_node = filled_polygon_node("platform-surface", world_outline, local_outline, PLATFORM_COLOUR)
            self.platform_surface_np = self._replace(self.platform_surface_np, surface_node)
            if self.platform_surface_np is not None:
                self.platform_surface_np.setTwoSided(True)
            self.platform_outline_np = self._replace(
                self.platform_outline_np,
                line_node("platform-outline", [world_outline], PLATFORM_EDGE_COLOUR, 3.0, [True]),
            )

        # LineSegs thickness is in pixels; follow Ctrl+/- window-size scaling.
        window_scale = 1.0 + 0.25 * self.window_scale_step
        segs = LineSegs("muscles"); segs.setColor(*MUSCLE_COLOUR); segs.setThickness(12.0 * window_scale)
        for i in range(6):
            b, p = self.platform.base_points[i], moving[i]
            segs.moveTo(*map(float, b)); segs.drawTo(*map(float, p))
            if self.platform.selected_platform != "platform_v2":
                self.moving_markers[i].setPos(float(p[0]), float(p[1]), float(p[2] - 10.0))
        self.muscles_np = self._replace(self.muscles_np, segs.create())
        self._update_chair_pose()

    def _update_status(self, now):
        self._frames += 1
        elapsed = now - self._fps_t0
        if elapsed >= 1.0:
            self._fps = self._frames / elapsed; self._frames = 0; self._fps_t0 = now
        if now - self._last_status_time < 0.2: return
        self._last_status_time = now
        if self.receiver is None:
            self.ui.set_status("Select EasyIP or Twin input to continue")
            return
        _msg, arrival, count, err = self.receiver.snapshot()
        receiver_diag = self.receiver.diagnostics()
        age = now - arrival if arrival else None
        detail = self.demo_stage if self.mode == "DEMO" else self.udp_state if self.mode in ("LENGTH", "FESTO") else "NEUTRAL"
        rms = "-" if self.solver_rms is None else f"{self.solver_rms:.2f} mm"
        age_txt = "-" if age is None else f"{age:.2f}s"
        pressure_diag = ""
        if self.input_mode == "twin" and self.udp_state == "WRONG PLATFORM":
            platform_msg = str(_msg.get("platform", "?")) if _msg is not None else "?"
            pressure_diag += f"\nTWIN platform mismatch: packet={platform_msg!r}, selected={self.platform.selected_platform!r}"
        if self.input_mode == "twin" and self.udp_state == "STALE":
            pressure_diag += "\nTwin packet stale; verify continuous transmission"
        if self.mode == "FESTO":
            iterations = "-" if self.solver_iterations is None else str(self.solver_iterations)
            solve_ms = "-" if self.solver_solve_ms is None else f"{self.solver_solve_ms:.1f}"
            delta = "-" if self.solver_max_length_delta_mm is None else f"{self.solver_max_length_delta_mm:.2f}"
            slew_cap = "-" if self.solver_slew_cap_mm is None else f"{self.solver_slew_cap_mm:.2f}"
            if self.packet_diag is None:
                seq_txt = dseq_txt = rxstep_txt = rxdt_txt = srcdt_txt = "-"
                skipped_total = source_missing_total = 0
            else:
                seq_txt = "-" if self.packet_diag.sequence is None else str(self.packet_diag.sequence)
                dseq_txt = "-" if self.packet_diag.sequence_delta is None else str(self.packet_diag.sequence_delta)
                rxstep_txt = "-" if self.packet_diag.receiver_count_delta is None else str(self.packet_diag.receiver_count_delta)
                rxdt_txt = "-" if self.packet_diag.local_dt_s is None else f"{self.packet_diag.local_dt_s * 1000.0:.1f}"
                srcdt_txt = "-" if self.packet_diag.source_dt_s is None else f"{self.packet_diag.source_dt_s * 1000.0:.1f}"
                skipped_total = self.packet_diag.render_skipped_total
                source_missing_total = self.packet_diag.source_missing_total
            pressure_diag += (
                f"\nF seq:{seq_txt} dSeq:{dseq_txt} rxStep:{rxstep_txt}"
                f"\ndt rx/src:{rxdt_txt}/{srcdt_txt} ms  skipped:{skipped_total}/{source_missing_total}"
                f"\ndL:{delta} mm  cap:{slew_cap}  peak:{self.solver_peak_length_delta_mm:.2f}"
                f"\nsolver:{iterations} iter  {solve_ms} ms"
            )
        # H hides routine overlay text, but keeps receiver errors visible.
        if not self.diagnostics_visible:
            self.ui.set_status(f"UDP error: {err}" if err else "")
            return
        pose_diff_diag = ""
        if self.pose_diff_enabled and self.pose_diff_pose is not None:
            delta = self.pose[:3] - self.pose_diff_pose[:3]
            rotation_delta = np.degrees(self.pose[3:] - self.pose_diff_pose[3:])
            pose_diff_diag = (f"\nPose difference: XYZ {np.linalg.norm(delta):.1f} mm, "
                            f"RPY {np.linalg.norm(rotation_delta):.1f} deg")
        self.ui.set_status(
            f"PLATFORM TWIN  {self.mode}:{detail}\n"
            f"Geometry: {self.geometry_name} ({self.source_name})\n"
            f"FPS: {self._fps:4.1f}\n"
            + (
                f"{self.input_mode.upper()} UDP raw/valid/rej: {receiver_diag['datagram_count']}/{count}/{receiver_diag['rejected_count']}   age: {age_txt}\n"
                + (
                    f"UDP from: {receiver_diag['last_sender'][0]}:{receiver_diag['last_sender'][1]}\n"
                    if receiver_diag['last_sender'] is not None else "UDP from: -\n"
                )
                if self.mode != "DEMO" else ""
            )
            + (
                f"UDP reject: {receiver_diag['last_reject']}\n"
                if receiver_diag['last_reject'] else ""
            )
            + f"solver RMS: {rms}"
            + pressure_diag + pose_diff_diag
            + ("" if self.help_visible else "\npress F1 for help")
            + (f"\nUDP error: {err}" if err else "")
        )

    def _update_task(self, task):
        now = time.monotonic()
        previous_pose = np.asarray(self.pose).copy()
        self._update_mode(now)
        # Moving geometry is expensive to rebuild; don't recreate it if pose
        # hasn't changed (camera movement is handled by scene graph separately).
        if not np.array_equal(previous_pose, self.pose):
            self._update_dynamic_scene()
        self._update_status(now)
        return task.cont

    def _window_event(self, window):
        if window is not None and not window.getProperties().getOpen(): self.receiver.close() if self.receiver is not None else None

    def userExit(self):
        self.receiver.close() if self.receiver is not None else None; super().userExit()


def _is_poc_root(path):
    path = Path(path)
    return (
        (path / "poc_config.json").is_file()
        and (path / "config" / "platforms.py").is_file()
        and (path / "kinematics" / "kinematics_V3.py").is_file()
    )


def resolve_poc_root(arg, configured=None, visualizer_dir=None):
    if arg:
        p = Path(arg).expanduser().resolve()
        if _is_poc_root(p):
            return p
        raise SystemExit(f"Invalid POC root: {p}")
    if configured:
        # App configuration paths are relative to the visualizer directory,
        # irrespective of the current working directory.
        base = Path(visualizer_dir) if visualizer_dir is not None else Path(__file__).resolve().parent
        p = Path(configured).expanduser()
        if not p.is_absolute():
            p = base / p
        p = p.resolve()
        if _is_poc_root(p):
            return p
        raise SystemExit(f"Invalid app_config.json poc_root: {p}")
    # Self-contained default: do not automatically select an unrelated POC
    # installation in the current directory or the user's home directory.
    base = Path(visualizer_dir) if visualizer_dir is not None else Path(__file__).resolve().parent
    bundled = (base / "bundled_poc").resolve()
    if _is_poc_root(bundled):
        return bundled
    raise SystemExit(f"Bundled POC runtime not found: {bundled}; use --poc-root PATH")


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--poc-root")
    ap.add_argument("--config", default=str(here / "app_config.json"))
    ap.add_argument("--platform-visual", help="Override compiled platform visual JSON")
    ap.add_argument("--fullscreen", action="store_true", help="Force fullscreen")
    ap.add_argument("--windowed", action="store_true", help="Force windowed")
    args = ap.parse_args()
    config_path = Path(args.config).expanduser().resolve()
    app_config = json.loads(config_path.read_text(encoding="utf-8"))
    fullscreen = True if args.fullscreen else False if args.windowed else None
    root = resolve_poc_root(args.poc_root, app_config.get("poc_root"), here)
    if root == (here / "bundled_poc").resolve():
        print(f"POC source: bundled {root}")
    else:
        report_poc_bundle_differences(root, here / "bundled_poc")
    PlatformVisualizer(
        root,
        config_path,
        args.platform_visual,
        fullscreen,
        selected_platform=app_config.get("selected_platform"),
    ).run()


if __name__ == "__main__": main()
