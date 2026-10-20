# test_cad_parked_geometry_v46.py ver 1
"""CAD parked-pose and MDX attachment-rendering regressions (Panda3D-free)."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from test_easyip_visualizer_integration import import_visualizer_with_panda_stubs


# Same measured MDX CAD coordinates used in POC v149 (POC Z datum, mm).
MOVING = np.array([
    [716., 166., -8.], [-222., 609., -2.], [-348., 536., -2.],
    [-347., -536., -3.], [-221., -608., -2.], [716., -166., -8.],
])
FIXED = np.array([
    [438., 585., 1083.], [291., 622., 1083.], [-731., 76., 1037.],
    [-731., -76., 1037.], [289., -654., 1083.], [413., -566., 1083.],
])
MID_Z = 177.26466075505908


@pytest.fixture
def mod(monkeypatch):
    return import_visualizer_with_panda_stubs(monkeypatch)


class CadKin:
    FLIP_TRANSLATION = np.array([1., 1., -1.])
    FIXED_HARDWARE_LENGTH = 200.

    def inverse_kinematics(self, pose, return_lengths=False):
        # MDX CAD local points are attached to a moving plane at neutral MID_Z.
        points = MOVING.copy()
        points[:, 2] += MID_Z - float(pose[2])
        if return_lengths:
            return points, np.linalg.norm(points - FIXED, axis=1)
        return points


def test_mdx_park_returns_cad_pose_beyond_operating_limits(mod):
    platform = SimpleNamespace(
        selected_platform="platform_v2",
        kin=CadKin(), mid_z=MID_Z,
        cfg=SimpleNamespace(MUSCLE_MAX_LENGTH=1000., FIXED_HARDWARE_LENGTH=200.),
        bounds=np.array([60., 60., 80., .1, .1, .1]),
    )
    parked, rms = mod.parked_pose_for_platform(platform)
    assert parked[2] == pytest.approx(MID_Z)
    assert parked[2] > platform.bounds[2]   # outside normal +/-80 mm range
    np.testing.assert_allclose(platform.kin.inverse_kinematics(parked)[:, 2], MOVING[:, 2])
    assert rms == pytest.approx(0.76045289123, abs=0.001)


def test_non_mdx_park_preserves_v45_constrained_solver(mod, monkeypatch):
    called = []
    def solver(kin, lengths, seed, bounds):
        called.append((kin, list(lengths), list(seed), list(bounds)))
        return np.array([1., 2., 3., 0., 0., 0.]), 3.0
    monkeypatch.setattr(mod, "solve_pose_from_muscle_lengths", solver)
    platform = SimpleNamespace(
        selected_platform="chair_v2", kin=object(),
        cfg=SimpleNamespace(MUSCLE_MAX_LENGTH=1000.), bounds=np.ones(6),
    )
    pose, rms = mod.parked_pose_for_platform(platform)
    assert pose[2] == 3.0 and rms == 3.0
    assert len(called) == 1 and called[0][1] == [1000.] * 6


class FakeNode:
    def __init__(self, node):
        self.node = node
        self.pos = None
        self.two_sided = False
        self.sz = None
        self.z = None
        self.parent = None
        self.depth_test = None
        self.depth_write = None

    def setPos(self, *xyz):
        self.pos = xyz

    def setTwoSided(self, value):
        self.two_sided = value

    def setSz(self, value):
        self.sz = value

    def setZ(self, value):
        self.z = value

    def setDepthTest(self, value):
        self.depth_test = value

    def setDepthWrite(self, value):
        self.depth_write = value

    def reparentTo(self, node):
        self.parent = node

    def isEmpty(self):
        return False


class FakeMount:
    def __init__(self):
        self.nodes = []
        self.pos = None
        self.look_args = None

    def attachNewNode(self, geom):
        np = FakeNode(geom)
        self.nodes.append(np)
        return np

    def setPos(self, *xyz):
        self.pos = xyz

    def lookAt(self, *args):
        self.look_args = args


def test_pyramid_tips_are_at_true_cad_points_with_17mm_local_foot(mod, monkeypatch):
    pyramids, boxes = [], []
    def pyramid_node(*args):
        pyramids.append(args)
        return ("pyramid", args)
    def box_node(*args):
        boxes.append(args)
        return ("box", args)
    monkeypatch.setattr(mod, "make_pyramid_node", pyramid_node)
    monkeypatch.setattr(mod, "make_box_node", box_node)
    mount = FakeMount()
    deck = mod.mdx_deck_top_z(MOVING)
    obj = SimpleNamespace(
        platform=SimpleNamespace(selected_platform="platform_v2", platform_local_points=MOVING),
        platform_model_mount_np=mount,
        moving_markers=[], mdx_deck_top_z=deck,
    )
    mod.PlatformVisualizer._build_moving_markers(obj)
    assert deck == -25.
    assert len(pyramids) == len(obj.moving_markers) == 6
    assert all(args[1:3] == (17., 17.) for args in pyramids)
    assert [mark.pos for mark in obj.moving_markers] == [tuple(p) for p in MOVING]
    assert all(mark.two_sided for mark in obj.moving_markers)
    assert len(boxes) == 4       # four points need 5 or 6 mm local deck pads
    pad_nodes = [node for node in mount.nodes if node.node[0] == "box"]
    assert sorted(box[3] for box in boxes) == [5., 6., 6., 6.]
    assert all(np.pos[2] == deck for np in pad_nodes)
    # Pad top meets the base of the 17 mm pyramid, i.e. tip Z minus 17.
    pads_by_xy = {(n.pos[0], n.pos[1]): n for n in pad_nodes}
    for x, y, z in MOVING:
        base_z = float(z) - 17.0
        pad = pads_by_xy.get((x, y))
        if pad is None:
            assert base_z == deck
        else:
            assert deck + pad.node[1][3] == base_z


def test_mdx_egg_is_lowered_and_scaled_to_poc_underside(mod, monkeypatch):
    model = FakeNode("model")
    monkeypatch.setattr(mod, "Filename", SimpleNamespace(fromOsSpecific=lambda s: s))
    cfg = {"enabled": True, "model": "assets/mdx_moving_platform.egg", "thickness_mm": 40.0}
    obj = SimpleNamespace(
        moving_platform_cfg=cfg,
        config_path=Path(__file__).resolve().parents[1] / "app_config.json",
        loader=SimpleNamespace(loadModel=lambda _: model),
        platform_model_np=None, platform_model_mount_np=FakeMount(),
        platform=SimpleNamespace(selected_platform="platform_v2", cfg=SimpleNamespace(PLATFORM_UNDERSIDE_Z=-80.)),
        mdx_deck_top_z=-25., use_platform_model=False,
    )
    mod.PlatformVisualizer._build_moving_platform_model(obj)
    assert obj.use_platform_model
    assert model.z == -25.
    assert model.sz == pytest.approx(55 / 40)
    assert model.z - 40 * model.sz == pytest.approx(-80.)


def test_non_mdx_moving_markers_stay_cylindrical(mod, monkeypatch):
    markers = []
    monkeypatch.setattr(mod, "make_cylinder_node", lambda *args: ("cylinder", args))
    root = FakeMount()
    obj = SimpleNamespace(
        platform=SimpleNamespace(selected_platform="chair_v2"),
        dynamic_root=root, moving_markers=markers,
    )
    mod.PlatformVisualizer._build_moving_markers(obj)
    assert len(markers) == 6
    assert all(n.node[0] == "cylinder" for n in markers)
