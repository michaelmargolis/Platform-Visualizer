# test_camera_projection_v53.py ver 1
"""Headless camera checks: orthographic toggle, polar orbit and zoom."""
import math
from types import SimpleNamespace
from test_easyip_visualizer_integration import import_visualizer_with_panda_stubs


def make_camera():
    calls = []
    class Camera:
        def setPos(self, *coords):
            calls.append(("eye", coords))
        def lookAt(self, *args):
            calls.append(("look", args))
        def node(self):
            return SimpleNamespace(setLens=lambda lens: calls.append(("lens", lens)))
    class Lens:
        def setFilmSize(self, w, h):
            calls.append(("film", (w, h)))
    lens = Lens()
    obj = SimpleNamespace(
        camera_azimuth=0., camera_elevation=0., camera_radius=2000.,
        camera_elevation_min=-90., camera_elevation_max=90.,
        camera_target=[0., 0., 0.], camera_view_z_offset=0.,
        camera=Camera(), projection="perspective", perspective_lens=object(),
        orthographic_lens=lens, ortho_default_height=1500., ortho_height=1500.,
        ortho_zoom_step=1.1, camera_radius_step=150.,
        camera_radius_min=1000., camera_radius_max=6000.,
        win=SimpleNamespace(getXSize=lambda:800, getYSize=lambda:480),
    )
    return obj,calls


def test_top_bottom_orbit_reaches_both_poles(monkeypatch):
    mod=import_visualizer_with_panda_stubs(monkeypatch)
    obj,calls=make_camera()
    mod.Vec3 = lambda *args: args
    mod.Point3 = lambda *args: args
    obj._apply_camera = lambda: mod.PlatformVisualizer._apply_camera(obj)
    for angle,z in ((90.,2000.),(-90.,-2000.)):
        obj.camera_elevation=0.
        mod.PlatformVisualizer._camera_vertical(obj,angle)
        assert obj.camera_elevation == angle
        assert abs(calls[-2][1][2]-z)<1e-8
        assert len(calls[-1][1]) == 2  # lookAt(target, up)
    mod.PlatformVisualizer._camera_vertical(obj,-50.)
    assert obj.camera_elevation == -90.


def test_projection_toggle_and_independent_zoom(monkeypatch):
    mod=import_visualizer_with_panda_stubs(monkeypatch)
    obj,calls=make_camera()
    mod.Vec3 = lambda *args: args
    mod.Point3 = lambda *args: args
    obj._apply_camera = lambda: mod.PlatformVisualizer._apply_camera(obj)
    obj.cam = obj.camera
    mod.PlatformVisualizer._toggle_projection(obj)
    assert obj.projection == "orthographic"
    assert calls[-1] == ("film", (2500.,1500.))
    mod.PlatformVisualizer._camera_zoom(obj,-150.)
    assert obj.camera_radius==2000.
    assert obj.ortho_height<1500.
    mod.PlatformVisualizer._toggle_projection(obj)
    assert obj.projection == "perspective"
    mod.PlatformVisualizer._camera_zoom(obj,-150.)
    assert obj.camera_radius == 1850.
