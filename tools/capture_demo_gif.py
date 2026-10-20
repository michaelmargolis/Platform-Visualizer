# capture_demo_gif.py ver 2
"""Capture one Panda3D demo cycle to docs/demo-<platform>.gif."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
import tempfile

from PIL import Image

# Edit this value to experiment with lower GIF sampling rates.
FPS = 12
WIDTH = 640
HEIGHT = 360
ROOT = Path(__file__).resolve().parents[1]


def frame_times(period, fps):
    """Sample exactly one loop; do not duplicate its neutral endpoint."""
    if not 1 <= fps <= 50:
        raise ValueError("GIF fps must be between 1 and 50")
    count = round(period * fps)
    if count < 1 or abs(count / fps - period) > 1e-8:
        raise ValueError("Demo period must be a multiple of 1/fps")
    return [frame / fps for frame in range(count)]


def frame_durations_ms(count, fps):
    """Quantize to GIF's 10-ms units without changing the total cycle time."""
    boundaries = [round(frame * 100 / fps) for frame in range(count + 1)]
    return [10 * (boundaries[i + 1] - boundaries[i]) for i in range(count)]


def demo_heading(stage):
    """Keep just the first status line in captured frames."""
    return f"PLATFORM TWIN DEMO: {stage}"


def default_output_path(platform_name):
    """Use the visualizer's selected platform for a filesystem-safe GIF name."""
    safe_name = re.sub(r"[^A-Za-z0-9_-]+", "_", str(platform_name)).strip("_")
    if not safe_name:
        raise ValueError("Selected platform cannot be used as a GIF filename")
    return ROOT / "docs" / f"demo-{safe_name}.gif"


def capture(args):
    # Import Panda3D only during capture so sampling/timing can be unit-tested
    # without a graphics installation.
    from panda3d.core import Filename, WindowProperties
    sys.path.insert(0, str(ROOT))
    from platform_visualizer import PlatformVisualizer, resolve_poc_root

    config_path = Path(args.config).expanduser().resolve()
    cfg = json.loads(config_path.read_text(encoding="utf-8"))
    root = resolve_poc_root(args.poc_root, cfg.get("poc_root"), ROOT)
    app = None
    try:
        # demo_only skips all UDP sockets and the input-choice popup. The
        # visualizer runs the exact same motion/rendering code as normal Demo.
        app = PlatformVisualizer(
            root, config_path, fullscreen_override=False,
            selected_platform=cfg.get("selected_platform"), demo_only=True,
        )
        app.taskMgr.remove("platform-update")
        # Keep the onscreen buttons and the status heading. Hide only unrelated
        # informational text (help and attachment numbers).
        app.ui.show_help(False)
        app.labels_root.hide()

        props = WindowProperties()
        props.setSize(args.width, args.height)
        app.win.requestProperties(props)
        # Let Panda3D establish the graphics context and resize before capture.
        for _ in range(3):
            app.taskMgr.step()
        size = (app.win.getXSize(), app.win.getYSize())
        if size != (args.width, args.height):
            raise RuntimeError(f"Could not set capture window size: {size}")

        times = frame_times(app.demo.period, args.fps)
        durations = frame_durations_ms(len(times), args.fps)
        frames = []
        with tempfile.TemporaryDirectory(prefix="mdx_demo_capture_") as tmp:
            tmp = Path(tmp)
            for i, t in enumerate(times):
                app.pose, app.demo_stage = app.demo.pose(t)
                app._update_dynamic_scene()
                app.ui.set_status(demo_heading(app.demo_stage))
                app.taskMgr.step()  # Draw the manually selected simulation pose
                frame_path = tmp / f"frame_{i:04d}.png"
                # Read Panda3D's own framebuffer, not a screenshot of the OS.
                if not app.win.saveScreenshot(Filename.fromOsSpecific(str(frame_path))):
                    raise RuntimeError(f"Panda3D failed to save {frame_path}")
                with Image.open(frame_path) as png:
                    frames.append(png.convert("RGB"))
                if (i + 1) % 24 == 0 or i == len(times) - 1:
                    print(f"Captured {i+1}/{len(times)} frames", flush=True)

        # Use one palette for all frames to limit palette flicker in the GIF.
        palette = frames[0].quantize(colors=128)
        gif_frames = [frame.quantize(palette=palette, dither=Image.Dither.NONE)
                      for frame in frames]
        output = (Path(args.output).expanduser().resolve() if args.output is not None
                  else default_output_path(app.platform.selected_platform))
        output.parent.mkdir(parents=True, exist_ok=True)
        gif_frames[0].save(
            output, format="GIF", save_all=True, append_images=gif_frames[1:],
            duration=durations, loop=0, optimize=True, disposal=1,
        )
        print(f"Saved {output} ({len(times)} frames, {app.demo.period:g}s, {args.fps} fps)")
    finally:
        if app is not None:
            app.destroy()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--fps", type=int, default=FPS, help=f"GIF frames/sec (default: {FPS})")
    ap.add_argument("--width", type=int, default=WIDTH)
    ap.add_argument("--height", type=int, default=HEIGHT)
    ap.add_argument("--config", default=str(ROOT / "app_config.json"))
    ap.add_argument("--poc-root", help="Optional external POC source")
    ap.add_argument("--output", help="GIF file (default: docs/demo-<selected platform>.gif)")
    args = ap.parse_args(argv)
    if args.width <= 0 or args.height <= 0:
        ap.error("Capture dimensions must be positive")
    if not 1 <= args.fps <= 50:
        ap.error("--fps must be 1 to 50")
    capture(args)


if __name__ == "__main__":
    main()
