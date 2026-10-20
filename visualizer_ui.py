# visualizer_ui.py ver 6
"""Panda3D screen-space presentation for the standalone platform visualizer.

Only owns display widgets; input mode, simulation state and diagnostics values
remain owned by the controller. No dependency on platform_visualizer.py.
"""
from __future__ import annotations

import socket
import subprocess
import sys

from direct.gui.OnscreenText import OnscreenText
from direct.gui.DirectGui import DirectFrame, DirectButton
from panda3d.core import TextNode

STATUS_TEXT_SCALE = 0.060


def current_listening_ipv4():
    """Read-only list of local IPv4 addresses for the UDP startup chooser.

    Receivers bind to 0.0.0.0, not to any single interface. On Windows,
    getaddrinfo is a small standard-library fallback: it may omit adapters
    not registered against the computer's hostname.
    """
    if sys.platform == "win32":
        try:
            resolved = socket.getaddrinfo(socket.gethostname(), None, family=socket.AF_INET)
        except (OSError, socket.gaierror):
            return "IPv4 addresses unavailable"
        addresses = sorted({item[4][0] for item in resolved
                            if item[4][0] and not item[4][0].startswith("127.")})
        return "\n".join(addresses) if addresses else "No active IPv4 addresses"

    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "up"],
            capture_output=True, text=True, timeout=2, check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return "IPv4 addresses unavailable"
    if result.returncode != 0:
        return "IPv4 addresses unavailable"
    addresses = []
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) < 4 or parts[1] == "lo" or "inet" not in parts:
            continue
        index = parts.index("inet")
        if index + 1 < len(parts):
            address = parts[index + 1].split("/", 1)[0]
            addresses.append(f"{parts[1]}: {address}")
    return "\n".join(addresses) if addresses else "No active IPv4 addresses"


class VisualizerUI:
    """Owns GUI widgets and their appearance, not the 3D scene or app state."""

    def __init__(self, *, aspect2d, top_right, bottom_right,
                 on_mode, on_input, on_toggle_text, on_quit, on_toggle_pose_diff=None,
                 mode, input_mode, diagnostics_visible):
        self._aspect2d = aspect2d
        self._top_right = top_right
        self._bottom_right = bottom_right
        self._on_mode = on_mode
        self._on_input = on_input
        self._on_toggle_text = on_toggle_text
        self._on_quit = on_quit
        self._on_toggle_pose_diff = on_toggle_pose_diff
        self._input_chooser = None
        self._swipe_origin = None
        self._build_status()
        self._build_touch_controls()
        self.refresh_touch_controls(mode, input_mode, diagnostics_visible)

    def _build_status(self):
        self.status = OnscreenText(
            text="", pos=(-1.30, 0.88), scale=STATUS_TEXT_SCALE, align=TextNode.ALeft,
            fg=(0.96, 0.98, 1.0, 1.0), shadow=(0.0, 0.0, 0.0, 1.0),
            shadowOffset=(0.045, 0.045), mayChange=True,
        )
        self.help = OnscreenText(
            text=("A/D or arrows: orbit\n"
                  "W/S or up/down: top/bottom orbit\n"
                  "+/-: camera zoom\n"
                  "O: perspective/ortho\n"
                  "1/2/3: top/front/side views\n"
                  "Ctrl +/-: window size\n"
                  "Ctrl 0: reset window\n"
                  "PgUp/PgDn: view up/down\n\n"
                  "T: test\n"
                  "L: UDP lengths (twin only)\n"
                  "F: Festo pressures\n"
                  "R: hold\n"
                  "Home/C: reset + perspective\n"
                  "N: numbers\n"
                  "H: diagnostic text\n"
                  "Q: quit"),
            pos=(-1.30, 0.52), scale=STATUS_TEXT_SCALE, align=TextNode.ALeft,
            fg=(0.96, 0.98, 1.0, 1.0), shadow=(0.0, 0.0, 0.0, 1.0),
            shadowOffset=(0.045, 0.045), mayChange=False,
        )
        self.help.hide()

    def set_status(self, text):
        self.status.setText(text)

    def show_help(self, visible):
        self.help.show() if visible else self.help.hide()

    def show_input_error(self, mode, exc):
        if self._input_chooser is not None:
            self._input_chooser["text"] = f"Cannot open {mode} UDP port: {exc}\nSelect an input:"

    def hide_input_chooser(self):
        if self._input_chooser is not None:
            self._input_chooser.destroy()
            self._input_chooser = None

    def show_input_chooser(self):
        # This is display-only: network settings are never changed here.
        self._input_chooser = DirectFrame(
            parent=self._aspect2d, frameColor=(0.08, 0.12, 0.19, 0.97),
            frameSize=(-0.92, 0.92, -0.37, 0.37),
            text="Select visualizer input",
            text_scale=0.065, text_pos=(0, 0.23), text_fg=(1, 1, 1, 1),
        )
        for x, mode, label in ((-0.44, "easyip", "EasyIP :995"),
                               (0.44, "twin", "Twin :10020")):
            DirectButton(parent=self._input_chooser, text=label,
                         pos=(x, 0, 0.015), scale=0.065,
                         command=self._on_input, extraArgs=[mode])
        OnscreenText(
            parent=self._input_chooser,
            text=f"UDP listening addresses:\n{current_listening_ipv4()}",
            pos=(0, -0.20), scale=0.055, align=TextNode.ACenter,
            fg=(0.92, 0.96, 1.0, 1.0), mayChange=False,
        )

    def _build_touch_controls(self):
        """Independent screen anchors: orbit/zoom does not move controls."""
        self.touch_buttons = {}
        common = dict(
            text_scale=0.056, text_fg=(0.06, 0.06, 0.06, 1.0),
            frameSize=(-0.140, 0.205, -0.052, 0.052), relief="flat",
            frameColor=(0.83, 0.83, 0.85, 1.0),
            text_pos=(0.020, -0.018),
        )
        for mode, label, y in (
            ("DEMO", "Demo", -0.34),
            ("FESTO", "Pressure", -0.56),
            ("LENGTH", "Lengths", -0.78),
        ):
            self.touch_buttons[mode] = DirectButton(
                parent=self._top_right, text=label, pos=(-0.225, 0, y),
                command=self._on_mode, extraArgs=[mode], **common,
            )
        self.touch_buttons["TEXT"] = DirectButton(
            parent=self._top_right, text="Text on/off", pos=(-0.225, 0, -1.02),
            command=self._on_toggle_text, **common,
        )
        self.touch_buttons["POSE_DIFF"] = DirectButton(
            parent=self._top_right, text="Pose Diff", pos=(-0.225, 0, -1.24),
            command=self._on_toggle_pose_diff, **common,
        )
        self.touch_buttons["QUIT"] = DirectButton(
            parent=self._bottom_right, text="Quit", pos=(-0.225, 0, 0.10),
            command=self._on_quit, **common,
        )

    def refresh_touch_controls(self, mode, input_mode, diagnostics_visible):
        """Refresh from controller-owned state; never maintain a second copy."""
        for button_mode in ("DEMO", "FESTO", "LENGTH"):
            button = self.touch_buttons[button_mode]
            if button_mode == "LENGTH" and input_mode == "easyip":
                button.hide()
            else:
                button.show()
            button["frameColor"] = (
                (0.55, 0.77, 0.91, 1.0) if mode == button_mode
                else (0.83, 0.83, 0.85, 1.0)
            )
        if input_mode != "twin":
            self.touch_buttons["POSE_DIFF"].hide()
        else:
            self.touch_buttons["POSE_DIFF"].show()
        self.touch_buttons["TEXT"]["frameColor"] = (
            (0.83, 0.83, 0.85, 1.0) if diagnostics_visible
            else (0.55, 0.77, 0.91, 1.0)
        )

    def set_pose_diff_enabled(self, enabled):
        self.touch_buttons["POSE_DIFF"]["frameColor"] = (
            (0.55, 0.77, 0.91, 1.0) if enabled else (0.83, 0.83, 0.85, 1.0)
        )

    def begin_scene_swipe(self, mouse_xy, aspect_ratio):
        """Track scene swipes, not presses on the controls or startup dialog."""
        self._swipe_origin = None
        if mouse_xy is None or self._input_chooser is not None:
            return
        # Derive the right-hand exclusion zone from actual button geometry.
        button = self.touch_buttons["DEMO"]
        left_edge = button["pos"][0] + button["frameSize"][0]
        if mouse_xy[0] >= 1.0 + (left_edge - 0.05) / max(float(aspect_ratio), 0.01):
            return
        self._swipe_origin = mouse_xy

    def end_scene_swipe(self, mouse_xy, window_size):
        """Return one arrow direction per swipe; ignore taps and diagonals."""
        start = self._swipe_origin
        self._swipe_origin = None
        if start is None or mouse_xy is None:
            return None
        width, height = window_size
        dx = (mouse_xy[0] - start[0]) * width * 0.5
        dy = (mouse_xy[1] - start[1]) * height * 0.5
        if max(abs(dx), abs(dy)) < 40.0:  # pixels, independent of window size
            return None
        if abs(dx) > 1.2 * abs(dy):
            return "right" if dx > 0 else "left"
        if abs(dy) > 1.2 * abs(dx):
            return "up" if dy > 0 else "down"
        return None
