"""Dashcam and debug camera system for the Panda3D 3D simulator.

DashCam renders an offscreen buffer from the car's front-facing perspective
and extracts frames as numpy BGR arrays (matching OpenCV's default format).

DebugCameraManager toggles the main window camera between chase, top-down,
and dashcam-mirror modes via keyboard shortcuts.
"""

from __future__ import annotations

import math

import numpy as np
from panda3d.core import (
    Camera,
    FrameBufferProperties,
    GraphicsOutput,
    GraphicsPipe,
    NodePath,
    PerspectiveLens,
    Texture,
    WindowProperties,
)

from tools.sim3d.config import TOPDOWN_HEIGHT


class DashCam:
    """Offscreen camera that captures BGR frames from the car's perspective."""

    def __init__(
        self,
        base,
        scene_root: NodePath,
        width: int = 320,
        height: int = 240,
        fov: float = 70.0,
    ) -> None:
        self._width = width
        self._height = height
        self._texture = Texture()

        # Offscreen buffer ---------------------------------------------------
        fb_props = FrameBufferProperties()
        fb_props.setRgbaBits(8, 8, 8, 8)
        fb_props.setDepthBits(24)

        win_props = WindowProperties()
        win_props.setSize(width, height)

        flags = GraphicsPipe.BF_refuse_window
        self._buffer = base.graphicsEngine.makeOutput(
            base.pipe,
            "dashcam",
            -100,
            fb_props,
            win_props,
            flags,
            base.win.getGsg(),
            base.win,
        )

        self._buffer.addRenderTexture(
            self._texture, GraphicsOutput.RTM_copy_ram
        )

        # Camera with perspective lens ---------------------------------------
        lens = PerspectiveLens()
        lens.setFov(fov)
        lens.setNearFar(0.01, 50.0)

        cam_node = Camera("dashcam")
        cam_node.setLens(lens)
        self._cam_np: NodePath = scene_root.attachNewNode(cam_node)

        # Display region ------------------------------------------------------
        dr = self._buffer.makeDisplayRegion()
        dr.setCamera(self._cam_np)

    # -----------------------------------------------------------------
    # Public helpers
    # -----------------------------------------------------------------

    def set_transform(
        self,
        x: float,
        y: float,
        z: float,
        h: float,
        p: float,
        r: float,
    ) -> None:
        """Position and orient the dashcam."""
        self._cam_np.setPosHpr(x, y, z, h, p, r)

    def get_frame(self) -> np.ndarray:
        """Extract the rendered frame as a BGR numpy array (H, W, 3)."""
        if not self._texture.hasRamImage():
            return np.zeros((self._height, self._width, 3), dtype=np.uint8)

        ram = self._texture.getRamImageAs("BGR")
        arr = np.frombuffer(bytes(ram), dtype=np.uint8)
        arr = arr.reshape((self._height, self._width, 3))
        arr = np.flipud(arr)  # Panda3D stores bottom-up
        return arr.copy()

    @property
    def camera_np(self) -> NodePath:
        """NodePath of the dashcam camera (useful for mirroring)."""
        return self._cam_np


class DebugCameraManager:
    """Manages the debug-window camera with two switchable modes.

    Modes:
        topdown  — overhead view of the whole track (default)
        dashcam  — mirrors the offscreen dashcam's viewpoint
    """

    def __init__(self, base) -> None:
        self._base = base
        self._mode: str = "topdown"  # 'topdown' | 'dashcam'
        self._dashcam: DashCam | None = None
        # Save the default lens for top-down view
        self._default_lens = base.cam.node().getLens()
        self._dashcam_lens = None

    def set_dashcam(self, dashcam: DashCam) -> None:
        """Register the DashCam so the dashcam-mirror mode can copy its pose and FOV."""
        self._dashcam = dashcam
        self._dashcam_lens = dashcam.camera_np.node().getLens()

    def switch_mode(self, mode: str) -> None:
        """Switch the debug camera to *mode* ('topdown', 'dashcam')."""
        self._mode = mode

    def update(self, car_x: float, car_y: float, heading: float) -> None:
        """Reposition the debug camera for the current frame."""
        if self._mode == "topdown":
            self._base.cam.node().setLens(self._default_lens)
            self._base.camera.setPos(0, 0, TOPDOWN_HEIGHT)
            self._base.camera.lookAt(0, 0, 0)

        elif self._mode == "dashcam" and self._dashcam is not None:
            if self._dashcam_lens is not None:
                self._base.cam.node().setLens(self._dashcam_lens)
            pos = self._dashcam.camera_np.getPos()
            hpr = self._dashcam.camera_np.getHpr()
            self._base.camera.setPosHpr(pos, hpr)
