"""Panda3D 3D simulated vehicle — drop-in replacement for ArduinoVehicle.

Does NOT call ``base.run()``. The caller drives the loop and each
``send_command()`` advances physics, updates the scene, and manually steps
Panda3D for one frame.
"""

import math
import time

import numpy as np

from panda3d.core import (
    Geom,
    GeomNode,
    GeomTriangles,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
    LVector4f,
    NodePath,
    WindowProperties,
)
from direct.showbase.ShowBase import ShowBase

from tools.physics import BicyclePhysics, VehicleState
from tools.sim3d.camera import DashCam, DebugCameraManager
from tools.sim3d.config import (
    DASHCAM_FORWARD_OFFSET,
    DASHCAM_FOV,
    DASHCAM_HEIGHT,
    DASHCAM_PITCH,
    DASHCAM_RESOLUTION,
    svg_to_world,
)
from tools.sim3d.track import TrackBuilder

# Start position from SVG START marker (~647, 2505)
_START_SVG = (647.0, 2505.0)
_START_HEADING = math.radians(50)  # facing along bottom-left portion of oval


def _make_car_box(
    length: float = 0.25,
    width: float = 0.12,
    height: float = 0.08,
) -> GeomNode:
    """Build a simple coloured box as the car model."""
    hl, hw, hh = length / 2.0, width / 2.0, height / 2.0

    # 8 corners: bottom then top (in local frame: X forward, Y left, Z up)
    corners = [
        (-hl, -hw, 0.0),
        (hl, -hw, 0.0),
        (hl, hw, 0.0),
        (-hl, hw, 0.0),
        (-hl, -hw, height),
        (hl, -hw, height),
        (hl, hw, height),
        (-hl, hw, height),
    ]

    # 12 triangles (2 per face)
    faces = [
        (0, 1, 2, 3),  # bottom
        (4, 7, 6, 5),  # top
        (0, 4, 5, 1),  # front-right
        (2, 6, 7, 3),  # back-left
        (1, 5, 6, 2),  # right
        (0, 3, 7, 4),  # left
    ]

    fmt = GeomVertexFormat.get_v3c4()
    vdata = GeomVertexData("car", fmt, Geom.UH_static)
    vdata.setNumRows(len(corners))

    vertex = GeomVertexWriter(vdata, "vertex")
    colour = GeomVertexWriter(vdata, "color")

    car_color = (0.1, 0.7, 0.2, 1.0)  # green
    for c in corners:
        vertex.addData3(*c)
        colour.addData4(*car_color)

    tris = GeomTriangles(Geom.UH_static)
    for a, b, c, d in faces:
        tris.addVertices(a, b, c)
        tris.addVertices(a, c, d)

    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode("car")
    node.addGeom(geom)
    return node


class SimulatedVehicle3D:
    """3D simulated vehicle backed by BicyclePhysics and Panda3D."""

    def __init__(self) -> None:
        # Physics
        self._physics = BicyclePhysics()
        sx, sy = svg_to_world(*_START_SVG)
        self._physics.x = sx
        self._physics.y = sy
        self._physics.heading = _START_HEADING
        self._last_time: float = time.monotonic()

        # Panda3D
        self._base = ShowBase(windowType="onscreen")
        self._base.disableMouse()
        self._base.setBackgroundColor(0.4, 0.6, 0.9, 1.0)  # sky blue

        # Window title
        wp = WindowProperties()
        wp.setTitle("3D Simulator")
        self._base.win.requestProperties(wp)

        # Build track
        track_root = self._base.render.attachNewNode("track")
        TrackBuilder(track_root).build()

        # Car model
        self._car_np: NodePath = self._base.render.attachNewNode(_make_car_box())
        self._update_car_transform()

        # Dashcam (offscreen render for CV pipeline)
        self._dashcam = DashCam(
            self._base,
            self._base.render,
            width=DASHCAM_RESOLUTION[0],
            height=DASHCAM_RESOLUTION[1],
            fov=DASHCAM_FOV,
        )

        # Debug camera manager (chase / top-down / dashcam-mirror)
        self._debug_cam = DebugCameraManager(self._base)
        self._debug_cam.set_dashcam(self._dashcam)

        # Keyboard shortcuts for camera modes
        self._base.accept("c", self._debug_cam.switch_mode, ["chase"])
        self._base.accept("t", self._debug_cam.switch_mode, ["topdown"])
        self._base.accept("v", self._debug_cam.switch_mode, ["dashcam"])

        # Initial camera update
        self._update_cameras()

    # ------------------------------------------------------------------
    # Public interface (matches SimulatedVehicle / ArduinoVehicle)
    # ------------------------------------------------------------------

    def send_command(self, throttle: float, steering: float) -> None:
        """Advance physics, update scene, step Panda3D."""
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now

        self._physics.update(throttle, steering, dt)
        self._update_car_transform()
        self._update_cameras()
        self._step_panda()

    def get_state(self) -> VehicleState:
        return self._physics.get_state()

    def stop(self) -> None:
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        self._physics.update(0.0, 0.0, dt)
        self._update_car_transform()
        self._update_cameras()
        self._step_panda()

    def close(self) -> None:
        try:
            self._base.destroy()
        except Exception:
            pass

    def get_frame(self) -> np.ndarray:
        """Capture a BGR frame from the front-facing dashcam (H, W, 3)."""
        return self._dashcam.get_frame()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _update_car_transform(self) -> None:
        self._car_np.setPos(self._physics.x, self._physics.y, 0.0)
        panda_h = math.degrees(self._physics.heading) - 90.0
        self._car_np.setH(panda_h)

    def _update_cameras(self) -> None:
        """Position the dashcam relative to the car and update the debug camera."""
        h = self._physics.heading

        # Dashcam: mounted at front of car, looking forward and slightly down
        cam_x = self._physics.x + DASHCAM_FORWARD_OFFSET * math.cos(h)
        cam_y = self._physics.y + DASHCAM_FORWARD_OFFSET * math.sin(h)
        cam_z = DASHCAM_HEIGHT
        panda_h = math.degrees(h) - 90.0
        panda_p = DASHCAM_PITCH  # negative = look down
        self._dashcam.set_transform(cam_x, cam_y, cam_z, panda_h, panda_p, 0.0)

        # Debug window camera
        self._debug_cam.update(self._physics.x, self._physics.y, h)

    def _step_panda(self) -> None:
        self._base.taskMgr.step()
        self._base.graphicsEngine.renderFrame()
