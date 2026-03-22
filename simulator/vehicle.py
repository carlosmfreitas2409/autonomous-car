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

import tools.physics as _phys_mod
from tools.physics import BicyclePhysics, VehicleState
from tools.sim3d.camera import DashCam, DebugCameraManager
from tools.sim3d.config import (
    CAR_HEIGHT,
    CAR_LENGTH,
    CAR_WIDTH,
    DASHCAM_FORWARD_OFFSET,
    DASHCAM_FOV,
    DASHCAM_HEIGHT,
    DASHCAM_PITCH,
    DASHCAM_RESOLUTION,
    SIM3D_FRICTION,
    SIM3D_MAX_RPM,
    SIM3D_MAX_STEERING_ANGLE,
    SIM3D_MOTOR_TIME_CONSTANT,
    SIM3D_WHEEL_BASE,
    SIM3D_WHEEL_RADIUS,
    svg_to_world,
)
from tools.sim3d.props import PropsBuilder
from tools.sim3d.track import TrackBuilder

# Start position: midpoint of the START line on the left semicircle at 250°
# Outer semicircle center (1704.5, 2005), radius 1693
# Inner semicircle center (1810, 2005.25), radius 1045.25
_START_ANGLE_RAD = math.radians(170)  # slightly behind the START line (clockwise)
_MID_CX = (1704.5 + 1810.0) / 2.0
_MID_CY = (2005.0 + 2005.25) / 2.0
_MID_R = (1693.0 + 1045.25) / 2.0
_START_SVG = (
    _MID_CX + _MID_R * math.cos(_START_ANGLE_RAD),
    _MID_CY + _MID_R * math.sin(_START_ANGLE_RAD),
)
# Heading: tangent to semicircle at 250° going counter-clockwise.
# In SVG (Y-down), tangent at angle θ going CCW is (θ - 90°).
# In world coords (Y flipped), heading = -(SVG_angle - 90°) = 90° - SVG_angle.
# Clockwise motion: at SVG angle 170°, tangent in world coords ≈ 100°.
_START_HEADING = math.radians(100)


def _add_box(
    vdata_writer: tuple,
    tris: GeomTriangles,
    corners: list[tuple[float, float, float]],
    faces: list[tuple[int, int, int, int]],
    color: tuple[float, float, float, float],
    idx_offset: int,
) -> int:
    """Add a colored box to the vertex data and triangle list."""
    vertex, colour = vdata_writer
    for c in corners:
        vertex.addData3(*c)
        colour.addData4(*color)
    for a, b, c, d in faces:
        tris.addVertices(idx_offset + a, idx_offset + b, idx_offset + c)
        tris.addVertices(idx_offset + a, idx_offset + c, idx_offset + d)
    return idx_offset + len(corners)


def _make_car_model(
    length: float = CAR_LENGTH,
    width: float = CAR_WIDTH,
    height: float = CAR_HEIGHT,
) -> GeomNode:
    """Build a car-shaped model: body + cabin + hood slope + wheels."""
    hl, hw = length / 2.0, width / 2.0
    body_h = height * 0.45       # lower body height
    cabin_h = height * 0.55      # cabin height (on top of body)
    cabin_front = length * 0.05  # cabin starts just ahead of center
    cabin_back = -hl + length * 0.15  # cabin ends near rear

    # Slight taper at the front for a hood shape
    hood_taper = width * 0.1

    body_color = (0.15, 0.45, 0.85, 1.0)   # blue body
    cabin_color = (0.08, 0.08, 0.12, 1.0)  # dark cabin/windows
    wheel_color = (0.15, 0.15, 0.15, 1.0)  # dark wheels

    box_faces = [
        (0, 1, 2, 3),  # bottom
        (4, 7, 6, 5),  # top
        (0, 4, 5, 1),  # side 1
        (2, 6, 7, 3),  # side 2
        (1, 5, 6, 2),  # front/end
        (0, 3, 7, 4),  # back/end
    ]

    # --- Body (full length, tapered front) ---
    body_corners = [
        (-hl, -hw, 0.0),
        (hl, -(hw - hood_taper), 0.0),
        (hl, (hw - hood_taper), 0.0),
        (-hl, hw, 0.0),
        (-hl, -hw, body_h),
        (hl, -(hw - hood_taper), body_h),
        (hl, (hw - hood_taper), body_h),
        (-hl, hw, body_h),
    ]

    # --- Cabin (rear portion, sits on body) ---
    cab_hw = hw * 0.85
    cabin_corners = [
        (cabin_back, -cab_hw, body_h),
        (cabin_front, -cab_hw, body_h),
        (cabin_front, cab_hw, body_h),
        (cabin_back, cab_hw, body_h),
        (cabin_back, -cab_hw, body_h + cabin_h),
        (cabin_front, -cab_hw, body_h + cabin_h),
        (cabin_front, cab_hw, body_h + cabin_h),
        (cabin_back, cab_hw, body_h + cabin_h),
    ]

    # --- Wheels (4 small boxes at corners) ---
    wl, ww, wh = length * 0.08, width * 0.06, height * 0.2
    wheel_positions = [
        (hl * 0.7, -hw - ww * 0.5, 0.0),   # front-right
        (hl * 0.7, hw + ww * 0.5, 0.0),     # front-left
        (-hl * 0.7, -hw - ww * 0.5, 0.0),   # rear-right
        (-hl * 0.7, hw + ww * 0.5, 0.0),    # rear-left
    ]

    total_verts = 8 + 8 + 4 * 8  # body + cabin + 4 wheels
    fmt = GeomVertexFormat.get_v3c4()
    vdata = GeomVertexData("car", fmt, Geom.UH_static)
    vdata.setNumRows(total_verts)

    vertex = GeomVertexWriter(vdata, "vertex")
    colour = GeomVertexWriter(vdata, "color")
    tris = GeomTriangles(Geom.UH_static)

    idx = 0
    idx = _add_box((vertex, colour), tris, body_corners, box_faces, body_color, idx)
    idx = _add_box((vertex, colour), tris, cabin_corners, box_faces, cabin_color, idx)

    for wx, wy, wz in wheel_positions:
        w_corners = [
            (wx - wl, wy - ww, wz),
            (wx + wl, wy - ww, wz),
            (wx + wl, wy + ww, wz),
            (wx - wl, wy + ww, wz),
            (wx - wl, wy - ww, wz + wh),
            (wx + wl, wy - ww, wz + wh),
            (wx + wl, wy + ww, wz + wh),
            (wx - wl, wy + ww, wz + wh),
        ]
        idx = _add_box((vertex, colour), tris, w_corners, box_faces, wheel_color, idx)

    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode("car")
    node.addGeom(geom)
    return node


class SimulatedVehicle3D:
    """3D simulated vehicle backed by BicyclePhysics and Panda3D."""

    def __init__(self) -> None:
        # Override physics constants for the 3D track scale.
        # This only affects BicyclePhysics instances created after this point.
        _phys_mod.WHEEL_BASE = SIM3D_WHEEL_BASE
        _phys_mod.WHEEL_RADIUS = SIM3D_WHEEL_RADIUS
        _phys_mod.MAX_RPM = SIM3D_MAX_RPM
        _phys_mod.MAX_STEERING_ANGLE = SIM3D_MAX_STEERING_ANGLE
        _phys_mod.FRICTION = SIM3D_FRICTION
        _phys_mod.MOTOR_TIME_CONSTANT = SIM3D_MOTOR_TIME_CONSTANT

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

        # Window size and title
        wp = WindowProperties()
        wp.setTitle("3D Simulator")
        wp.setSize(1080, 600)
        self._base.win.requestProperties(wp)

        # Build track
        track_root = self._base.render.attachNewNode("track")
        TrackBuilder(track_root).build()

        # Props (signs + traffic light)
        self._props = PropsBuilder(self._base.render)
        self._props.build()
        self._traffic_light = self._props.traffic_light

        # Car model — geometry is built with +X forward, but Panda3D heading
        # treats +Y as forward, so rotate the model 90° inside its parent node.
        self._car_np: NodePath = self._base.render.attachNewNode("car_root")
        car_geom = self._car_np.attachNewNode(_make_car_model())
        car_geom.setH(90.0)  # rotate model so +X forward becomes +Y forward
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
        self._traffic_light.update(dt)
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
        self._traffic_light.update(dt)
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
