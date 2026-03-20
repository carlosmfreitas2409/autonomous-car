"""Sign geometry builders and TrafficLight for the 3D simulator."""

import math

from panda3d.core import (
    Geom,
    GeomNode,
    GeomTriangles,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
    NodePath,
    TextNode,
)

from tools.sim3d.config import TRAFFIC_LIGHT_CYCLE, svg_to_world

SIGN_HEIGHT: float = 0.20  # meters (sign face)
POLE_HEIGHT: float = 0.30  # meters (total pole height, sign sits on top)
POLE_WIDTH: float = 0.01  # meters


# ---------------------------------------------------------------------------
# Helper: polygon geometry
# ---------------------------------------------------------------------------

def _make_flat_polygon(
    name: str,
    vertices_2d: list[tuple[float, float]],
    color: tuple[float, float, float, float],
) -> GeomNode:
    """Build a double-sided flat polygon from 2D vertices in the XZ plane.

    Vertices are given as (x, z) offsets from the node origin.
    Two sets of triangles are created (front and back) so the sign is
    visible from both sides.
    """
    n = len(vertices_2d)
    fmt = GeomVertexFormat.get_v3c4()
    vdata = GeomVertexData(name, fmt, Geom.UH_static)
    # Front vertices + back vertices (duplicated with slight Y offset)
    vdata.setNumRows(n * 2)

    vertex = GeomVertexWriter(vdata, "vertex")
    colour = GeomVertexWriter(vdata, "color")

    # Front face (y = 0.001)
    for x, z in vertices_2d:
        vertex.addData3(x, 0.001, z)
        colour.addData4(*color)
    # Back face (y = -0.001)
    for x, z in vertices_2d:
        vertex.addData3(x, -0.001, z)
        colour.addData4(*color)

    tris = GeomTriangles(Geom.UH_static)

    # Fan triangulation (works for convex polygons)
    for i in range(1, n - 1):
        # Front face (CCW)
        tris.addVertices(0, i, i + 1)
        # Back face (CW from front = CCW from back)
        tris.addVertices(n, n + i + 1, n + i)

    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode(name)
    node.addGeom(geom)
    return node


def _make_pole(parent: NodePath) -> None:
    """Add a thin gray pole from ground to POLE_HEIGHT."""
    hw = POLE_WIDTH / 2.0
    # A thin box (4 faces visible)
    corners = [
        (-hw, -hw, 0.0),
        (hw, -hw, 0.0),
        (hw, hw, 0.0),
        (-hw, hw, 0.0),
        (-hw, -hw, POLE_HEIGHT),
        (hw, -hw, POLE_HEIGHT),
        (hw, hw, POLE_HEIGHT),
        (-hw, hw, POLE_HEIGHT),
    ]
    faces = [
        (0, 1, 5, 4),  # front
        (1, 2, 6, 5),  # right
        (2, 3, 7, 6),  # back
        (3, 0, 4, 7),  # left
    ]
    pole_color = (0.5, 0.5, 0.5, 1.0)

    fmt = GeomVertexFormat.get_v3c4()
    vdata = GeomVertexData("pole", fmt, Geom.UH_static)
    vdata.setNumRows(len(corners))

    vertex = GeomVertexWriter(vdata, "vertex")
    colour = GeomVertexWriter(vdata, "color")
    for c in corners:
        vertex.addData3(*c)
        colour.addData4(*pole_color)

    tris = GeomTriangles(Geom.UH_static)
    for a, b, c, d in faces:
        tris.addVertices(a, b, c)
        tris.addVertices(a, c, d)

    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode("pole")
    node.addGeom(geom)
    pole_np = parent.attachNewNode(node)
    return pole_np


# ---------------------------------------------------------------------------
# SignBuilder
# ---------------------------------------------------------------------------

class SignBuilder:
    """Factory methods for road-sign geometry."""

    @staticmethod
    def triangle_sign(
        number: int,
        parent: NodePath,
        svg_x: float,
        svg_y: float,
        facing_angle: float,
    ) -> NodePath:
        """Blue equilateral-triangle sign with a white number, on a pole.

        *facing_angle*: radians, direction the sign faces (toward the
        approaching car).
        """
        wx, wy = svg_to_world(svg_x, svg_y)
        root = parent.attachNewNode(f"sign_{number}")
        root.setPos(wx, wy, 0.0)

        _make_pole(root)

        size = SIGN_HEIGHT
        half = size / 2.0
        h = size * math.sqrt(3) / 2.0  # equilateral triangle height

        # Triangle centred horizontally, base at POLE_HEIGHT, apex up
        verts = [
            (-half, POLE_HEIGHT),          # bottom-left
            (half, POLE_HEIGHT),           # bottom-right
            (0.0, POLE_HEIGHT + h),        # top
        ]
        face_node = _make_flat_polygon(
            f"tri_face_{number}", verts, (0.2, 0.3, 0.9, 1.0),
        )
        face_np = root.attachNewNode(face_node)
        face_np.setH(math.degrees(facing_angle) - 90)

        # Number text
        tn = TextNode(f"sign_text_{number}")
        tn.setText(str(number))
        tn.setAlign(TextNode.ACenter)
        tn.setTextColor(1, 1, 1, 1)
        text_np = root.attachNewNode(tn)
        text_np.setPos(0, 0, POLE_HEIGHT + h * 0.38)
        text_np.setScale(size * 0.35)
        text_np.setH(math.degrees(facing_angle) - 90)

        return root

    @staticmethod
    def circle_sign(
        parent: NodePath,
        svg_x: float,
        svg_y: float,
        facing_angle: float,
    ) -> NodePath:
        """Red circle direction sign on a pole."""
        wx, wy = svg_to_world(svg_x, svg_y)
        root = parent.attachNewNode("circle_sign")
        root.setPos(wx, wy, 0.0)

        _make_pole(root)

        # 16-sided polygon approximating a circle
        n = 16
        radius = SIGN_HEIGHT / 2.0
        center_z = POLE_HEIGHT + radius
        verts = [
            (radius * math.cos(2 * math.pi * i / n),
             center_z + radius * math.sin(2 * math.pi * i / n))
            for i in range(n)
        ]
        face_node = _make_flat_polygon(
            "circle_face", verts, (0.9, 0.15, 0.15, 1.0),
        )
        face_np = root.attachNewNode(face_node)
        face_np.setH(math.degrees(facing_angle) - 90)

        return root

    @staticmethod
    def octagon_sign(
        text: str,
        parent: NodePath,
        svg_x: float,
        svg_y: float,
        facing_angle: float,
    ) -> NodePath:
        """Red octagon sign (PARE / stop) on a pole."""
        wx, wy = svg_to_world(svg_x, svg_y)
        root = parent.attachNewNode("octagon_sign")
        root.setPos(wx, wy, 0.0)

        _make_pole(root)

        radius = SIGN_HEIGHT / 2.0
        center_z = POLE_HEIGHT + radius
        verts = [
            (radius * math.cos(2 * math.pi * i / 8 + math.pi / 8),
             center_z + radius * math.sin(2 * math.pi * i / 8 + math.pi / 8))
            for i in range(8)
        ]
        face_node = _make_flat_polygon(
            "octagon_face", verts, (0.85, 0.1, 0.1, 1.0),
        )
        face_np = root.attachNewNode(face_node)
        face_np.setH(math.degrees(facing_angle) - 90)

        # Text
        tn = TextNode("octagon_text")
        tn.setText(text)
        tn.setAlign(TextNode.ACenter)
        tn.setTextColor(1, 1, 1, 1)
        text_np = root.attachNewNode(tn)
        text_np.setPos(0, 0, center_z - radius * 0.2)
        text_np.setScale(SIGN_HEIGHT * 0.3)
        text_np.setH(math.degrees(facing_angle) - 90)

        return root

    @staticmethod
    def diamond_sign(
        parent: NodePath,
        svg_x: float,
        svg_y: float,
        facing_angle: float,
    ) -> NodePath:
        """Yellow diamond warning sign on a pole."""
        wx, wy = svg_to_world(svg_x, svg_y)
        root = parent.attachNewNode("diamond_sign")
        root.setPos(wx, wy, 0.0)

        _make_pole(root)

        half = SIGN_HEIGHT / 2.0
        center_z = POLE_HEIGHT + half
        verts = [
            (0.0, center_z + half),   # top
            (half, center_z),          # right
            (0.0, center_z - half),    # bottom
            (-half, center_z),         # left
        ]
        face_node = _make_flat_polygon(
            "diamond_face", verts, (1.0, 0.84, 0.0, 1.0),
        )
        face_np = root.attachNewNode(face_node)
        face_np.setH(math.degrees(facing_angle) - 90)

        return root


# ---------------------------------------------------------------------------
# TrafficLight
# ---------------------------------------------------------------------------

class TrafficLight:
    """Horizontal traffic light with red/yellow/green cycle."""

    def __init__(
        self,
        parent: NodePath,
        svg_x: float,
        svg_y: float,
        facing_angle: float,
    ) -> None:
        self._state: str = "red"
        self._timer: float = 0.0
        self._cycle = TRAFFIC_LIGHT_CYCLE
        self._lights: dict[str, NodePath] = {}

        wx, wy = svg_to_world(svg_x, svg_y)
        self._root = parent.attachNewNode("traffic_light")
        self._root.setPos(wx, wy, 0.0)

        self._build(facing_angle)
        self._update_visuals()

    # -- public --------------------------------------------------------------

    def update(self, dt: float) -> None:
        """Advance the traffic-light timer."""
        self._timer += dt
        if self._timer >= self._cycle[self._state]:
            self._timer = 0.0
            _next = {"red": "green", "green": "yellow", "yellow": "red"}
            self._state = _next[self._state]
            self._update_visuals()

    @property
    def state(self) -> str:
        return self._state

    # -- internal ------------------------------------------------------------

    def _build(self, facing_angle: float) -> None:
        """Build housing box + three coloured discs on a pole."""
        _make_pole(self._root)

        panda_h = math.degrees(facing_angle) - 90

        # Housing: a dark box behind the three lights
        box_w = SIGN_HEIGHT * 1.6  # width (spans 3 lights)
        box_h = SIGN_HEIGHT * 0.55
        half_w = box_w / 2.0
        half_h = box_h / 2.0
        center_z = POLE_HEIGHT + half_h

        box_verts = [
            (-half_w, center_z - half_h),
            (half_w, center_z - half_h),
            (half_w, center_z + half_h),
            (-half_w, center_z + half_h),
        ]
        housing_node = _make_flat_polygon(
            "tl_housing", box_verts, (0.15, 0.15, 0.15, 1.0),
        )
        housing_np = self._root.attachNewNode(housing_node)
        housing_np.setH(panda_h)

        # Three light discs (12-sided circles) arranged horizontally
        disc_r = SIGN_HEIGHT * 0.18
        n_sides = 12
        offsets = {"red": -box_w / 3.0, "yellow": 0.0, "green": box_w / 3.0}

        for colour_name, x_off in offsets.items():
            verts = [
                (x_off + disc_r * math.cos(2 * math.pi * i / n_sides),
                 center_z + disc_r * math.sin(2 * math.pi * i / n_sides))
                for i in range(n_sides)
            ]
            # Placeholder colour — _update_visuals will set the real one
            disc_node = _make_flat_polygon(
                f"tl_{colour_name}", verts, (1, 1, 1, 1),
            )
            disc_np = self._root.attachNewNode(disc_node)
            disc_np.setH(panda_h)
            # Push disc slightly in front of housing
            disc_np.setY(disc_np.getY() + 0.002)
            self._lights[colour_name] = disc_np

    def _update_visuals(self) -> None:
        """Active light is bright; inactive lights are dimmed."""
        colors = {
            "red": (0.9, 0.1, 0.1, 1.0),
            "yellow": (0.9, 0.8, 0.1, 1.0),
            "green": (0.1, 0.9, 0.1, 1.0),
        }
        dim = 0.2
        for name, np in self._lights.items():
            if name == self._state:
                np.setColor(*colors[name])
            else:
                r, g, b, a = colors[name]
                np.setColor(r * dim, g * dim, b * dim, a)


# ---------------------------------------------------------------------------
# PropsBuilder — places all signs and the traffic light
# ---------------------------------------------------------------------------

class PropsBuilder:
    """Orchestrates placing all signs and the traffic light on the track."""

    def __init__(self, parent: NodePath) -> None:
        self._parent = parent
        self.traffic_light: TrafficLight | None = None

    def build(self) -> None:
        """Place all signs and the traffic light."""
        # Numbered signs (blue triangles)
        # Signs 1 & 2: top straight, face pi (car approaches from left)
        # Signs 3 & 4: bottom straight, face 0 (car approaches from right)
        SignBuilder.triangle_sign(1, self._parent, 2700, 800, math.pi)
        SignBuilder.triangle_sign(2, self._parent, 5300, 800, math.pi)
        SignBuilder.triangle_sign(3, self._parent, 3900, 3200, 0)
        SignBuilder.triangle_sign(4, self._parent, 2300, 3200, 0)

        # Direction signs at DP1 and DP2 entrances
        SignBuilder.circle_sign(self._parent, 2350, 1100, math.pi)
        SignBuilder.circle_sign(self._parent, 4900, 1100, math.pi)

        # PARE stop sign on right side
        SignBuilder.octagon_sign("PARE", self._parent, 7500, 2000, math.pi / 2)

        # Curve warning sign
        SignBuilder.diamond_sign(self._parent, 3200, 3900, 0)

        # Traffic light
        self.traffic_light = TrafficLight(
            self._parent, 4500, 3900, 0,
        )
