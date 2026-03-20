"""Track geometry builder — generates 3D road meshes from SVG track data.

Phase 1: outer road (outer boundary capsule + inner boundary capsule,
road surface, and white lane-line strips).

Phase 2: inner curve shortcut roads, inner curve lane lines, and yellow
START line.
"""

import math

from panda3d.core import (
    Geom,
    GeomNode,
    GeomTriangles,
    GeomVertexData,
    GeomVertexFormat,
    GeomVertexWriter,
    LVector4f,
    NodePath,
)

from tools.sim3d.config import (
    GROUND_COLOR,
    LANE_LINE_COLOR,
    LANE_LINE_WIDTH,
    ROAD_COLOR,
    SCALE,
    svg_to_world,
)

# ---------------------------------------------------------------------------
# SVG capsule parameters (from Group 25.svg)
# ---------------------------------------------------------------------------
# Outer boundary capsule
_OUTER_Y_TOP = 312.0
_OUTER_Y_BOT = 3698.0
_OUTER_LEFT_CX = 1704.5
_OUTER_RIGHT_CX = 6491.5
_OUTER_RADIUS = (_OUTER_Y_BOT - _OUTER_Y_TOP) / 2.0  # 1693

# Inner boundary capsule (inner edge of outer road)
_INNER_Y_TOP = 960.0
_INNER_Y_BOT = 3050.5
_INNER_LEFT_CX = 1810.0
_INNER_RIGHT_CX = 6378.5
_INNER_RADIUS = (_INNER_Y_BOT - _INNER_Y_TOP) / 2.0  # ~1045.25

_SEMICIRCLE_SEGMENTS = 64

# ---------------------------------------------------------------------------
# Inner curve bezier data (from SVG — 4 cubic bezier paths)
# ---------------------------------------------------------------------------
# DP1 shortcut (left pair)
_DP1_LEFT = ((2345, 3057.5), (2695.09, 2247.6), (2708.21, 1788.7), (2345, 960))
_DP1_RIGHT = ((3056, 3057.5), (3406.09, 2247.6), (3419.21, 1788.7), (3056, 960))

# DP2 shortcut (right pair)
_DP2_LEFT = ((4873, 3050.5), (5223.09, 2240.6), (5236.21, 1781.7), (4873, 953))
_DP2_RIGHT = ((5584, 3050.5), (5934.09, 2240.6), (5947.21, 1781.7), (5584, 953))

# Yellow START line endpoints (SVG coords)
_START_LINE_P1 = (331.849, 2690.04)
_START_LINE_P2 = (962.849, 2319.04)
_START_LINE_COLOR = (1.0, 0.84, 0.0, 1.0)
_START_LINE_WIDTH_SVG = 44.0  # SVG stroke-width

_BEZIER_SAMPLES = 50


def _cubic_bezier(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    n: int = _BEZIER_SAMPLES,
) -> list[tuple[float, float]]:
    """Sample a cubic bezier curve at *n*+1 evenly-spaced t values.

    Control points are in SVG coordinates; returned points are in world coords.
    """
    points: list[tuple[float, float]] = []
    for i in range(n + 1):
        t = i / n
        u = 1 - t
        x = u**3 * p0[0] + 3 * u**2 * t * p1[0] + 3 * u * t**2 * p2[0] + t**3 * p3[0]
        y = u**3 * p0[1] + 3 * u**2 * t * p1[1] + 3 * u * t**2 * p2[1] + t**3 * p3[1]
        points.append(svg_to_world(x, y))
    return points


def _capsule_points(
    left_cx: float,
    right_cx: float,
    y_top: float,
    y_bot: float,
    radius: float,
    n_arc: int = _SEMICIRCLE_SEGMENTS,
) -> list[tuple[float, float]]:
    """Generate world-coordinate points for a capsule (stadium shape).

    The capsule consists of:
      - top straight from right_cx to left_cx at y_top
      - left semicircle (from top to bottom, going left/CCW in SVG space)
      - bottom straight from left_cx to right_cx at y_bot
      - right semicircle (from bottom to top, going right/CCW in SVG space)

    Returns a closed polyline (first == last) in Panda3D world coords.
    """
    cy = (y_top + y_bot) / 2.0  # vertical center in SVG

    points: list[tuple[float, float]] = []

    # Top straight: right → left
    points.append(svg_to_world(right_cx, y_top))
    points.append(svg_to_world(left_cx, y_top))

    # Left semicircle: top → bottom (SVG angles: -π/2 → π/2, center at left_cx, cy)
    for i in range(1, n_arc):
        angle = -math.pi / 2.0 + math.pi * i / n_arc
        sx = left_cx + radius * math.cos(angle + math.pi)  # semicircle opens left
        sy = cy + radius * math.sin(angle)
        points.append(svg_to_world(sx, sy))

    # Bottom straight: left → right
    points.append(svg_to_world(left_cx, y_bot))
    points.append(svg_to_world(right_cx, y_bot))

    # Right semicircle: bottom → top (semicircle opens right)
    for i in range(1, n_arc):
        angle = math.pi / 2.0 - math.pi * i / n_arc
        sx = right_cx + radius * math.cos(-angle + math.pi * 0)  # opens right
        sy = cy + radius * math.sin(angle)
        points.append(svg_to_world(sx, sy))

    # Close the loop
    points.append(points[0])
    return points


def _make_strip_mesh(
    outer: list[tuple[float, float]],
    inner: list[tuple[float, float]],
    color: tuple[float, float, float, float],
    z: float,
    name: str,
) -> GeomNode:
    """Create a triangle-strip mesh between two polylines at height *z*."""
    n = min(len(outer), len(inner))

    fmt = GeomVertexFormat.get_v3c4()
    vdata = GeomVertexData(name, fmt, Geom.UH_static)
    vdata.setNumRows(n * 2)

    vertex = GeomVertexWriter(vdata, "vertex")
    colour = GeomVertexWriter(vdata, "color")

    for i in range(n):
        ox, oy = outer[i]
        ix, iy = inner[i]
        vertex.addData3(ox, oy, z)
        colour.addData4(*color)
        vertex.addData3(ix, iy, z)
        colour.addData4(*color)

    tris = GeomTriangles(Geom.UH_static)
    for i in range(n - 1):
        o0 = i * 2
        i0 = o0 + 1
        o1 = o0 + 2
        i1 = o0 + 3
        tris.addVertices(o0, i0, o1)
        tris.addVertices(i0, i1, o1)

    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode(name)
    node.addGeom(geom)
    return node


def _offset_polyline(
    points: list[tuple[float, float]], width: float
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """Expand a polyline into two parallel polylines offset by ±width/2.

    Uses per-segment normals (perpendicular to each segment direction).
    """
    left: list[tuple[float, float]] = []
    right: list[tuple[float, float]] = []
    half = width / 2.0

    for i in range(len(points)):
        # Compute averaged normal at vertex i
        if i == 0:
            dx = points[1][0] - points[0][0]
            dy = points[1][1] - points[0][1]
        elif i == len(points) - 1:
            dx = points[-1][0] - points[-2][0]
            dy = points[-1][1] - points[-2][1]
        else:
            dx = points[i + 1][0] - points[i - 1][0]
            dy = points[i + 1][1] - points[i - 1][1]

        length = math.hypot(dx, dy)
        if length < 1e-9:
            nx, ny = 0.0, 0.0
        else:
            nx, ny = -dy / length, dx / length

        px, py = points[i]
        left.append((px + nx * half, py + ny * half))
        right.append((px - nx * half, py - ny * half))

    return left, right


class TrackBuilder:
    """Builds 3D track geometry and attaches it to a Panda3D scene graph."""

    def __init__(self, parent: NodePath) -> None:
        self._parent = parent

    def build(self) -> None:
        """Generate outer road, lane lines, and ground plane."""
        outer_pts = _capsule_points(
            _OUTER_LEFT_CX, _OUTER_RIGHT_CX, _OUTER_Y_TOP, _OUTER_Y_BOT, _OUTER_RADIUS
        )
        inner_pts = _capsule_points(
            _INNER_LEFT_CX, _INNER_RIGHT_CX, _INNER_Y_TOP, _INNER_Y_BOT, _INNER_RADIUS
        )

        # Road surface
        road_node = _make_strip_mesh(outer_pts, inner_pts, ROAD_COLOR, 0.001, "road")
        self._parent.attachNewNode(road_node)

        # Outer lane line
        outer_left, outer_right = _offset_polyline(outer_pts, LANE_LINE_WIDTH)
        ol_node = _make_strip_mesh(outer_left, outer_right, LANE_LINE_COLOR, 0.002, "lane_outer")
        self._parent.attachNewNode(ol_node)

        # Inner lane line
        inner_left, inner_right = _offset_polyline(inner_pts, LANE_LINE_WIDTH)
        il_node = _make_strip_mesh(inner_left, inner_right, LANE_LINE_COLOR, 0.002, "lane_inner")
        self._parent.attachNewNode(il_node)

        # --- Phase 2: inner curve shortcut roads, lane lines, START line ---
        self._build_inner_curves()
        self._build_start_line()

        # Ground plane
        self._build_ground()

    def _build_inner_curves(self) -> None:
        """Build inner curve shortcut roads and their lane lines."""
        curves = {
            "dp1": (_DP1_LEFT, _DP1_RIGHT),
            "dp2": (_DP2_LEFT, _DP2_RIGHT),
        }

        for label, (left_ctrl, right_ctrl) in curves.items():
            left_pts = _cubic_bezier(*left_ctrl)
            right_pts = _cubic_bezier(*right_ctrl)

            # Road surface between left and right edge curves
            road = _make_strip_mesh(
                left_pts, right_pts, ROAD_COLOR, 0.001, f"road_{label}"
            )
            self._parent.attachNewNode(road)

            # Lane line along each edge curve
            for side, pts in [("left", left_pts), ("right", right_pts)]:
                ll, lr = _offset_polyline(pts, LANE_LINE_WIDTH)
                node = _make_strip_mesh(
                    ll, lr, LANE_LINE_COLOR, 0.002, f"lane_{label}_{side}"
                )
                self._parent.attachNewNode(node)

    def _build_start_line(self) -> None:
        """Build the yellow START line across the road."""
        p1 = svg_to_world(*_START_LINE_P1)
        p2 = svg_to_world(*_START_LINE_P2)

        # Direction from p1 to p2
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return
        nx, ny = -dy / length, dx / length  # perpendicular

        half_w = (_START_LINE_WIDTH_SVG * SCALE) / 2.0

        # Build a quad (two triangles) for the line
        outer = [
            (p1[0] + nx * half_w, p1[1] + ny * half_w),
            (p2[0] + nx * half_w, p2[1] + ny * half_w),
        ]
        inner = [
            (p1[0] - nx * half_w, p1[1] - ny * half_w),
            (p2[0] - nx * half_w, p2[1] - ny * half_w),
        ]

        node = _make_strip_mesh(outer, inner, _START_LINE_COLOR, 0.003, "start_line")
        self._parent.attachNewNode(node)

    def _build_ground(self) -> None:
        """Large quad at z=0 as the ground."""
        size = 30.0  # meters — plenty bigger than the track
        fmt = GeomVertexFormat.get_v3c4()
        vdata = GeomVertexData("ground", fmt, Geom.UH_static)
        vdata.setNumRows(4)

        vertex = GeomVertexWriter(vdata, "vertex")
        colour = GeomVertexWriter(vdata, "color")

        for x, y in [(-size, -size), (size, -size), (size, size), (-size, size)]:
            vertex.addData3(x, y, 0.0)
            colour.addData4(*GROUND_COLOR)

        tris = GeomTriangles(Geom.UH_static)
        tris.addVertices(0, 1, 2)
        tris.addVertices(0, 2, 3)

        geom = Geom(vdata)
        geom.addPrimitive(tris)
        node = GeomNode("ground")
        node.addGeom(geom)
        self._parent.attachNewNode(node)
