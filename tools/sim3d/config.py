"""Panda3D 3D simulator constants and coordinate conversion."""

# ---------------------------------------------------------------------------
# SVG / world coordinate mapping
# ---------------------------------------------------------------------------
SVG_VIEWBOX = (8457, 4119)

# The outer road lane is 1.5 m wide in the real world.
# Outer boundary top edge y=312, inner boundary top edge y=960 → gap = 648 SVG units.
# SCALE = 1.5 m / 648 SVG-units ≈ 0.00231 m/SVG-unit.
SCALE: float = 1.5 / 648.0


def svg_to_world(sx: float, sy: float) -> tuple[float, float]:
    """Convert SVG coordinates to Panda3D world coordinates.

    SVG is Y-down; Panda3D uses XY as the ground plane with Z up.
    The track is centered at the origin.
    """
    wx = (sx - SVG_VIEWBOX[0] / 2.0) * SCALE
    wy = -(sy - SVG_VIEWBOX[1] / 2.0) * SCALE
    return wx, wy


# ---------------------------------------------------------------------------
# Road appearance
# ---------------------------------------------------------------------------
ROAD_COLOR = (0.15, 0.15, 0.15, 1.0)
LANE_LINE_COLOR = (1.0, 1.0, 1.0, 1.0)
LANE_LINE_WIDTH: float = 0.03  # meters

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
DASHCAM_RESOLUTION = (320, 240)
DASHCAM_FOV: float = 70.0
DASHCAM_HEIGHT: float = 0.12
DASHCAM_FORWARD_OFFSET: float = 0.10
DASHCAM_PITCH: float = -12.0

CHASE_CAM_DISTANCE: float = 1.5
CHASE_CAM_HEIGHT: float = 0.8

TOPDOWN_HEIGHT: float = 12.0

# ---------------------------------------------------------------------------
# Traffic light (future phases)
# ---------------------------------------------------------------------------
TRAFFIC_LIGHT_CYCLE: dict[str, float] = {
    "red": 10.0,
    "green": 8.0,
    "yellow": 3.0,
}

# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------
TICK_RATE: int = 60

# ---------------------------------------------------------------------------
# Ground
# ---------------------------------------------------------------------------
GROUND_COLOR = (0.1, 0.2, 0.05, 1.0)
