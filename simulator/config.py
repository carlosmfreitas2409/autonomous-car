"""Panda3D 3D simulator constants and coordinate conversion."""

# ---------------------------------------------------------------------------
# SVG / world coordinate mapping
# ---------------------------------------------------------------------------
SVG_VIEWBOX = (8457, 4119)

# Lane width scaled to match car proportions.
# Outer boundary top edge y=312, inner boundary top edge y=960 → gap = 648 SVG units.
SCALE: float = 0.75 / 648.0


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
# 3D simulator physics overrides (scaled to fit the track)
# The real car physics (tools/physics.py) are for a small RC car.
# The 3D track has 1.5 m lanes, so scale everything up to feel like a
# real car on a real road.
# ---------------------------------------------------------------------------
SIM3D_WHEEL_BASE: float = 0.5         # meters (was 0.25)
SIM3D_WHEEL_RADIUS: float = 0.10      # meters (was 0.033)
SIM3D_MAX_RPM: float = 100.0           # max speed ~0.8 m/s
SIM3D_MAX_STEERING_ANGLE: float = 0.4
SIM3D_FRICTION: float = 0.4
SIM3D_MOTOR_TIME_CONSTANT: float = 0.15

# Car model dimensions (~1/2 of 1.5 m lane width)
CAR_LENGTH: float = 0.85  # meters
CAR_WIDTH: float = 0.45   # meters
CAR_HEIGHT: float = 0.18  # meters

# ---------------------------------------------------------------------------
# Camera — tripod style: mounted above/behind the car, looking forward
# with a slight downward angle to see road + signs ahead.
# ---------------------------------------------------------------------------
DASHCAM_RESOLUTION = (1080, 600)
DASHCAM_FOV: float = 110.0
DASHCAM_HEIGHT: float = 0.60         # tripod well above car roof
DASHCAM_FORWARD_OFFSET: float = 0.6  # in front of car center (tripod style)
DASHCAM_PITCH: float = -50.0          # angled down to see lane markings and signs

TOPDOWN_HEIGHT: float = 10.0

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
