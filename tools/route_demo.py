"""Visual demo of the autonomous route planner.

Animated diagram showing the car following the optimal path for each
pickup/delivery combo. Simple dot moving along the track — no physics.

Usage:
    python3 tools/route_demo.py                  # default: pickup=1, delivery=3
    python3 tools/route_demo.py 4 1              # pickup=4, delivery=1
    python3 tools/route_demo.py --all            # cycle through all 12 combos

Controls:
    Q/ESC  — quit
"""

import math
import sys

import pygame
from pygame._freetype import Font as FTFont
from pygame._freetype import init as ft_init

from src.autonomous.mission import MissionPlanner
from src.autonomous.route_table import Decision, SignAction

# --- Display ---
WIDTH, HEIGHT = 900, 650
FPS = 60

# --- Colors ---
BG = (25, 25, 30)
TRACK_COLOR = (80, 85, 95)
INNER_CURVE_DIM = (55, 55, 65)
CAR_COLOR = (80, 220, 120)
CAR_STOPPED = (240, 70, 70)
SIGN_DEFAULT = (100, 100, 100)
SIGN_PICKUP = (240, 140, 40)
SIGN_DELIVERY = (60, 160, 240)
SIGN_DONE = (80, 200, 80)
START_COLOR = (255, 255, 255)
HUD_BG = (0, 0, 0, 200)
WHITE = (255, 255, 255)
YELLOW = (240, 200, 40)
RED = (220, 50, 50)
GREEN = (60, 200, 80)
GRAY = (140, 140, 140)

# --- Track layout (stadium shape) ---
# Flat top and bottom, semicircular ends on left and right.
CX, CY = 450, 310
STRAIGHT_HALF = 200     # half-length of straight sections
END_RADIUS = 180        # radius of semicircular ends
LEFT_CENTER = CX - STRAIGHT_HALF   # 250
RIGHT_CENTER = CX + STRAIGHT_HALF  # 650
TOP_Y = CY - END_RADIUS            # 130
BOT_Y = CY + END_RADIUS            # 490


def _semicircle(cx, cy, radius, start_deg, end_deg, n):
    """Points along a semicircular arc (degrees, standard math convention)."""
    pts = []
    for i in range(n):
        t = i / (n - 1) if n > 1 else 0
        a = math.radians(start_deg + (end_deg - start_deg) * t)
        pts.append((cx + radius * math.cos(a), cy - radius * math.sin(a)))
    return pts


def _lerp(p1, p2, n):
    """n evenly spaced points from p1 to p2."""
    pts = []
    for i in range(n):
        t = i / (n - 1) if n > 1 else 0
        pts.append((p1[0] + (p2[0] - p1[0]) * t, p1[1] + (p2[1] - p1[1]) * t))
    return pts


def _bezier_quad(p0, p1, p2, n):
    """Quadratic bezier curve: p0=start, p1=control, p2=end."""
    pts = []
    for i in range(n):
        t = i / (n - 1) if n > 1 else 0
        x = (1-t)**2 * p0[0] + 2*(1-t)*t * p1[0] + t**2 * p2[0]
        y = (1-t)**2 * p0[1] + 2*(1-t)*t * p1[1] + t**2 * p2[1]
        pts.append((x, y))
    return pts


# --- Key positions on the stadium ---
# Clockwise from START (bottom-left)

# START: on the left semicircle, partway up the left side
_START_ANGLE = math.radians(-130)
START_POS = (LEFT_CENTER + END_RADIUS * math.cos(_START_ANGLE),
             CY - END_RADIUS * math.sin(_START_ANGLE))

# DP1: top-left, where left inner curve branches off
DP1_POS = (LEFT_CENTER + 40, TOP_Y)

# DP2: top-right, where right inner curve branches off
DP2_POS = (RIGHT_CENTER - 40, TOP_Y)

# Inner curve exit points on the bottom (where curves meet the track)
# Symmetric: same offset from their respective semicircle centers
_CURVE_OFFSET = 80
LEFT_EXIT = (LEFT_CENTER + _CURVE_OFFSET, BOT_Y)    # (330, 490)
RIGHT_EXIT = (RIGHT_CENTER - _CURVE_OFFSET, BOT_Y)  # (570, 490)

# Sign positions: signs are slightly PAST the curve exits
# (further left on the bottom, since clockwise = right-to-left on the bottom)
# Sign 2 is placed ON the right semicircle arc (~55°)
_SIGN2_ANGLE = math.radians(55)
SIGN_POS = {
    1: (CX, TOP_Y - 5),                                          # top center
    2: (RIGHT_CENTER + END_RADIUS * math.cos(_SIGN2_ANGLE),      # on right arc
        CY - END_RADIUS * math.sin(_SIGN2_ANGLE)),
    3: (RIGHT_EXIT[0] - 50, BOT_Y + 5),                          # past right exit
    4: (LEFT_EXIT[0] - 50, BOT_Y + 5),                           # past left exit
}


# --- Full outer track (stadium outline for drawing) ---
def _build_outer_track():
    """Full stadium outline for background drawing."""
    top = _lerp((LEFT_CENTER, TOP_Y), (RIGHT_CENTER, TOP_Y), 20)
    right = _semicircle(RIGHT_CENTER, CY, END_RADIUS, 90, -90, 30)
    bottom = _lerp((RIGHT_CENTER, BOT_Y), (LEFT_CENTER, BOT_Y), 20)
    left = _semicircle(LEFT_CENTER, CY, END_RADIUS, -90, -270, 30)
    return top + right[1:] + bottom[1:] + left[1:]


OUTER_TRACK = _build_outer_track()


# --- Inner curves (D-shaped bezier arcs, mirrored) ---
# Both curves use the same offset from their respective centers
_CTRL_OFFSET = 160  # how far the control point bulges toward center

def _build_inner_curve_1():
    """Left inner curve: DP1 → left exit point."""
    ctrl = (LEFT_CENTER + _CTRL_OFFSET, CY)
    return _bezier_quad(DP1_POS, ctrl, LEFT_EXIT, 25)

def _build_inner_curve_2():
    """Right inner curve: DP2 → right exit point (bulges right, follows traffic)."""
    ctrl = (RIGHT_CENTER + 80, CY)
    return _bezier_quad(DP2_POS, ctrl, RIGHT_EXIT, 25)


INNER_CURVE_1 = _build_inner_curve_1()
INNER_CURVE_2 = _build_inner_curve_2()


# --- Path segments (clockwise) ---

def _seg_a():
    """START → DP1: up left semicircle and along top-left."""
    arc = _semicircle(LEFT_CENTER, CY, END_RADIUS, -130, -270, 25)
    return arc + _lerp(arc[-1], DP1_POS, 6)[1:]


def _seg_b():
    """DP1 → Sign1 → DP2: across the top (outer path)."""
    return _lerp(DP1_POS, SIGN_POS[1], 10) + _lerp(SIGN_POS[1], DP2_POS, 10)[1:]


def _seg_c():
    """DP2 → Sign2 → Sign3: outer right side (long way around)."""
    p1 = _lerp(DP2_POS, (RIGHT_CENTER, TOP_Y), 6)
    arc = _semicircle(RIGHT_CENTER, CY, END_RADIUS, 90, -90, 25)
    p3 = _lerp((RIGHT_CENTER, BOT_Y), SIGN_POS[3], 10)
    return p1 + arc[1:] + p3[1:]


def _seg_d():
    """DP2 → Sign3: inner curve 2 shortcut then to sign."""
    return list(INNER_CURVE_2) + _lerp(RIGHT_EXIT, SIGN_POS[3], 6)[1:]


def _seg_e1():
    """Sign3 → Sign4: across the bottom."""
    return _lerp(SIGN_POS[3], SIGN_POS[4], 15)


def _seg_e2():
    """Sign4 → START: along bottom then up left semicircle to start."""
    to_bottom = _lerp(SIGN_POS[4], (LEFT_CENTER, BOT_Y), 8)
    arc = _semicircle(LEFT_CENTER, CY, END_RADIUS, -90, -130, 8)
    return to_bottom + arc[1:]


def _seg_f():
    """DP1 → Sign4: inner curve 1 shortcut then to sign."""
    return list(INNER_CURVE_1) + _lerp(LEFT_EXIT, SIGN_POS[4], 6)[1:]


# Pre-build segments
SEG_A = _seg_a()
SEG_B = _seg_b()
SEG_C = _seg_c()
SEG_D = _seg_d()
SEG_E1 = _seg_e1()
SEG_E2 = _seg_e2()
SEG_F = _seg_f()


def build_demo_path(dp1: Decision, dp2: Decision | None) -> list[tuple[float, float]]:
    """Build the full pixel-coordinate path for a given route."""
    path = list(SEG_A)
    if dp1 == Decision.INNER:
        path += SEG_F[1:] + SEG_E2[1:]
    elif dp2 == Decision.OUTER:
        path += SEG_B[1:] + SEG_C[1:] + SEG_E1[1:] + SEG_E2[1:]
    else:
        path += SEG_B[1:] + SEG_D[1:] + SEG_E1[1:] + SEG_E2[1:]
    return path


def _find_sign_indices(path, sign_positions):
    """For each sign, find the nearest point index on the path."""
    result = {}
    for sign_num, (sx, sy) in sign_positions.items():
        best_idx = 0
        best_dist = float("inf")
        for i, (px, py) in enumerate(path):
            d = math.hypot(px - sx, py - sy)
            if d < best_dist:
                best_dist = d
                best_idx = i
        if best_dist < 50:
            result[best_idx] = sign_num
    return result


# --- Drawing ---

def draw_track(screen):
    """Draw the full stadium track outline and inner curves."""
    # Outer track
    pts = [(int(x), int(y)) for x, y in OUTER_TRACK]
    pygame.draw.lines(screen, TRACK_COLOR, True, pts, 3)

    # Inner curves (dimmed)
    for curve in (INNER_CURVE_1, INNER_CURVE_2):
        pts = [(int(x), int(y)) for x, y in curve]
        pygame.draw.lines(screen, INNER_CURVE_DIM, False, pts, 2)



def draw_signs(screen, font, mission: MissionPlanner):
    """Draw sign markers with mission-aware coloring."""
    m = mission.mission
    for num, (sx, sy) in SIGN_POS.items():
        sx, sy = int(sx), int(sy)

        if num == m.pickup_sign and m.pickup_done:
            color, label = SIGN_DONE, f"{num} (picked up)"
        elif num == m.delivery_sign and m.delivery_done:
            color, label = SIGN_DONE, f"{num} (delivered)"
        elif num == m.pickup_sign:
            color, label = SIGN_PICKUP, f"{num} PICKUP"
        elif num == m.delivery_sign:
            color, label = SIGN_DELIVERY, f"{num} DELIVER"
        else:
            color, label = SIGN_DEFAULT, str(num)

        # Triangle marker
        pts = [(sx, sy - 18), (sx - 14, sy + 8), (sx + 14, sy + 8)]
        pygame.draw.polygon(screen, color, pts)
        pygame.draw.polygon(screen, WHITE, pts, 1)

        # Number inside
        surf, _ = font.render(str(num), WHITE)
        screen.blit(surf, (sx - surf.get_width() // 2, sy - 12))

        # Label below
        surf, _ = font.render(label, color)
        screen.blit(surf, (sx - surf.get_width() // 2, sy + 14))


def draw_landmarks(screen, font):
    """Draw START and DP markers."""
    sx, sy = int(START_POS[0]), int(START_POS[1])
    pygame.draw.circle(screen, START_COLOR, (sx, sy), 10, 2)
    surf, _ = font.render("START", START_COLOR)
    screen.blit(surf, (sx - surf.get_width() // 2, sy + 14))

    for dp_id, pos in [(1, DP1_POS), (2, DP2_POS)]:
        px, py = int(pos[0]), int(pos[1])
        pygame.draw.circle(screen, GRAY, (px, py), 7, 1)
        surf, _ = font.render(f"DP{dp_id}", GRAY)
        screen.blit(surf, (px + 10, py - 8))


def draw_car(screen, pos, stopped):
    """Draw the car as a filled circle."""
    color = CAR_STOPPED if stopped else CAR_COLOR
    pygame.draw.circle(screen, color, (int(pos[0]), int(pos[1])), 8)
    pygame.draw.circle(screen, WHITE, (int(pos[0]), int(pos[1])), 8, 1)



def draw_hud(screen, font, mission: MissionPlanner, state: str,
             stop_timer: float, current_lap: int):
    m = mission.mission
    lines = [
        (f"Mission: Sign {m.pickup_sign} -> Sign {m.delivery_sign}", WHITE),
        (f"Lap: {current_lap + 1} / {mission.total_laps}", WHITE),
        ("", WHITE),
    ]

    for lap_idx in range(mission.total_laps):
        plan = m.route[lap_idx]
        dp1_str = "INNER" if plan.dp1 == Decision.INNER else "OUTER"
        dp2_str = plan.dp2.name if plan.dp2 else "---"
        marker = " <--" if lap_idx == current_lap else ""
        lines.append((f"  Lap {lap_idx+1}: DP1={dp1_str}  DP2={dp2_str}{marker}", GRAY))

    lines.append(("", WHITE))

    pickup_color = GREEN if m.pickup_done else SIGN_PICKUP
    lines.append((f"Pickup #{m.pickup_sign}: {'DONE' if m.pickup_done else 'pending'}", pickup_color))

    delivery_color = GREEN if m.delivery_done else SIGN_DELIVERY
    lines.append((f"Deliver #{m.delivery_sign}: {'DONE' if m.delivery_done else 'pending'}", delivery_color))

    lines.append(("", WHITE))

    if state == "STOPPED":
        state_display = f"STOPPED ({stop_timer:.1f}/4.0s)"
        state_color = RED
    elif state == "COMPLETE":
        state_display = "COMPLETE"
        state_color = GREEN
    else:
        state_display = state
        state_color = WHITE
    lines.append((f"State: {state_display}", state_color))

    padding = 10
    line_h = 22
    hud_w = 340
    hud_h = padding * 2 + line_h * len(lines)

    hud = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
    hud.fill(HUD_BG)
    screen.blit(hud, (8, 8))

    for i, (text, color) in enumerate(lines):
        if not text:
            continue
        surf, _ = font.render(text, color)
        screen.blit(surf, (8 + padding, 8 + padding + i * line_h))


# --- Main loop ---

STOP_DURATION = 3.0
CAR_SPEED = 20.0  # points per second (slow and visible)


def run_demo(pickup: int, delivery: int) -> bool:
    """Run the demo for one pickup/delivery combo. Returns False to quit."""
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(f"Route Demo: Pickup {pickup} -> Deliver {delivery}")
    clock = pygame.time.Clock()
    ft_init()
    font = FTFont(None, 18)

    mission = MissionPlanner(pickup, delivery)
    current_lap = 0

    plan = mission.mission.route[current_lap]
    path = build_demo_path(plan.dp1, plan.dp2)
    sign_indices = _find_sign_indices(path, SIGN_POS)

    path_pos = 0.0
    state = "NAVIGATING"
    stop_timer = 0.0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return False
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    pygame.quit()
                    return False

        dt = clock.get_time() / 1000.0
        if dt <= 0:
            dt = 1.0 / FPS

        if state != "COMPLETE":
            if state == "STOPPED":
                stop_timer += dt
                if stop_timer >= STOP_DURATION:
                    stop_timer = 0.0
                    if mission.is_complete:
                        state = "COMPLETE"
                    else:
                        state = "NAVIGATING"
                        # Advance lap if at end of path
                        if int(path_pos) >= len(path) - 1:
                            if current_lap + 1 < mission.total_laps:
                                current_lap += 1
                                mission.advance_lap()
                                plan = mission.mission.route[current_lap]
                                path = build_demo_path(plan.dp1, plan.dp2)
                                sign_indices = _find_sign_indices(path, SIGN_POS)
                                path_pos = 0.0

            elif state == "NAVIGATING":
                path_pos += CAR_SPEED * dt

                if path_pos >= len(path) - 1:
                    path_pos = len(path) - 1
                    if current_lap + 1 < mission.total_laps and not mission.is_complete:
                        current_lap += 1
                        mission.advance_lap()
                        plan = mission.mission.route[current_lap]
                        path = build_demo_path(plan.dp1, plan.dp2)
                        sign_indices = _find_sign_indices(path, SIGN_POS)
                        path_pos = 0.0
                    elif mission.is_complete:
                        state = "COMPLETE"

                # Check sign proximity
                idx = int(path_pos)
                for sign_idx, sign_num in sign_indices.items():
                    if abs(idx - sign_idx) < 2:
                        action = mission.record_sign_visit(sign_num)
                        if action != SignAction.NONE:
                            state = "STOPPED"
                            stop_timer = 0.0
                            path_pos = float(sign_idx)
                            break

        # Current car position
        idx = int(path_pos)
        frac = path_pos - idx
        if idx < len(path) - 1:
            x1, y1 = path[idx]
            x2, y2 = path[idx + 1]
            car_pos = (x1 + (x2 - x1) * frac, y1 + (y2 - y1) * frac)
        else:
            car_pos = path[-1]

        # --- Draw ---
        screen.fill(BG)
        draw_track(screen)
        draw_landmarks(screen, font)
        draw_signs(screen, font, mission)
        draw_car(screen, car_pos, stopped=(state in ("STOPPED", "COMPLETE")))
        draw_hud(screen, font, mission, state, stop_timer, current_lap)

        pygame.display.flip()
        clock.tick(FPS)


def main():
    if "--all" in sys.argv:
        combos = [(p, d) for p in range(1, 5) for d in range(1, 5) if p != d]
        for pickup, delivery in combos:
            if not run_demo(pickup, delivery):
                break
    else:
        pickup = int(sys.argv[1]) if len(sys.argv) > 1 else 1
        delivery = int(sys.argv[2]) if len(sys.argv) > 2 else 3
        run_demo(pickup, delivery)


if __name__ == "__main__":
    main()
