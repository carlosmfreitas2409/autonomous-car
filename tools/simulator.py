"""PS5 controller testing simulator.

Standalone tool for testing the PS5 DualSense controller without hardware.
Runs a 2D pygame window with bicycle-model physics, a live HUD, and a trail.

Usage:
    python3 tools/simulator.py
"""

import math
import time
from collections import deque

import pygame

from src.input import PS5Controller
from src.control import Controller, EStopManager, CommandWatchdog
from tools.physics import BicyclePhysics, VehicleState

# ---------------------------------------------------------------------------
# Sim-only display constants
# ---------------------------------------------------------------------------
TICK_RATE = 60
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
PIXELS_PER_METER = 200


# ---------------------------------------------------------------------------
# Simulated vehicle
# ---------------------------------------------------------------------------
class SimulatedVehicle:
    def __init__(self):
        self._physics = BicyclePhysics()
        self._last_time: float = time.monotonic()

    def send_command(self, throttle: float, steering: float) -> None:
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        self._physics.update(throttle, steering, dt)

    def get_state(self) -> VehicleState:
        return self._physics.get_state()

    def stop(self) -> None:
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        self._physics.update(0, 0, dt)

    def close(self) -> None:
        pass


# ---------------------------------------------------------------------------
# Renderer
# ---------------------------------------------------------------------------
class Renderer:
    _VEHICLE_LENGTH = 0.2
    _VEHICLE_HALF_WIDTH = 0.06
    _GRID_SPACING = 0.5
    _TRAIL_MAX_POINTS = 500
    _TRAIL_DOT_RADIUS = 2
    _TRAIL_COLOR = (100, 180, 255)

    _COLOR_GREEN = (0, 200, 0)
    _COLOR_RED = (220, 40, 40)
    _COLOR_GRID = (60, 60, 60)
    _COLOR_WHITE = (255, 255, 255)
    _COLOR_HUD_BG = (0, 0, 0, 180)

    def __init__(self, screen: pygame.Surface):
        self._screen = screen
        self._font = self._init_font(18)
        self._clock = pygame.time.Clock()
        self._fps = 0.0
        self._trail: deque[tuple[float, float]] = deque(maxlen=self._TRAIL_MAX_POINTS)

    def render(self, state: VehicleState, throttle: float, steering: float,
               estop_active: bool) -> None:
        self._fps = self._clock.get_fps()
        self._screen.fill((30, 30, 30))
        self._trail.append((state.x, state.y))
        cam_x, cam_y = state.x, state.y
        self._draw_grid(cam_x, cam_y)
        self._draw_trail(cam_x, cam_y)
        self._draw_vehicle(state, cam_x, cam_y, estop_active)
        self._draw_hud(state, throttle, steering, estop_active)
        pygame.display.flip()
        self._clock.tick(0)

    @staticmethod
    def _init_font(size: int):
        try:
            from pygame._freetype import Font as FTFont, init as ft_init
            ft_init()
            return FTFont(None, size)
        except Exception:
            pass
        try:
            return pygame.font.SysFont(None, size)
        except Exception:
            return None

    def _world_to_screen(self, wx: float, wy: float,
                         cam_x: float, cam_y: float) -> tuple[int, int]:
        sx = int((wx - cam_x) * PIXELS_PER_METER + SCREEN_WIDTH / 2)
        sy = int(-(wy - cam_y) * PIXELS_PER_METER + SCREEN_HEIGHT / 2)
        return sx, sy

    def _draw_trail(self, cam_x: float, cam_y: float) -> None:
        n = len(self._trail)
        if n == 0:
            return
        surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        for i, (wx, wy) in enumerate(self._trail):
            alpha = int(40 + 180 * (i / max(n - 1, 1)))
            sx, sy = self._world_to_screen(wx, wy, cam_x, cam_y)
            pygame.draw.circle(surface, (*self._TRAIL_COLOR, alpha), (sx, sy),
                               self._TRAIL_DOT_RADIUS)
        self._screen.blit(surface, (0, 0))

    def _draw_grid(self, cam_x: float, cam_y: float) -> None:
        spacing = self._GRID_SPACING
        half_w = SCREEN_WIDTH / (2 * PIXELS_PER_METER)
        half_h = SCREEN_HEIGHT / (2 * PIXELS_PER_METER)
        x = math.floor((cam_x - half_w) / spacing) * spacing
        while x <= cam_x + half_w:
            sx, _ = self._world_to_screen(x, 0, cam_x, cam_y)
            pygame.draw.line(self._screen, self._COLOR_GRID, (sx, 0), (sx, SCREEN_HEIGHT))
            x += spacing
        y = math.floor((cam_y - half_h) / spacing) * spacing
        while y <= cam_y + half_h:
            _, sy = self._world_to_screen(0, y, cam_x, cam_y)
            pygame.draw.line(self._screen, self._COLOR_GRID, (0, sy), (SCREEN_WIDTH, sy))
            y += spacing

    def _draw_vehicle(self, state: VehicleState, cam_x: float, cam_y: float,
                      estop_active: bool) -> None:
        color = self._COLOR_RED if estop_active else self._COLOR_GREEN
        h = state.heading
        length, half_w = self._VEHICLE_LENGTH, self._VEHICLE_HALF_WIDTH
        nose = (length * 0.6, 0.0)
        left_rear = (-length * 0.4, half_w)
        right_rear = (-length * 0.4, -half_w)

        def rt(lx: float, ly: float) -> tuple[float, float]:
            return (state.x + lx * math.cos(h) - ly * math.sin(h),
                    state.y + lx * math.sin(h) + ly * math.cos(h))

        points = [self._world_to_screen(*rt(*p), cam_x, cam_y)
                  for p in (nose, left_rear, right_rear)]
        pygame.draw.polygon(self._screen, color, points)

    def _draw_hud(self, state: VehicleState, throttle: float, steering: float,
                  estop_active: bool) -> None:
        lines = [
            (f"Speed: {state.speed:.2f} m/s", self._COLOR_WHITE),
            (f"Heading: {math.degrees(state.heading):.0f}\u00b0", self._COLOR_WHITE),
            (f"Throttle: {throttle:.0%}", self._COLOR_WHITE),
            (f"Steering: {steering:+.2f}", self._COLOR_WHITE),
            (f"E-STOP: {'ON' if estop_active else 'OFF'}",
             self._COLOR_RED if estop_active else self._COLOR_WHITE),
            (f"FPS: {self._fps:.0f}", self._COLOR_WHITE),
        ]
        padding, line_height = 8, 22
        hud_w = 220
        hud_h = padding * 2 + line_height * len(lines)
        hud = pygame.Surface((hud_w, hud_h), pygame.SRCALPHA)
        hud.fill(self._COLOR_HUD_BG)
        self._screen.blit(hud, (8, 8))
        if self._font is None:
            return
        for i, (text, color) in enumerate(lines):
            result = self._font.render(text, color)
            surface = result[0] if isinstance(result, tuple) else result
            self._screen.blit(surface, (8 + padding, 8 + padding + i * line_height))


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    pygame.init()

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("PS5 Controller Simulator")

    input_controller = PS5Controller()
    vehicle = SimulatedVehicle()
    renderer = Renderer(screen)
    estop = EStopManager()
    watchdog = CommandWatchdog()
    controller = Controller(estop=estop)
    clock = pygame.time.Clock()

    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            dt = clock.get_time() / 1000.0
            if dt <= 0:
                dt = 1.0 / TICK_RATE

            throttle, steering, emergency_stop = input_controller.read()

            watchdog.feed()
            if watchdog.is_expired():
                estop.activate()

            if not emergency_stop and estop.is_active():
                estop.reset()

            throttle, steering = controller.process(throttle, steering, emergency_stop, dt)

            if estop.is_active():
                vehicle.stop()
            else:
                vehicle.send_command(throttle, steering)

            state = vehicle.get_state()
            renderer.render(state, throttle, steering, estop.is_active())

            clock.tick(TICK_RATE)
    finally:
        vehicle.close()
        pygame.quit()


if __name__ == "__main__":
    main()
