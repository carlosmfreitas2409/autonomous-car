"""3D simulator entry point — PS5 controller drives a car in Panda3D.

Usage:
    python3 tools/run_sim3d.py
"""

import time

import pygame

from src.input import PS5Controller
from src.control import Controller, EStopManager, CommandWatchdog
from tools.sim3d.config import TICK_RATE
from tools.sim3d.vehicle import SimulatedVehicle3D


def main() -> None:
    # pygame is needed for PS5Controller (joystick subsystem).
    # Only init the subsystems we need — no display window.
    pygame.init()

    vehicle: SimulatedVehicle3D | None = None
    try:
        input_controller = PS5Controller()
        estop = EStopManager()
        watchdog = CommandWatchdog()
        controller = Controller(estop=estop)
        vehicle = SimulatedVehicle3D()

        dt = 1.0 / TICK_RATE
        running = True

        while running:
            loop_start = time.monotonic()

            # Pump pygame events so joystick data stays fresh
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            throttle, steering, emergency_stop = input_controller.read()

            watchdog.feed()
            if watchdog.is_expired():
                estop.activate()

            if not emergency_stop and estop.is_active():
                estop.reset()

            throttle, steering = controller.process(
                throttle, steering, emergency_stop, dt
            )

            if estop.is_active():
                vehicle.stop()
            else:
                vehicle.send_command(throttle, steering)

            # Frame pacing
            elapsed = time.monotonic() - loop_start
            sleep_time = (1.0 / TICK_RATE) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        if vehicle is not None:
            vehicle.close()
        pygame.quit()


if __name__ == "__main__":
    main()
