"""Entry point for the autonomous vehicle simulator."""

import pygame

from src import config
from src.control.controller import Controller
from src.input.ps5_controller import PS5Controller
from src.safety.emergency_stop import EStopManager
from src.safety.watchdog import CommandWatchdog
from src.simulation.renderer import Renderer
from src.vehicle.simulated import SimulatedVehicle


def main():
    """Main entry point."""
    pygame.init()
    screen = pygame.display.set_mode((config.SCREEN_WIDTH, config.SCREEN_HEIGHT))
    pygame.display.set_caption("Autonomous Vehicle Simulator")
    clock = pygame.time.Clock()

    input_controller = PS5Controller()

    if config.MODE == "arduino":
        from src.vehicle.arduino import ArduinoVehicle
        vehicle = ArduinoVehicle(port=config.SERIAL_PORT)
    else:
        vehicle = SimulatedVehicle()

    estop = EStopManager()
    watchdog = CommandWatchdog()
    controller = Controller(estop=estop)
    renderer = Renderer(screen)

    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            dt = clock.get_time() / 1000.0
            if dt <= 0:
                dt = 1.0 / config.TICK_RATE

            command = input_controller.read()

            watchdog.feed()
            if watchdog.is_expired():
                estop.activate()

            if not command.emergency_stop and estop.is_active():
                estop.reset()

            motor_cmd = controller.process(command, dt)

            if estop.is_active():
                vehicle.stop()
            else:
                vehicle.send_command(motor_cmd)

            state = vehicle.get_state()

            renderer.render(state, motor_cmd, estop.is_active())
            clock.tick(config.TICK_RATE)
    finally:
        vehicle.close()
        pygame.quit()


if __name__ == "__main__":
    main()
