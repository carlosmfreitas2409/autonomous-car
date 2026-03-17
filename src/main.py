"""Entry point for the autonomous vehicle system."""

import pygame

from src import config
from src.control import Controller, EStopManager, CommandWatchdog
from src.input import PS5Controller
from src.simulation.renderer import Renderer
from src.vehicle.simulated import SimulatedVehicle

def main():
    pygame.init()

    input_controller = PS5Controller()

    if config.MODE == "arduino":
        from src.vehicle.arduino import ArduinoVehicle
        vehicle = ArduinoVehicle(port=config.SERIAL_PORT)
    else:
        vehicle = SimulatedVehicle()
        
        screen = pygame.display.set_mode((config.SCREEN_WIDTH, config.SCREEN_HEIGHT))
        pygame.display.set_caption("Autonomous Vehicle Simulator")
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
                dt = 1.0 / config.TICK_RATE

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

            if config.MODE == "sim":
                state = vehicle.get_state()
                renderer.render(state, throttle, steering, estop.is_active())

            clock.tick(config.TICK_RATE)
    finally:
        vehicle.close()
        pygame.quit()


if __name__ == "__main__":
    main()
