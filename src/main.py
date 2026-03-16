"""Entry point for the autonomous vehicle simulator."""

import argparse
import warnings

import pygame

from src.config import SCREEN_WIDTH, SCREEN_HEIGHT, TICK_RATE
from src.control.controller import Controller
from src.input.base import InputController
from src.input.keyboard_controller import KeyboardController
from src.input.ps5_controller import PS5Controller
from src.models.commands import MotorCommand
from src.simulation.renderer import Renderer
from src.vehicle.simulated import SimulatedVehicle


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Autonomous Vehicle Simulator")
    parser.add_argument(
        "--mode",
        choices=["sim", "arduino"],
        default="sim",
        help="Run mode: sim (simulation) or arduino (real hardware). Default: sim",
    )
    parser.add_argument(
        "--input",
        choices=["ps5", "keyboard"],
        default="keyboard",
        help="Input method: ps5 (DualSense controller) or keyboard. Default: keyboard",
    )
    return parser.parse_args()


def create_input_controller(input_type: str) -> InputController:
    """Create the appropriate input controller, with fallback."""
    if input_type == "ps5":
        controller = PS5Controller()
        if controller.is_connected():
            return controller
        warnings.warn(
            "PS5 controller not available — falling back to keyboard.",
            stacklevel=2,
        )
    return KeyboardController()


def main():
    """Main entry point."""
    args = parse_args()

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Autonomous Vehicle Simulator")
    clock = pygame.time.Clock()

    input_controller = create_input_controller(args.input)
    vehicle = SimulatedVehicle()
    controller = Controller()
    renderer = Renderer(screen)

    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            dt = clock.get_time() / 1000.0  # milliseconds to seconds
            if dt <= 0:
                dt = 1.0 / TICK_RATE  # sensible default on first frame

            # 1. Read input
            command = input_controller.read()

            # 2. Process through controller
            motor_cmd = controller.process(command, dt)

            # 3. Handle e-stop
            if command.emergency_stop:
                motor_cmd = MotorCommand(left=0, right=0)
                vehicle.stop()
            else:
                # 4. Send to vehicle
                vehicle.send_command(motor_cmd)

            # 5. Get state
            state = vehicle.get_state()

            # 6. Render
            renderer.render(state, motor_cmd, command.emergency_stop)

            clock.tick(TICK_RATE)
    finally:
        vehicle.close()
        pygame.quit()


if __name__ == "__main__":
    main()
