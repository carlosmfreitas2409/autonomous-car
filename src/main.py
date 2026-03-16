"""Entry point for the autonomous vehicle simulator."""

import argparse
import warnings

import pygame

from src.config import SCREEN_WIDTH, SCREEN_HEIGHT, TICK_RATE
from src.input.base import InputController
from src.input.keyboard_controller import KeyboardController
from src.input.ps5_controller import PS5Controller


DARK_GRAY = (30, 30, 30)


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

    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            command = input_controller.read()
            print(
                f"throttle={command.throttle:+.2f}  "
                f"steering={command.steering:+.2f}  "
                f"e_stop={command.emergency_stop}"
            )

            screen.fill(DARK_GRAY)
            pygame.display.flip()
            clock.tick(TICK_RATE)
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
