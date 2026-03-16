"""Entry point for the autonomous vehicle simulator."""

import argparse

import pygame

from src.config import SCREEN_WIDTH, SCREEN_HEIGHT, TICK_RATE


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


def main():
    """Main entry point."""
    args = parse_args()

    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Autonomous Vehicle Simulator")
    clock = pygame.time.Clock()

    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            screen.fill(DARK_GRAY)
            pygame.display.flip()
            clock.tick(TICK_RATE)
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
