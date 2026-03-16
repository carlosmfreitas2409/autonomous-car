"""CSV logging for simulation data."""

import csv
import io

from src.models.commands import MotorCommand
from src.models.state import VehicleState


class SimLogger:
    """Logs simulation state and commands to a CSV file.

    Columns: timestamp, x, y, heading, speed, left_motor, right_motor,
             cmd_left, cmd_right
    """

    _HEADER = [
        "timestamp", "x", "y", "heading", "speed",
        "left_motor", "right_motor", "cmd_left", "cmd_right",
    ]

    def __init__(self, filename: str = "sim_log.csv"):
        self._file: io.TextIOWrapper = open(filename, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self._HEADER)

    def log(self, state: VehicleState, cmd: MotorCommand) -> None:
        """Write a single row of simulation data.

        Args:
            state: Current vehicle state.
            cmd: Current motor command.
        """
        self._writer.writerow([
            state.timestamp,
            state.x,
            state.y,
            state.heading,
            state.speed,
            state.left_motor,
            state.right_motor,
            cmd.left,
            cmd.right,
        ])

    def close(self) -> None:
        """Flush and close the CSV file."""
        self._file.flush()
        self._file.close()
