import serial
import logging
import time

from dataclasses import dataclass
from typing import Optional

from cargo import CargoReading
from lights import LightsOutput

logger = logging.getLogger(__name__)

SONAR_NO_ECHO = 0xFF

# The cargo node reports at 10 Hz: no `C` line for this long means it is not
# connected (or never was), and the mission runs without the compartment.
_CARGO_STALE_S = 1.0


@dataclass
class SonarReading:
  state: str
  nearest_cm: Optional[int]
  # Per-sensor distances, left to right, None where nothing echoed. Empty when
  # the master only sent the nearest value.
  distances: tuple[Optional[int], ...] = ()


class _SerialPort:
  def __init__(self, baudrate: int):
    self._baudrate = baudrate
    self._serial: Optional[serial.Serial] = None

  def open(self, port: str):
    try:
      self.close()
      new_serial = serial.Serial(port, self._baudrate, timeout=0.01)
      logger.info(f"Opened serial port {port} at {self._baudrate} baud, waiting for Arduino to boot")
      time.sleep(2.5)
      new_serial.reset_input_buffer()
      self._serial = new_serial
      logger.info("Arduino ready")
    except serial.SerialException as e:
      logger.error(f"Failed to open serial port {port}: {e}")
      raise

  def write(self, message: str):
    if self._serial is None:
      return
    try:
      self._serial.write(f"{message}\n".encode("ascii"))
    except serial.SerialException as e:
      logger.error(f"Serial write failed: {e}")

  def read(self) -> Optional[str]:
    if self._serial is None:
      return None
    try:
      if self._serial.in_waiting:
        raw = self._serial.readline()
        if raw:
          return raw.decode("ascii", errors="replace").strip()
    except serial.SerialException as e:
      logger.error(f"Serial read failed: {e}")
    return None

  def read_lines(self, limit: int = 40) -> list[str]:
    lines = []
    for _ in range(limit):
      line = self.read()
      if not line:
        break
      lines.append(line)
    return lines

  def close(self):
    if self._serial is None:
      return
    self._serial.close()
    self._serial = None
    logger.info("Serial port closed")


class Arduino:
  def __init__(self, baudrate: int = 115200):
    self._master = _SerialPort(baudrate)
    self._cargo: Optional[CargoReading] = None
    self._cargo_at = float("-inf")

  def set_master_port(self, port: str):
    self._master.open(port)

  def send_motor(self, throttle: float, steering: float):
    throttle_pwm = int(max(-1.0, min(1.0, throttle)) * 255)
    steering_pwm = int(max(-1.0, min(1.0, steering)) * 255)
    self._master.write(f"M {throttle_pwm} {steering_pwm}")

  def send_estop(self):
    self._master.write("E")

  def send_heartbeat(self):
    self._master.write("H")

  def read_sonar(self) -> Optional[SonarReading]:
    """The newest `S` line, if any arrived. Reads every waiting line, so it
    also keeps the newest `C` line for `read_cargo`."""
    latest = None

    for line in self._master.read_lines():
      parts = line.split()
      if len(parts) >= 3 and parts[0] == "S" and parts[1] in ("C", "S", "B", "X"):
        try:
          nearest = int(parts[2])
        except ValueError:
          continue
        distances = []
        for part in parts[3:]:
          try:
            value = int(part)
          except ValueError:
            break
          distances.append(None if value >= SONAR_NO_ECHO else value)
        latest = SonarReading(
          state=parts[1],
          nearest_cm=None if nearest >= SONAR_NO_ECHO else nearest,
          distances=tuple(distances),
        )
      elif len(parts) == 6 and parts[0] == "C":
        try:
          grams, stable, lid, unlocked, fault = (int(part) for part in parts[1:])
        except ValueError:
          continue
        self._cargo = CargoReading(grams=grams, stable=bool(stable), lid_closed=bool(lid),
                                   unlocked=bool(unlocked), fault=bool(fault))
        self._cargo_at = time.monotonic()
      elif line.startswith("ERR"):
        logger.warning(f"master: {line}")
    return latest

  def read_cargo(self, now: float) -> Optional[CargoReading]:
    """The cargo node's last status, or None when it is not connected."""
    if now - self._cargo_at > _CARGO_STALE_S:
      return None
    return self._cargo

  def send_cargo(self, unlock: bool):
    """Renews (or ends) the unlock lease; the node locks within 300 ms of
    the last renewal. Only for a node that is reporting: an older master
    would answer `ERR K` to every line."""
    self._master.write(f"K {int(unlock)}")

  def send_lights(self, output: LightsOutput):
    left = int(output.front_left_signal or output.back_left_signal)
    right = int(output.front_right_signal or output.back_right_signal)
    drl = int(
      output.front_left_drl or output.front_right_drl
      or output.back_left_position or output.back_right_position
    )
    low = int(output.low_beam)
    brake = int(output.back_left_brake or output.back_right_brake)
    reverse = int(output.reverse_left or output.reverse_right)
    logo = int(output.logo)
    projector = int(output.projector)
    strip = output.strip_effect.value
    self._master.write(f"L {left} {right} {drl} {low} {brake} {reverse} {logo} {projector} {strip}")

  def close(self):
    self._master.write("L 0 0 0 0 0 0 0 0 0")
    self._master.write("E")
    self._master.close()
