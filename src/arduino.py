import serial
import logging
import time

from dataclasses import dataclass
from typing import Optional

from lights import LightsOutput

logger = logging.getLogger(__name__)

SONAR_NO_ECHO = 0xFF


@dataclass
class SonarReading:
  state: str
  nearest_cm: Optional[int]
  # Per-sensor distances, left to right, None where nothing echoed. Empty when
  # the master only sent the nearest value.
  distances: tuple[Optional[int], ...] = ()

  @property
  def blocked(self) -> bool:
    return self.state in ("B", "X")


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
      elif line.startswith("ERR"):
        logger.warning(f"master: {line}")
    return latest

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
