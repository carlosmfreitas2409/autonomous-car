import serial
import serial.tools.list_ports
import logging
import time

from dataclasses import dataclass
from typing import Optional

from cargo import CargoReading
from lights import LightsOutput

logger = logging.getLogger(__name__)

SONAR_NO_ECHO = 0xFF

_CARGO_STALE_S = 1.0


@dataclass
class SonarReading:
  state: str
  nearest_cm: Optional[int]
  distances: tuple[Optional[int], ...] = ()


@dataclass
class MasterTelemetry:
  angle_raw: int
  target_raw: int
  throttle_pwm: int
  steer_pwm: int
  state: str
  ms_since_command: int
  fault: Optional[str]
  received_at: float


@dataclass
class BatteryReading:
  volts: float
  percent: int
  received_at: float


def parse_battery(line: str, now: float) -> Optional[BatteryReading]:
  parts = line.split()
  if len(parts) < 3 or parts[0] != "B":
    return None
  try:
    millivolts, percent = int(parts[1]), int(parts[2])
  except ValueError:
    return None
  return BatteryReading(millivolts / 1000.0, max(0, min(100, percent)), now)


def parse_telemetry(line: str, now: float) -> Optional[MasterTelemetry]:
  parts = line.split()
  if len(parts) < 7 or parts[0] != "T" or parts[5] not in ("R", "W", "E", "F"):
    return None
  try:
    angle, target, throttle, steer = (int(p) for p in parts[1:5])
    ms_since = int(parts[6])
  except ValueError:
    return None
  fault = parts[7] if len(parts) > 7 else None
  return MasterTelemetry(angle, target, throttle, steer, parts[5], ms_since, fault, now)


def usb_serial_ports() -> list[str]:
  return [port.device for port in serial.tools.list_ports.comports() if port.vid is not None]


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

  @property
  def is_open(self) -> bool:
    return self._serial is not None

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
    self.telemetry: Optional[MasterTelemetry] = None
    self.battery: Optional[BatteryReading] = None

  @property
  def is_open(self) -> bool:
    return self._master.is_open

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
      elif len(parts) == 6 and parts[0] == "C":
        try:
          grams, stable, lid, unlocked, fault = (int(part) for part in parts[1:])
        except ValueError:
          continue
        self._cargo = CargoReading(grams=grams, stable=bool(stable), lid_closed=bool(lid),
                                   unlocked=bool(unlocked), fault=bool(fault))
        self._cargo_at = time.monotonic()
      elif line.startswith("T "):
        telemetry = parse_telemetry(line, time.monotonic())
        if telemetry is not None:
          self.telemetry = telemetry
      elif line.startswith("B "):
        battery = parse_battery(line, time.monotonic())
        if battery is not None:
          self.battery = battery
      elif line.startswith("ERR"):
        logger.warning(f"master: {line}")
    return latest

  def read_cargo(self, now: float) -> Optional[CargoReading]:
    if now - self._cargo_at > _CARGO_STALE_S:
      return None
    return self._cargo

  def send_cargo(self, unlock: bool):
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
