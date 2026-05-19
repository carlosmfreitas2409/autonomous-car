import serial
import logging

from typing import Optional

from lights import LightsOutput

logger = logging.getLogger(__name__)


class _SerialPort:
  def __init__(self, baudrate: int):
    self._baudrate = baudrate
    self._serial: Optional[serial.Serial] = None

  def open(self, port: str):
    try:
      self.close()
      self._serial = serial.Serial(port, self._baudrate, timeout=0.01)
      logger.info(f"Opened serial port {port} at {self._baudrate} baud")
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

  def close(self):
    if self._serial is None:
      return
    self._serial.close()
    self._serial = None
    logger.info("Serial port closed")


class Arduino:
  def __init__(self, baudrate: int = 115200):
    self._master = _SerialPort(baudrate)
    self._lights = _SerialPort(baudrate)

  def set_master_port(self, port: str):
    self._master.open(port)

  def set_lights_port(self, port: str):
    self._lights.open(port)

  def send_motor(self, throttle: float, steering: float):
    throttle_pwm = int(max(0.0, min(1.0, throttle)) * 255)
    steering_pwm = int(max(-1.0, min(1.0, steering)) * 255)
    self._master.write(f"M {throttle_pwm} {steering_pwm}")

  def send_estop(self):
    self._master.write("E")

  def send_heartbeat(self):
    self._master.write("H")

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
    logo = int(output.logo_left or output.logo_right)
    self._lights.write(f"L {left} {right} {drl} {low} {brake} {reverse} {logo}")

  def close(self):
    self._master.write("E")
    self._lights.write("L 0 0 0 0 0 0 0")
    self._master.close()
    self._lights.close()
