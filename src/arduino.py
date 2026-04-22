import serial
import logging
import serial.tools.list_ports

from typing import Optional

from lights import LightsOutput

logger = logging.getLogger(__name__)

class Arduino:
  def __init__(self, port: Optional[str] = None, baudrate: int = 115200):
    self._baudrate = baudrate
    self._serial: Optional[serial.Serial] = None
    if port:
      self.set_port(port)

  def _write(self, message: str):
    if self._serial is None:
      return

    try:
      self._serial.write(f"{message}\n".encode("ascii"))
    except serial.SerialException as e:
      logger.error(f"Serial write failed: {e}")

  def list_ports(self) -> list[str]:
    return [port.device for port in serial.tools.list_ports.comports()]

  def set_port(self, port: str):
    print(port)
    try:
      self.close()
      self._serial = serial.Serial(port, self._baudrate, timeout=0.01)
      logger.info(f"Opened serial port {port} at {self._baudrate} baud")
    except serial.SerialException as e:
      logger.error(f"Failed to open serial port {port}: {e}")
      raise

  def send_motor(self, throttle: float, steering: float):
    throttle_pwm = int(max(0.0, min(1.0, throttle)) * 255)
    steering_pwm = int(max(-1.0, min(1.0, steering)) * 255)
    self._write(f"M {throttle_pwm} {steering_pwm}")

  def send_estop(self):
    self._write("E")
    
  def send_heartbeat(self):
    self._write("H")

  def send_lights(self, output: LightsOutput):
    self._write(
      "L "
      f"{int(output.front_left_signal)} {int(output.front_right_signal)} "
      f"{int(output.back_left_signal)} {int(output.back_right_signal)} "
      f"{int(output.front_left_drl)} {int(output.front_right_drl)} "
      f"{int(output.low_beam)} "
      f"{output.tail_left} {output.tail_right} "
      f"{int(output.logo_left)} {int(output.logo_right)}"
    )

  def read(self) -> Optional[str]:
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
    self.send_estop()
    self._serial.close()
    self._serial = None
    logger.info("Serial port closed")