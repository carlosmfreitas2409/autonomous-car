import logging
import signal
import time

import numpy as np

from config import Config

logger = logging.getLogger("headless")

_FPS_LOG_PERIOD_S = 5.0


def _slider(value: float) -> float:
    return float(np.float32(value))


class HeadlessGUI:
    def __init__(self, config: Config):
        self._params = Config(
            block_size=config.block_size,
            threshold_constant=int(config.threshold_constant),
            blur_ksize=config.blur_ksize,
            roi_x0=_slider(config.roi_x0),
            roi_x1=_slider(config.roi_x1),
            roi_y0=_slider(config.roi_y0),
            roi_y1=_slider(config.roi_y1),
            ema_alpha=_slider(config.ema_alpha),
            scan_lines=config.scan_lines,
        )
        self._running = True
        self._fps_logged_at = time.monotonic()
        signal.signal(signal.SIGINT, self._stop)

    def _stop(self, _signum, _frame):
        if not self._running:
            raise KeyboardInterrupt
        logger.info("Stopping (Ctrl+C again to force)")
        self._running = False

    def is_running(self) -> bool:
        return self._running

    def read_params(self) -> Config:
        return self._params

    def update_telemetry(self, fps: float, *_args, **_kwargs):
        now = time.monotonic()
        if now - self._fps_logged_at >= _FPS_LOG_PERIOD_S:
            self._fps_logged_at = now
            logger.info(f"FPS: {fps:.1f}")

    def poll_estop_toggle(self) -> bool:
        return False

    def poll_mode_toggle(self) -> bool:
        return False

    def poll_tuning_request(self) -> dict[str, float]:
        return {}

    def poll_challenge_request(self):
        return None

    def poll_path_request(self):
        return None

    def poll_mission_request(self):
        return None

    def poll_dock_request(self):
        return None

    def poll_open_request(self) -> bool:
        return False

    def set_pid_reset_callback(self, _callback):
        pass

    def set_master_port_callback(self, _callback):
        pass

    def set_lights_controller(self, _controller):
        pass

    def update_drive(self, *_args, **_kwargs):
        pass

    def update_lights(self, *_args, **_kwargs):
        pass

    def update_tuning(self, *_args, **_kwargs):
        pass

    def update_textures(self, *_args, **_kwargs):
        pass

    def update_challenge(self, *_args, **_kwargs):
        pass

    def update_path_status(self, *_args, **_kwargs):
        pass

    def update_sonar(self, *_args, **_kwargs):
        pass

    def update_cargo(self, *_args, **_kwargs):
        pass

    def update_signs(self, *_args, **_kwargs):
        pass

    def update_mission(self, *_args, **_kwargs):
        pass

    def render_frame(self):
        pass

    def shutdown(self):
        pass
