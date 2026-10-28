from dataclasses import dataclass


@dataclass
class Config:
    downscale_ratio: float = 2.0

    # Canny
    block_size: int = 95
    blur_ksize: int = 9
    threshold_constant: float = -26

    # ROI
    roi_x0: float = 0.01
    roi_x1: float = 0.99
    roi_y0: float = 0.60
    roi_y1: float = 0.85
    ema_alpha: float = 0.7
    scan_lines: int = 5

    # PID
    kp: float = 0.0140
    ki: float = 0.0
    kd: float = 0.0027

    # Speed
    base_speed: float = 0.80
    min_speed: float = 0.67


SPEED_TEST = Config(base_speed=1.0, min_speed=0.70)
