from dataclasses import dataclass


@dataclass
class Config:
    downscale_ratio: float = 1.5

    # Canny
    block_size: int = 51
    blur_ksize: int = 3
    threshold_constant: float = -25

    # ROI
    roi_y0: float = 0.8
    roi_y1: float = 0.92
    ema_alpha: float = 0.4
    scan_lines: int = 5

    # PID
    kp: float = 0.01
    ki: float = 0.0
    kd: float = 0.005

    # Speed
    base_speed: float = 0.4
    min_speed: float = 0.2
