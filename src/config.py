from dataclasses import dataclass


@dataclass
class Config:
    downscale_ratio: float = 2.3

    # Canny
    block_size: int = 81
    blur_ksize: int = 7
    threshold_constant: float = -26

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
