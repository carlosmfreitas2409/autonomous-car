from dataclasses import dataclass


@dataclass
class Config:
    downscale_ratio: float = 2.0

    # Canny
    block_size: int = 95
    blur_ksize: int = 9
    threshold_constant: float = -26

    # ROI
    roi_x0: float = 0.03
    roi_x1: float = 0.98
    roi_y0: float = 0.67
    roi_y1: float = 0.94
    ema_alpha: float = 0.4
    scan_lines: int = 5

    # PID
    kp: float = 0.0231
    ki: float = 0.0
    kd: float = 0.0027

    # Speed
    base_speed: float = 0.63
    min_speed: float = 0.50
