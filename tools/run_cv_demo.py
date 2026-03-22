"""CV integration demo — shows how OpenCV connects to the 3D simulator.

The car moves forward at constant throttle while the dashcam feed is
displayed in an OpenCV window.  No CV logic yet — this just proves the
pipeline (dashcam frame → OpenCV → vehicle command) works end-to-end.

Usage:
    python3 tools/run_cv_demo.py
"""

import time

import cv2

from simulator.config import TICK_RATE
from simulator.vehicle import SimulatedVehicle3D


THROTTLE = 0.0
STEERING = 0.0


def main() -> None:
    vehicle: SimulatedVehicle3D | None = None
    try:
        vehicle = SimulatedVehicle3D()
        dt = 1.0 / TICK_RATE

        while True:
            loop_start = time.monotonic()

            vehicle.send_command(THROTTLE, STEERING)

            frame = vehicle.get_frame()
            if frame is not None:
                # --- CV processing would go here ---
                # e.g. steering = detect_lanes(frame)
                # e.g. sign = detect_sign(frame)
                # e.g. light = detect_traffic_light(frame)
                cv2.imshow("dashcam", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            elapsed = time.monotonic() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        if vehicle is not None:
            vehicle.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
