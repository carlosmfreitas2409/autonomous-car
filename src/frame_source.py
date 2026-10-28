import logging
import threading
import time

logger = logging.getLogger(__name__)


class FrameSource:
    def __init__(self, cap, live: bool):
        self._cap = cap
        self._live = live
        self._lock = threading.Condition()
        self._frame = None
        self._seq = 0
        self._failed = False
        self._stop = threading.Event()
        self._closed = False
        self._thread = None
        if live:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            with self._lock:
                if ok:
                    self._frame = frame
                    self._seq += 1
                    self._failed = False
                else:
                    self._failed = True
                self._lock.notify_all()
            if not ok:
                time.sleep(0.01)

    def read(self, timeout: float = 0.5):
        if not self._live:
            return self._cap.read()
        with self._lock:
            seen = getattr(self, "_handed", 0)
            deadline = time.monotonic() + timeout
            while self._seq == seen and not self._failed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False, None
                self._lock.wait(remaining)
            if self._seq == seen:
                return False, None
            self._handed = self._seq
            return True, self._frame

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                logger.warning("Camera reader did not stop; leaving the capture to the process exit")
                return
        self._cap.release()
