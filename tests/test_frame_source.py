import threading
import time

from frame_source import FrameSource


class FakeCapture:
    def __init__(self):
        self._reading = threading.Lock()
        self.released_during_read = False
        self.released = 0

    def read(self):
        with self._reading:
            time.sleep(0.03)
            return True, object()

    def release(self):
        if self._reading.locked():
            self.released_during_read = True
        self.released += 1


def test_close_stops_the_reader_before_releasing():
    for _ in range(20):
        capture = FakeCapture()
        source = FrameSource(capture, live=True)
        ok, _frame = source.read()
        assert ok
        source.close()
        assert not capture.released_during_read and capture.released == 1


def test_close_twice_releases_once():
    capture = FakeCapture()
    source = FrameSource(capture, live=True)
    source.close()
    source.close()
    assert capture.released == 1


def test_a_recording_is_read_in_order_and_released():
    capture = FakeCapture()
    source = FrameSource(capture, live=False)
    assert source.read()[0]
    source.close()
    assert capture.released == 1
