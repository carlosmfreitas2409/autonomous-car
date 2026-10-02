"""The snapshot and the overlay, without a thread or a camera."""

import numpy as np
import pytest

from perception.sign_tracker import Track
from perception.sign_worker import (DEBUG_HEIGHT, DEBUG_WIDTH, GREY, SignSnapshot,
                                    render_overlay)
from perception.traffic_light import LightState


def _track(**overrides):
    fields = dict(id=3, label="traffic_light", bbox=(600, 100, 700, 340), confidence=0.7,
                  distance_m=2.3, confirmed=True, light_state=LightState.UNKNOWN,
                  light_reason="rejected: no three-cell stack", rejected=True, variant=None,
                  variant_reason="", first_seen=0.0, last_seen=1.0)
    fields.update(overrides)
    return Track(**fields)


def test_snapshot_age_is_infinite_before_the_first_frame():
    assert SignSnapshot().age(100.0) == float("inf")
    assert SignSnapshot(timestamp=10.0).age(10.4) == pytest.approx(0.4)


def test_unavailable_snapshot_renders_a_fixed_size_canvas():
    canvas = render_overlay(None, [], detector_available=False, camera_open=False, status="no camera")
    assert canvas.shape == (DEBUG_HEIGHT, DEBUG_WIDTH, 3)
    # The red banner is there.
    band = canvas[DEBUG_HEIGHT // 2 - 20:DEBUG_HEIGHT // 2 + 20]
    assert (band[..., 2] >= 150).mean() > 0.3


def test_camera_frame_without_a_model_still_shows_the_frame_and_the_banner():
    frame = np.full((720, 1280, 3), 200, np.uint8)
    canvas = render_overlay(frame, [], detector_available=False, camera_open=True, status="no model")
    assert canvas.shape == (DEBUG_HEIGHT, DEBUG_WIDTH, 3)
    assert canvas[10, 10].tolist() == [200, 200, 200]
    assert canvas[DEBUG_HEIGHT // 2, DEBUG_WIDTH // 2].tolist() == [0, 0, 160]


def test_rejected_track_is_drawn_grey():
    frame = np.zeros((720, 1280, 3), np.uint8)
    canvas = render_overlay(frame, [_track()], detector_available=True, camera_open=True)
    # Box left edge x=600 in 1280 -> 300 in 640; top y=100 -> 50.
    column = canvas[60:160, 300]
    assert any(pixel.tolist() == list(GREY) for pixel in column)


def test_confirmed_light_is_drawn_in_its_state_colour():
    frame = np.zeros((720, 1280, 3), np.uint8)
    track = _track(rejected=False, light_state=LightState.RED, light_reason="")
    canvas = render_overlay(frame, [track], detector_available=True, camera_open=True)
    column = canvas[60:160, 300:302].reshape(-1, 3)
    assert any(pixel.tolist() == [60, 60, 235] for pixel in column)
