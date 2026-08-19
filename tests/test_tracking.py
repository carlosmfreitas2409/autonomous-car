"""Detection association, confirmation, expiry, and distance."""

import pytest

from perception.sign_detection import Detection
from perception.tracking import (
    DetectionTracker,
    REAL_HEIGHT_M,
    distance_from_height,
    focal_px_from_hfov,
)
from perception.traffic_light import LightState


FRAME_W = 640
STEP = 0.05


def stop_sign(x0=300, y0=200, x1=350, y1=250, confidence=0.9):
    return Detection(label="stop", bbox=(x0, y0, x1, y1), confidence=confidence)


def light(state=LightState.UNKNOWN, x0=100, y0=100, x1=130, y1=190):
    return Detection(
        label="traffic_light",
        bbox=(x0, y0, x1, y1),
        confidence=0.9,
        light_state=state,
        light_confidence=0.8,
    )


def run(tracker, frames, start=100.0):
    """Feed a list of per-frame detection lists; return the final tracks."""
    tracks = []
    for i, detections in enumerate(frames):
        tracks = tracker.update(detections, FRAME_W, start + i * STEP)
    return tracks


def test_new_detection_creates_a_track():
    tracks = run(DetectionTracker(), [[stop_sign()]])
    assert len(tracks) == 1
    assert tracks[0].label == "stop"
    assert tracks[0].id == 1


def test_same_object_keeps_its_id_while_it_moves():
    tracker = DetectionTracker()
    ids = []
    for i in range(5):
        drift = i * 4
        tracks = tracker.update([stop_sign(300 + drift, 200, 350 + drift, 250)], FRAME_W, 100.0 + i * STEP)
        ids.append(tracks[0].id)
    assert set(ids) == {1}


def test_disjoint_box_becomes_a_new_track():
    tracker = DetectionTracker()
    run(tracker, [[stop_sign()]])
    tracks = tracker.update([stop_sign(10, 10, 60, 60)], FRAME_W, 100.1)
    assert {t.id for t in tracks} == {1, 2}


def test_same_box_with_a_different_label_is_not_associated():
    tracker = DetectionTracker()
    run(tracker, [[stop_sign(100, 100, 130, 190)]])
    tracks = tracker.update([light()], FRAME_W, 100.1)
    assert {t.label for t in tracks} == {"stop", "traffic_light"}
    assert len(tracks) == 2


def test_two_objects_tracked_independently():
    tracker = DetectionTracker()
    tracks = run(tracker, [[stop_sign(), light()]] * 3)
    assert len({t.id for t in tracks}) == 2
    assert {t.label for t in tracks} == {"stop", "traffic_light"}


def test_track_is_not_confirmed_on_a_single_frame():
    tracks = run(DetectionTracker(), [[stop_sign()]])
    assert tracks[0].confirmed is False


def test_track_confirms_after_repeated_sightings():
    tracks = run(DetectionTracker(), [[stop_sign()]] * 3)
    assert tracks[0].confirmed is True


def test_intermittent_detection_still_confirms():
    """One dropped frame in the middle should not reset confirmation."""
    tracks = run(DetectionTracker(), [[stop_sign()], [], [stop_sign()], [stop_sign()]])
    assert tracks[0].confirmed is True


def test_track_expires_once_unseen_for_too_long():
    tracker = DetectionTracker()
    tracker.update([stop_sign()], FRAME_W, 100.0)
    assert tracker.update([], FRAME_W, 100.2)
    assert tracker.update([], FRAME_W, 101.0) == []


@pytest.mark.parametrize("gap_seconds", [1.0, 5.0, 12.0, 30.0, 90.0])
def test_a_sign_returning_after_a_lap_gets_a_fresh_id(gap_seconds):
    """The load-bearing assumption behind stopping again on every lap: a sign
    that has left the camera and come back is a new track, even though the box
    lands in exactly the same place."""
    tracker = DetectionTracker()
    first = run(tracker, [[stop_sign()]] * 3)[0].id

    tracker.update([], FRAME_W, 100.0 + gap_seconds)
    second = tracker.update([stop_sign()], FRAME_W, 100.5 + gap_seconds)[0].id

    assert second != first


def test_reset_clears_tracks_without_reusing_ids():
    """The policy keys "already handled" off the track ID, so a reset must not
    hand a fresh object an ID that already means something."""
    tracker = DetectionTracker()
    run(tracker, [[stop_sign()]] * 3)
    tracker.reset()
    tracks = tracker.update([stop_sign()], FRAME_W, 200.0)
    assert len(tracks) == 1
    assert tracks[0].id != 1


def test_focal_length_matches_the_pinhole_model():
    # A 90 degree field of view puts the focal length at exactly half the width.
    assert focal_px_from_hfov(640, 90.0) == pytest.approx(320.0)


def test_distance_matches_the_pinhole_model():
    focal = focal_px_from_hfov(FRAME_W)
    expected = REAL_HEIGHT_M["stop"] * focal / 50
    tracks = run(DetectionTracker(), [[stop_sign(300, 200, 350, 250)]])
    assert tracks[0].distance_m == pytest.approx(expected)


def test_distance_is_none_without_a_degenerate_box():
    assert distance_from_height(0.15, 500.0, 0) is None


def test_distance_shrinks_as_the_sign_grows():
    tracker = DetectionTracker()
    far = tracker.update([stop_sign(300, 200, 320, 220)], FRAME_W, 100.0)[0].distance_m
    near = None
    for i in range(1, 8):
        size = 20 + i * 10
        near = tracker.update([stop_sign(300, 200, 300 + size, 200 + size)], FRAME_W, 100.0 + i * STEP)[0].distance_m
    assert near < far


def test_two_red_frames_commit_to_red():
    tracks = run(DetectionTracker(), [[light(LightState.RED)]] * 2)
    assert tracks[0].light_state is LightState.RED


def test_one_red_frame_does_not_commit():
    tracks = run(DetectionTracker(), [[light(LightState.RED)]])
    assert tracks[0].light_state is LightState.UNKNOWN


def test_releasing_to_green_takes_longer_than_committing_to_red():
    tracker = DetectionTracker()
    frames = [[light(LightState.RED)]] * 2
    assert run(tracker, frames)[0].light_state is LightState.RED

    # Three greens is not enough while reds are still in the window.
    tracks = run(tracker, [[light(LightState.GREEN)]] * 3, start=101.0)
    assert tracks[0].light_state is LightState.RED

    tracks = run(tracker, [[light(LightState.GREEN)]], start=102.0)
    assert tracks[0].light_state is LightState.GREEN


def test_red_interrupts_green_quickly():
    tracker = DetectionTracker()
    run(tracker, [[light(LightState.GREEN)]] * 4)
    tracks = run(tracker, [[light(LightState.RED)]] * 2, start=101.0)
    assert tracks[0].light_state is LightState.RED


def test_unknown_readings_leave_the_committed_state_standing():
    tracker = DetectionTracker()
    run(tracker, [[light(LightState.RED)]] * 2)
    tracks = run(tracker, [[light(LightState.UNKNOWN)]] * 5, start=101.0)
    assert tracks[0].light_state is LightState.RED


def test_yellow_commits_as_quickly_as_red():
    tracks = run(DetectionTracker(), [[light(LightState.YELLOW)]] * 2)
    assert tracks[0].light_state is LightState.YELLOW
