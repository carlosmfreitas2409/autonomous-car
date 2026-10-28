import numpy as np
import pytest

from perception.delivery_variant import VariantReading
from perception.sign_detection import Detection
from perception.sign_tracker import EXPIRE_S, SignTracker
from perception.traffic_light import CellStack, LightReading, LightState

FRAME = np.zeros((720, 1280, 3), np.uint8)


def _stack():
    return CellStack(cx=10.0, centres=(5.0, 15.0, 25.0), radius=3.0, contrast=(1.0, 1.0, 1.0))


def light_reader(*states, verified=True):
    queue = list(states)

    def read(_crop):
        state = queue.pop(0) if len(queue) > 1 else queue[0]
        return LightReading(state=state, cells=_stack() if verified else None,
                            reason="" if verified else "no three-cell stack")
    return read


def variant_reader(*variants):
    queue = list(variants)

    def read(_crop):
        variant = queue.pop(0) if len(queue) > 1 else queue[0]
        return VariantReading(variant=variant, reason="" if variant else "refused")
    return read


def det(label="stop", box=(500, 200, 600, 300), conf=0.8):
    return Detection(label, box, conf)


def run(tracker, frames, start=0.0, step=0.1):
    tracks = []
    for index, detections in enumerate(frames):
        tracks = tracker.update(detections, FRAME, start + index * step)
    return tracks


def test_same_object_keeps_its_id_and_a_new_one_gets_a_new_id():
    tracker = SignTracker()
    first = tracker.update([det()], FRAME, 0.0)
    second = tracker.update([det(box=(505, 203, 606, 305))], FRAME, 0.1)
    assert first[0].id == second[0].id

    third = tracker.update([det(box=(505, 203, 606, 305)), det(box=(50, 50, 150, 150))], FRAME, 0.2)
    ids = sorted(t.id for t in third)
    assert ids[0] == first[0].id and ids[1] > first[0].id


def test_different_labels_never_associate():
    tracker = SignTracker()
    tracker.update([det("stop")], FRAME, 0.0)
    tracks = tracker.update([det("detour")], FRAME, 0.1)
    assert len(tracks) == 2


def test_confirmation_needs_three_of_five():
    tracker = SignTracker()
    light = det("traffic_light", (600, 100, 700, 340))
    tracks = run(tracker, [[light]])
    assert not tracks[0].confirmed and not tracks[0].actionable
    tracks = run(tracker, [[light]], start=0.1)
    assert not tracks[0].confirmed
    tracks = run(tracker, [[light]], start=0.2)
    assert tracks[0].confirmed


def test_stop_confirms_on_two_of_three():
    tracker = SignTracker()
    tracks = run(tracker, [[det()]])
    assert not tracks[0].confirmed
    tracks = run(tracker, [[det()]], start=0.1)
    assert tracks[0].confirmed and tracks[0].actionable
    tracker = SignTracker()
    tracks = run(tracker, [[det()], [], [det()]])
    assert tracks[0].confirmed


def test_track_survives_a_short_gap_and_dies_after_expiry():
    tracker = SignTracker()
    (track,) = tracker.update([det()], FRAME, 0.0)
    (still,) = tracker.update([], FRAME, 0.4)
    assert still.id == track.id
    (again,) = tracker.update([det()], FRAME, 0.5)
    assert again.id == track.id

    tracker.update([], FRAME, 0.5 + EXPIRE_S + 0.05)
    tracks = tracker.update([det()], FRAME, 0.5 + EXPIRE_S + 0.1)
    assert len(tracks) == 1 and tracks[0].id != track.id


def test_returned_tracks_are_immutable_values():
    tracker = SignTracker()
    (track,) = tracker.update([det()], FRAME, 0.0)
    tracker.update([det(box=(600, 200, 700, 300))], FRAME, 0.1)
    assert track.bbox == (500, 200, 600, 300)
    with pytest.raises(Exception):
        track.bbox = (0, 0, 1, 1)


def test_light_hysteresis_survives_one_blown_frame():
    reader = light_reader(LightState.RED, LightState.RED, LightState.GREEN, LightState.RED)
    tracker = SignTracker(read_light=reader)
    light = det("traffic_light", (600, 100, 700, 340))
    tracks = run(tracker, [[light]] * 4)
    assert tracks[0].light_state is LightState.RED


def test_three_greens_flip_a_committed_red():
    reader = light_reader(LightState.RED, LightState.RED, LightState.RED,
                          LightState.GREEN, LightState.GREEN, LightState.GREEN)
    tracker = SignTracker(read_light=reader)
    light = det("traffic_light", (600, 100, 700, 340))
    tracks = run(tracker, [[light]] * 3)
    assert tracks[0].light_state is LightState.RED
    tracks = run(tracker, [[light]] * 2, start=0.3)
    assert tracks[0].light_state is LightState.RED
    tracks = run(tracker, [[light]], start=0.5)
    assert tracks[0].light_state is LightState.GREEN


def test_unknown_never_commits_and_a_committed_state_persists_through_it():
    tracker = SignTracker(read_light=light_reader(LightState.UNKNOWN))
    light = det("traffic_light", (600, 100, 700, 340))
    tracks = run(tracker, [[light]] * 6)
    assert tracks[0].light_state is LightState.UNKNOWN
    assert tracks[0].confirmed and not tracks[0].actionable

    reader = light_reader(LightState.RED, LightState.RED, LightState.RED, LightState.UNKNOWN)
    tracker = SignTracker(read_light=reader)
    tracks = run(tracker, [[light]] * 10)
    assert tracks[0].light_state is LightState.RED


def test_unverified_light_is_rejected_and_never_actionable():
    tracker = SignTracker(read_light=light_reader(LightState.UNKNOWN, verified=False))
    light = det("traffic_light", (600, 100, 700, 340))
    tracks = run(tracker, [[light]] * 5)
    assert tracks[0].rejected
    assert tracks[0].confirmed and not tracks[0].actionable
    assert "no three-cell stack" in tracks[0].light_reason


def test_variant_needs_agreement_and_clears_when_refused():
    tracker = SignTracker(read_variant=variant_reader("A", "A", "A", None, None, None, None))
    sign = det("delivery_sign", (600, 200, 700, 300))
    tracks = run(tracker, [[sign]] * 2)
    assert tracks[0].variant is None
    tracks = run(tracker, [[sign]], start=0.2)
    assert tracks[0].variant == "A"
    tracks = run(tracker, [[sign]] * 4, start=0.3)
    assert tracks[0].variant is None


def test_distance_matches_the_260px_anchor():
    tracker = SignTracker(read_light=light_reader(LightState.RED))
    light = det("traffic_light", (600, 200, 700, 460))
    (track,) = tracker.update([light], FRAME, 0.0)
    assert track.distance_m == pytest.approx(2.0, abs=0.02)


def test_edge_clipped_box_does_not_update_distance():
    tracker = SignTracker()
    (track,) = tracker.update([det(box=(500, 0, 600, 300))], FRAME, 0.0)
    assert track.distance_m is None
    (track,) = tracker.update([det(box=(500, 100, 600, 400))], FRAME, 0.1)
    before = track.distance_m
    (track,) = tracker.update([det(box=(500, 200, 600, 719))], FRAME, 0.2)
    assert before is not None and track.distance_m == before


def test_distance_uses_the_frame_width_it_is_given():
    tracker = SignTracker()
    small = np.zeros((360, 640, 3), np.uint8)
    (track,) = tracker.update([det("traffic_light", (300, 100, 350, 230))], small, 0.0)
    assert track.distance_m == pytest.approx(2.0, abs=0.02)


def test_reset_clears_tracks_but_not_ids():
    tracker = SignTracker()
    (track,) = tracker.update([det()], FRAME, 0.0)
    tracker.reset()
    (fresh,) = tracker.update([det()], FRAME, 0.1)
    assert fresh.id > track.id
