from types import SimpleNamespace

from path import _RETURN_GAP_S, ForkWatch, Path

DT = 0.05


def lane(measured=True):
    return SimpleNamespace(lane_offset=12, both_rows=2 if measured else 0, band_rows=3 if measured else 0)


def run(watch, frames, path=Path.STRAIGHT):
    return [watch.update(path, perc, DT) for perc in frames]


def frames(seconds, measured):
    return [lane(measured)] * round(seconds / DT)


def test_mouth_is_a_gap_then_the_lane_back():
    events = run(ForkWatch(), frames(1.0, True) + frames(_RETURN_GAP_S + DT, False)
                 + frames(_RETURN_GAP_S + DT, True) + frames(1.0, True))
    assert events.count(True) == 1


def test_short_gap_is_not_a_mouth():
    events = run(ForkWatch(), frames(1.0, True) + frames(_RETURN_GAP_S / 2, False) + frames(1.0, True))
    assert not any(events)


def test_gap_without_the_lane_coming_back_is_not_yet_a_mouth():
    events = run(ForkWatch(), frames(_RETURN_GAP_S + DT, False) + frames(_RETURN_GAP_S / 2, True)
                 + frames(1.0, False))
    assert not any(events)


def test_two_mouths_two_events():
    mouth = frames(_RETURN_GAP_S + DT, False) + frames(_RETURN_GAP_S + DT, True)
    assert run(ForkWatch(), mouth + mouth).count(True) == 2


def test_nothing_counts_in_detour():
    watch = ForkWatch()
    run(watch, frames(_RETURN_GAP_S + DT, False), path=Path.DETOUR)
    assert not any(run(watch, frames(1.0, True)))
