from types import SimpleNamespace

from input.autonomous import ControlProfile
from path import _CARRY_HOLD_S, HOME_PATH, DetourReturn, Path

DT = 0.05


def lane(whole=True):
    if not whole:
        return SimpleNamespace(lane_offset=None, provisional=False, both_rows=0, band_rows=0)
    return SimpleNamespace(lane_offset=12, provisional=False, both_rows=3, band_rows=3)


def drive(watch, path, seconds, whole=True, holding=None, standing=False):
    held = 0.0
    profiles = []
    for _ in range(round(seconds / DT)):
        home = watch.update(path, lane(whole), DT, standing=standing)
        if home is not None:
            path = home
        held = held + DT if holding else 0.0
        watch.observe_hold(held, DT, standing=standing)
        profiles.append(watch.profile_for(path))
    return path, profiles


def into_the_detour(watch):
    path, _ = drive(watch, Path.DETOUR, 1.0)
    path, _ = drive(watch, path, 0.5, whole=False)
    return drive(watch, path, 1.0)[0]


def test_a_long_dock_inside_the_detour_keeps_the_carry_for_the_exit():
    watch = DetourReturn()
    path = into_the_detour(watch)
    path, _ = drive(watch, path, 25.0, standing=True)
    assert path is Path.DETOUR
    path, _ = drive(watch, path, 3.0)
    assert path is HOME_PATH
    _, profiles = drive(watch, path, 2.5, whole=False, holding=True)
    assert set(profiles) == {ControlProfile.DETOUR}


def test_the_carry_still_has_its_cap_while_moving():
    watch = DetourReturn()
    path = into_the_detour(watch)
    path, _ = drive(watch, path, 2.0)
    assert path is HOME_PATH
    path, profiles = drive(watch, path, _CARRY_HOLD_S + 1.0)
    assert profiles[0] is ControlProfile.DETOUR and profiles[-1] is ControlProfile.NORMAL
