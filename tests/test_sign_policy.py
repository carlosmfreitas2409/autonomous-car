import pytest

from input.sign_policy import _SLOW_DISTANCE_M as SLOW
from input.sign_policy import _STOP_DISTANCE_M as STOP
from input.sign_policy import PolicyState, SignPolicy
from perception.sign_tracker import Track
from perception.sign_worker import SignSnapshot
from perception.traffic_light import LightState


def track(id=1, label="stop", distance=(SLOW + STOP) / 2, confirmed=True, state=LightState.UNKNOWN,
          rejected=False, variant=None):
    return Track(id=id, label=label, bbox=(0, 0, 10, 10), confidence=0.8, distance_m=distance,
                 confirmed=confirmed, light_state=state, light_reason="", rejected=rejected,
                 variant=variant, variant_reason="", first_seen=0.0, last_seen=0.0)


def light(id=1, distance=(SLOW + STOP) / 2, state=LightState.RED, **kw):
    return track(id=id, label="traffic_light", distance=distance, state=state, **kw)


BASE = 100.0


def snap(*tracks, now=0.0, camera=True, detector=True, age=0.0):
    return SignSnapshot(tracks=list(tracks), timestamp=BASE + now - age, camera_open=camera,
                        detector_available=detector)


def step(policy, snapshot, now):
    return policy.update(snapshot, BASE + now)


def feed(policy, tracks, start, end, step_s=0.1):
    constraint = None
    t = start
    while t <= end + 1e-9:
        constraint = step(policy, snap(*tracks, now=t), t)
        t += step_s
    return constraint


@pytest.mark.parametrize("distance,scale", [(SLOW, 1.0), ((SLOW + STOP) / 2, 0.5),
                                            (STOP + 0.2, 0.2 / (SLOW - STOP))])
def test_ramp_follows_distance(distance, scale):
    policy = SignPolicy()
    constraint = step(policy, snap(track(distance=distance)), 0.0)
    assert constraint.scale == pytest.approx(scale)
    assert constraint.state is PolicyState.SLOWING
    assert not constraint.hold


def test_hold_settle_dwell_release_timing():
    policy = SignPolicy()
    c = step(policy, snap(track(distance=STOP), now=0.0), 0.0)
    assert c.hold and c.scale == 0.0 and c.state is PolicyState.HOLDING

    c = step(policy, snap(track(distance=STOP), now=0.4), 0.4)
    assert c.state is PolicyState.HOLDING
    c = step(policy, snap(track(distance=STOP), now=0.6), 0.6)
    assert c.state is PolicyState.DWELLING and c.hold
    c = step(policy, snap(track(distance=STOP), now=3.4), 3.4)
    assert c.state is PolicyState.DWELLING
    c = step(policy, snap(track(distance=STOP), now=3.5), 3.5)
    assert c.state is PolicyState.CLEAR and c.scale == 1.0 and not c.hold


def test_distance_jitter_across_the_line_does_not_reopen_the_ramp():
    policy = SignPolicy()
    step(policy, snap(track(distance=STOP), now=0.0), 0.0)
    c = step(policy, snap(track(distance=STOP + 0.3), now=1.0), 1.0)
    assert c.hold and c.scale == 0.0


def test_same_id_never_triggers_twice():
    policy = SignPolicy()
    feed(policy, [track(distance=STOP)], 0.0, 3.6)
    c = feed(policy, [track(distance=STOP)], 3.7, 10.0)
    assert c.state is PolicyState.CLEAR and c.scale == 1.0


def test_every_stop_id_in_view_at_release_is_consumed():
    policy = SignPolicy()
    feed(policy, [track(id=1, distance=STOP)], 0.0, 3.4)
    step(policy, snap(track(id=1, distance=STOP), track(id=2, distance=STOP + 0.1, confirmed=False), now=3.5), 3.5)
    c = feed(policy, [track(id=1, distance=STOP), track(id=2, distance=STOP + 0.1)], 7.0, 8.0)
    assert c.state is PolicyState.CLEAR


def test_rearm_window_blocks_near_new_ids_only():
    policy = SignPolicy()
    feed(policy, [track(id=1, distance=STOP)], 0.0, 3.5)
    c = step(policy, snap(track(id=7, distance=2.3), now=4.0), 4.0)
    assert c.state is PolicyState.CLEAR
    c = step(policy, snap(track(id=8, distance=2.9), now=4.1), 4.1)
    assert c.state is PolicyState.SLOWING
    policy = SignPolicy()
    feed(policy, [track(id=1, distance=STOP)], 0.0, 3.5)
    c = step(policy, snap(track(id=7, distance=2.3), now=6.6), 6.6)
    assert c.state is PolicyState.SLOWING


def test_abandoned_approach_leaves_released_at_untouched():
    policy = SignPolicy()
    step(policy, snap(track(id=1, distance=STOP + 0.6), now=0.0), 0.0)
    c = step(policy, snap(now=1.2), 1.2)
    assert c.state is PolicyState.CLEAR
    c = step(policy, snap(track(id=2, distance=STOP + 0.4), now=1.3), 1.3)
    assert c.state is PolicyState.SLOWING


def test_hold_completes_with_no_tracks_at_all():
    policy = SignPolicy()
    step(policy, snap(track(distance=STOP), now=0.0), 0.0)
    c = step(policy, snap(now=2.0), 2.0)
    assert c.hold and c.state is PolicyState.DWELLING
    c = step(policy, snap(now=3.5), 3.5)
    assert not c.hold and c.state is PolicyState.CLEAR


def test_unconfirmed_track_never_acts():
    policy = SignPolicy()
    c = step(policy, snap(track(distance=STOP, confirmed=False)), 0.0)
    assert c.scale == 1.0 and not c.hold


def test_reset_keeps_consumed_but_drops_an_active_hold():
    policy = SignPolicy()
    feed(policy, [track(id=1, distance=STOP)], 0.0, 3.5)
    policy.reset()
    c = step(policy, snap(track(id=1, distance=STOP), now=10.0), 10.0)
    assert c.state is PolicyState.CLEAR

    policy = SignPolicy()
    step(policy, snap(track(id=1, distance=STOP), now=0.0), 0.0)
    policy.reset()
    c = step(policy, snap(track(id=1, distance=STOP), now=0.1), 0.1)
    assert c.hold


def test_stale_snapshot_is_clear_and_releases_an_open_gate():
    policy = SignPolicy()
    step(policy, snap(light(distance=(SLOW + STOP) / 2), now=-0.1), -0.1)
    step(policy, snap(light(distance=STOP), now=0.0), 0.0)
    c = step(policy, snap(light(distance=STOP), now=0.5), 0.5)
    assert c.state is PolicyState.WAITING_GREEN
    c = step(policy, snap(light(distance=STOP), now=1.0), 1.6)
    assert c.stale and c.state is PolicyState.CLEAR and c.scale == 1.0 and not c.hold
    c = step(policy, snap(light(distance=(SLOW + STOP) / 2), now=2.0), 2.0)
    assert c.state is PolicyState.CLEAR


def test_no_camera_or_no_model_does_not_change_driving():
    policy = SignPolicy()
    c = step(policy, SignSnapshot(camera_open=False), 5.0)
    assert c.scale == 1.0 and not c.hold
    c = step(policy, snap(now=5.0, detector=False), 5.0)
    assert c.scale == 1.0 and not c.hold


def test_unverified_or_unknown_light_never_slows():
    policy = SignPolicy()
    c = step(policy, snap(light(state=LightState.UNKNOWN, rejected=True)), 0.0)
    assert c.scale == 1.0
    c = step(policy, snap(light(state=LightState.UNKNOWN)), 0.1)
    assert c.scale == 1.0


def test_green_in_range_does_nothing():
    policy = SignPolicy()
    c = step(policy, snap(light(state=LightState.GREEN, distance=(SLOW + STOP) / 2)), 0.0)
    assert c.scale == 1.0 and c.state is PolicyState.CLEAR


def test_gate_forms_only_between_the_line_and_the_slow_distance():
    policy = SignPolicy()
    assert step(policy, snap(light(distance=SLOW + 0.5)), 0.0).scale == 1.0
    c = step(policy, snap(light(distance=(SLOW + STOP) / 2), now=0.1), 0.1)
    assert c.scale == pytest.approx(0.5) and c.state is PolicyState.SLOWING
    assert c.reason == "red light ahead"


def test_yellow_slows_but_never_holds():
    policy = SignPolicy()
    c = step(policy, snap(light(distance=(SLOW + STOP) / 2, state=LightState.YELLOW)), 0.0)
    assert c.scale == pytest.approx(0.5) and c.state is PolicyState.SLOWING and not c.hold
    assert c.reason == "yellow light ahead"
    c = feed(policy, [light(distance=STOP, state=LightState.YELLOW)], 0.1, 12.0)
    assert not c.hold


def test_yellow_turning_red_in_range_gates():
    policy = SignPolicy()
    step(policy, snap(light(distance=(SLOW + STOP) / 2, state=LightState.YELLOW)), 0.0)
    c = step(policy, snap(light(distance=(SLOW + STOP) / 2, state=LightState.RED), now=0.1), 0.1)
    assert c.state is PolicyState.SLOWING and c.reason == "red light ahead"
    c = step(policy, snap(light(distance=STOP, state=LightState.RED), now=0.2), 0.2)
    assert c.hold and c.state is PolicyState.WAITING_GREEN


def test_light_first_seen_under_the_car_is_waved_through():
    policy = SignPolicy()
    c = step(policy, snap(light(distance=0.7)), 0.0)
    assert c.scale == 1.0
    c = step(policy, snap(light(distance=STOP + 0.2), now=0.1), 0.1)
    assert c.scale == 1.0


def test_light_revealed_inside_the_line_holds_at_once():
    policy = SignPolicy()
    c = step(policy, snap(light(distance=STOP - 0.2)), 0.0)
    assert c.hold and c.state is PolicyState.WAITING_GREEN
    c = step(policy, snap(light(distance=STOP - 0.2, state=LightState.GREEN), now=0.5), 0.5)
    assert not c.hold


def test_stop_revealed_inside_the_line_holds_at_once():
    policy = SignPolicy()
    c = step(policy, snap(track(distance=STOP - 0.3)), 0.0)
    assert c.hold and c.state is PolicyState.HOLDING


def test_gate_holds_at_the_line_and_green_releases_it():
    policy = SignPolicy()
    step(policy, snap(light(distance=(SLOW + STOP) / 2), now=0.0), 0.0)
    c = step(policy, snap(light(distance=STOP), now=0.1), 0.1)
    assert c.hold and c.state is PolicyState.WAITING_GREEN
    c = step(policy, snap(light(distance=STOP), now=0.2), 0.2)
    assert c.hold
    c = step(policy, snap(light(distance=STOP, state=LightState.GREEN), now=0.3), 0.3)
    assert not c.hold and c.scale == 1.0 and c.state is PolicyState.CLEAR


def test_same_light_regates_when_it_goes_red_again_in_range():
    policy = SignPolicy()
    step(policy, snap(light(distance=(SLOW + STOP) / 2), now=0.0), 0.0)
    step(policy, snap(light(distance=(SLOW + STOP) / 2, state=LightState.GREEN), now=0.1), 0.1)
    c = step(policy, snap(light(distance=(SLOW + STOP) / 2, state=LightState.RED), now=0.2), 0.2)
    assert c.state is PolicyState.SLOWING and c.scale == pytest.approx(0.5)


def test_unknown_while_waiting_stays_held_until_the_ceiling():
    policy = SignPolicy()
    step(policy, snap(light(distance=(SLOW + STOP) / 2), now=-0.1), -0.1)
    step(policy, snap(light(distance=STOP), now=0.0), 0.0)
    c = feed(policy, [light(distance=STOP, state=LightState.UNKNOWN)], 0.1, 9.9)
    assert c.hold and c.state is PolicyState.WAITING_GREEN
    c = step(policy, snap(light(distance=STOP, state=LightState.UNKNOWN), now=10.0), 10.0)
    assert not c.hold and c.state is PolicyState.CLEAR
    c = step(policy, snap(light(distance=(SLOW + STOP) / 2), now=10.1), 10.1)
    assert c.state is PolicyState.CLEAR


def test_committed_light_lost_while_waiting_releases_after_two_seconds():
    policy = SignPolicy()
    step(policy, snap(light(distance=(SLOW + STOP) / 2), now=-0.1), -0.1)
    step(policy, snap(light(distance=STOP), now=0.0), 0.0)
    c = step(policy, snap(now=1.5), 1.5)
    assert c.hold
    c = step(policy, snap(now=2.1), 2.1)
    assert not c.hold


def test_hold_beats_ramp_when_both_present():
    policy = SignPolicy()
    c = step(policy, snap(track(id=1, distance=STOP), light(id=2, distance=STOP + 0.8)), 0.0)
    assert c.hold and c.scale == 0.0 and c.state is PolicyState.HOLDING


def test_lower_scale_wins():
    policy = SignPolicy()
    c = step(policy, snap(track(id=1, distance=STOP + 0.8), light(id=2, distance=STOP + 0.2)), 0.0)
    assert c.scale == pytest.approx(0.2 / (SLOW - STOP))
    assert c.reason == "red light ahead"


def test_detour_and_delivery_are_reported_not_acted_on():
    policy = SignPolicy()
    c = step(policy, snap(track(id=1, label="detour", distance=1.5),
                           track(id=2, label="delivery_sign", distance=1.8, variant="A"),
                           track(id=3, label="delivery_sign", distance=(SLOW + STOP) / 2, variant="B")), 0.0)
    assert c.scale == 1.0 and not c.hold
    assert c.detour.id == 1
    assert c.delivery.id == 2 and c.delivery.variant == "A"


def test_approach_that_never_reaches_the_line_stops_and_releases():
    policy = SignPolicy()
    c = feed(policy, [track(distance=STOP + 0.4)], 0.0, 7.9)
    assert c.state is PolicyState.SLOWING
    c = step(policy, snap(track(distance=STOP + 0.4), now=8.1), 8.1)
    assert c.hold and c.state is PolicyState.HOLDING
    c = step(policy, snap(track(distance=STOP + 0.4), now=11.7), 11.7)
    assert not c.hold and c.state is PolicyState.CLEAR
    c = feed(policy, [track(distance=STOP + 0.4)], 11.8, 20.0)
    assert c.state is PolicyState.CLEAR


def test_light_gate_that_never_reaches_the_line_waits_and_green_releases():
    policy = SignPolicy()
    feed(policy, [light(distance=STOP + 0.4)], 0.0, 8.1)
    c = step(policy, snap(light(distance=STOP + 0.4), now=8.2), 8.2)
    assert c.state is PolicyState.WAITING_GREEN
    c = step(policy, snap(light(distance=STOP + 0.4, state=LightState.GREEN), now=8.3), 8.3)
    assert c.state is PolicyState.CLEAR
