"""Behaviour: approaching, stopping, dwelling, and gating on a light."""

import pytest

from input.traffic_policy import PolicyState, TrafficPolicy
from perception.sign_worker import SignSnapshot
from perception.tracking import Track
from perception.traffic_light import LightState


THROTTLE = 0.5
DT = 0.1


def track(label, *, id=1, distance=5.0, confirmed=True, light_state=LightState.UNKNOWN):
    return Track(
        id=id,
        label=label,
        bbox=(100, 100, 140, 140),
        confidence=0.9,
        distance_m=distance,
        light_state=light_state,
        light_confidence=0.8,
        confirmed=confirmed,
        first_seen=0.0,
        last_seen=0.0,
    )


def snapshot(tracks, now, age=0.0):
    return SignSnapshot(
        tracks=tracks,
        timestamp=now - age,
        camera_open=True,
        detector_available=True,
    )


def step(policy, tracks, now, throttle=THROTTLE, dt=DT, age=0.0):
    return policy.update(snapshot(tracks, now, age), throttle, dt, now)


def arrive_at_light(policy, state, start=100.0):
    """See the light during the approach, then reach the line.

    A gate only forms while the cart is still short of the stop distance, so a
    light that is already at the line the first time it is seen is waved
    through. Tests that want a hold have to approach for it.
    """
    step(policy, [track("traffic_light", distance=2.0, light_state=state)], start)
    return step(policy, [track("traffic_light", distance=0.4, light_state=state)], start + 0.1)


def drive_until(policy, tracks_for, predicate, *, start=100.0, limit=400):
    """Run the policy forward until predicate(constraint) holds."""
    now = start
    for _ in range(limit):
        constraint = step(policy, tracks_for(now), now)
        if predicate(constraint):
            return constraint, now
        now += DT
    raise AssertionError("policy never reached the expected state")


def test_empty_scene_imposes_no_constraint():
    constraint = step(TrafficPolicy(), [], 100.0)
    assert constraint.speed_cap == 1.0
    assert constraint.hold is False
    assert constraint.state is PolicyState.CLEAR


def test_distant_stop_sign_is_ignored():
    constraint = step(TrafficPolicy(), [track("stop", distance=8.0)], 100.0)
    assert constraint.speed_cap == 1.0
    assert constraint.state is PolicyState.CLEAR


def test_unconfirmed_track_is_ignored():
    constraint = step(TrafficPolicy(), [track("stop", distance=1.0, confirmed=False)], 100.0)
    assert constraint.state is PolicyState.CLEAR


def test_stale_snapshot_is_ignored_and_reported():
    constraint = step(TrafficPolicy(), [track("stop", distance=1.0)], 100.0, age=2.0)
    assert constraint.stale is True
    assert constraint.speed_cap == 1.0


def test_approaching_stop_sign_slows_down():
    constraint = step(TrafficPolicy(), [track("stop", distance=2.0)], 100.0)
    assert constraint.state is PolicyState.SLOWING
    assert 0.0 < constraint.speed_cap < 1.0
    assert constraint.hold is False


def test_speed_ceiling_falls_as_the_sign_gets_closer():
    far = step(TrafficPolicy(), [track("stop", distance=2.5)], 100.0).speed_cap
    near = step(TrafficPolicy(), [track("stop", distance=1.0)], 100.0).speed_cap
    assert near < far


def test_stop_sign_at_the_line_holds():
    constraint = step(TrafficPolicy(), [track("stop", distance=0.4)], 100.0)
    assert constraint.hold is True
    assert constraint.speed_cap == 0.0
    assert constraint.state is PolicyState.HOLDING


def test_hold_becomes_a_dwell_then_releases():
    policy = TrafficPolicy()
    sign = [track("stop", distance=0.4)]

    assert step(policy, sign, 100.0).state is PolicyState.HOLDING
    assert step(policy, sign, 101.0).state is PolicyState.DWELLING
    assert step(policy, sign, 102.0).hold is True

    released = step(policy, sign, 104.0)
    assert released.hold is False
    assert released.state is PolicyState.CLEAR


def test_a_consumed_stop_sign_never_triggers_again():
    policy = TrafficPolicy()
    sign = [track("stop", distance=0.4)]
    step(policy, sign, 100.0)
    step(policy, sign, 104.0)

    for offset in (0.0, 1.0, 5.0):
        constraint = step(policy, sign, 104.5 + offset)
        assert constraint.hold is False
        assert constraint.state is PolicyState.CLEAR


def test_stop_is_latched_after_the_sign_leaves_the_frame():
    """The sign leaves a forward camera's view before the cart reaches it, so
    losing sight of it must not cancel the stop."""
    policy = TrafficPolicy()
    step(policy, [track("stop", distance=1.0)], 100.0)

    constraint, _ = drive_until(policy, lambda _now: [], lambda c: c.hold)
    assert constraint.state is PolicyState.HOLDING


def test_a_latch_is_abandoned_if_the_sign_never_reappears():
    policy = TrafficPolicy()
    step(policy, [track("stop", distance=3.0)], 100.0)
    constraint = step(policy, [], 110.0, throttle=0.0)
    assert constraint.hold is False
    assert constraint.state is PolicyState.CLEAR


def test_red_light_slows_then_waits_for_green():
    policy = TrafficPolicy()
    red = lambda d: [track("traffic_light", distance=d, light_state=LightState.RED)]

    assert step(policy, red(2.0), 100.0).state is PolicyState.SLOWING
    constraint = step(policy, red(0.4), 100.1)
    assert constraint.hold is True
    assert constraint.state is PolicyState.WAITING_GREEN


def test_green_releases_the_hold():
    policy = TrafficPolicy()
    assert arrive_at_light(policy, LightState.RED).hold is True

    constraint = step(policy, [track("traffic_light", distance=0.4, light_state=LightState.GREEN)], 100.2)
    assert constraint.hold is False
    assert constraint.state is PolicyState.CLEAR


def test_unreadable_light_keeps_the_cart_stopped():
    policy = TrafficPolicy()
    assert arrive_at_light(policy, LightState.RED).hold is True

    for i in range(2, 20):
        constraint = step(policy, [track("traffic_light", distance=0.4)], 100.0 + i * DT)
        assert constraint.hold is True


def test_unreadable_light_on_approach_slows_but_does_not_stop():
    constraint = step(TrafficPolicy(), [track("traffic_light", distance=2.0)], 100.0)
    assert constraint.hold is False
    assert constraint.state is PolicyState.SLOWING
    assert constraint.speed_cap < 1.0


def test_green_light_on_approach_imposes_nothing():
    constraint = step(
        TrafficPolicy(),
        [track("traffic_light", distance=2.0, light_state=LightState.GREEN)],
        100.0,
    )
    assert constraint.speed_cap == 1.0
    assert constraint.state is PolicyState.CLEAR


def test_hold_always_ends_eventually():
    """A cart frozen forever on a light it cannot read is the worst outcome."""
    policy = TrafficPolicy()
    assert arrive_at_light(policy, LightState.RED).hold is True

    red = [track("traffic_light", distance=0.4, light_state=LightState.RED)]
    assert step(policy, red, 105.0).hold is True
    assert step(policy, red, 125.0).hold is False


def test_yellow_is_treated_like_red():
    assert arrive_at_light(TrafficPolicy(), LightState.YELLOW).hold is True


def test_detour_sign_is_reported_without_constraining():
    constraint = step(TrafficPolicy(), [track("detour", distance=1.5)], 100.0)
    assert constraint.detour is not None
    assert constraint.detour.distance_m == pytest.approx(1.5)
    assert constraint.speed_cap == 1.0
    assert constraint.state is PolicyState.CLEAR


def test_the_most_restrictive_rule_wins():
    policy = TrafficPolicy()
    constraint = step(
        policy,
        [
            track("stop", id=1, distance=0.4),
            track("traffic_light", id=2, distance=2.0, light_state=LightState.RED),
        ],
        100.0,
    )
    assert constraint.hold is True
    assert constraint.reason == "stopped at sign"


def test_a_light_turning_red_as_the_cart_passes_does_not_halt_it_on_the_line():
    """Past the line with nothing committed, the cart was waved through.
    Stopping now would leave it parked on top of the light."""
    policy = TrafficPolicy()
    green = [track("traffic_light", distance=1.5, light_state=LightState.GREEN)]
    assert step(policy, green, 100.0).hold is False

    caught_out = step(policy, [track("traffic_light", distance=0.4, light_state=LightState.RED)], 100.5)
    assert caught_out.hold is False
    assert caught_out.state is PolicyState.CLEAR


def test_a_light_turning_red_during_the_approach_still_halts():
    """The counterpart: still short of the line, so it must stop."""
    policy = TrafficPolicy()
    step(policy, [track("traffic_light", distance=2.5, light_state=LightState.GREEN)], 100.0)

    constraint = step(policy, [track("traffic_light", distance=2.0, light_state=LightState.RED)], 100.5)
    assert constraint.state is PolicyState.SLOWING
    assert step(policy, [track("traffic_light", distance=0.4, light_state=LightState.RED)], 101.0).hold is True


def test_a_sign_reappearing_with_a_new_id_cannot_stop_the_cart_twice():
    """"Consumed" is keyed on track ID, so a sign that blinked out of view
    during the halt comes back unrecognised. The re-arm delay covers it."""
    policy = TrafficPolicy()
    step(policy, [track("stop", id=1, distance=0.4)], 100.0)
    assert step(policy, [track("stop", id=1, distance=0.4)], 104.0).hold is False

    reappeared = [track("stop", id=99, distance=0.5)]
    for offset in (0.1, 0.5, 1.0, 1.8):
        assert step(policy, reappeared, 104.0 + offset).hold is False


@pytest.mark.parametrize("lap_seconds", [5.0, 8.0, 12.0, 20.0, 29.0, 31.0, 60.0])
def test_the_same_sign_stops_the_cart_again_on_every_lap(lap_seconds):
    """Laps are driven at whatever speed they are driven at, so this must not
    depend on lap duration.

    It does not, because a sign that has left the camera comes back as a new
    track, and track IDs are never reused. The expiry on the consumed set is
    only there to bound memory - deliberately not load-bearing, since a lap
    shorter than it would otherwise skip the sign entirely.
    """
    policy = TrafficPolicy()

    # Lap 1: approach, stop, dwell, pull away.
    step(policy, [track("stop", id=1, distance=2.0)], 100.0)
    step(policy, [track("stop", id=1, distance=0.4)], 101.0)
    assert step(policy, [track("stop", id=1, distance=0.4)], 105.0).hold is False

    # Driving the rest of the lap with the sign behind the cart.
    now = 105.0
    while now < 105.0 + lap_seconds:
        now += 0.5
        assert step(policy, [], now).hold is False

    # Lap 2: the same physical sign, seen fresh, so a new track ID.
    step(policy, [track("stop", id=2, distance=2.0)], now)
    assert step(policy, [track("stop", id=2, distance=0.4)], now + 1.0).hold is True


def test_reset_clears_latches_and_consumption():
    policy = TrafficPolicy()
    sign = [track("stop", distance=0.4)]
    step(policy, sign, 100.0)
    step(policy, sign, 104.0)
    policy.reset()

    assert step(policy, sign, 105.0).hold is True


def test_pickup_markers_are_reported_without_constraining():
    """Detected and handed to routing, but they change no behaviour here."""
    constraint = step(TrafficPolicy(), [track("pickup_b", distance=1.2)], 100.0)
    assert constraint.pickup is not None
    assert constraint.pickup.label == "pickup_b"
    assert constraint.speed_cap == 1.0
    assert constraint.hold is False


def test_the_nearest_pickup_marker_is_the_one_reported():
    constraint = step(
        TrafficPolicy(),
        [track("pickup_a", id=1, distance=2.5), track("pickup_c", id=2, distance=0.9)],
        100.0,
    )
    assert constraint.pickup.label == "pickup_c"
