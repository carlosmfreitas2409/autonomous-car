"""Pre-computed route table for the oval track with two inner curve shortcuts."""

from enum import Enum, auto
from dataclasses import dataclass


class Decision(Enum):
    INNER = auto()
    OUTER = auto()


class SignAction(Enum):
    NONE = auto()
    PICKUP = auto()
    DELIVERY = auto()


@dataclass(frozen=True)
class LapPlan:
    """Plan for one lap: which path to take at each decision point.

    dp2 is None when dp1 is INNER (inner curve at DP1 skips DP2).
    """
    dp1: Decision
    dp2: Decision | None = None


# Sign-to-decision-point mapping.
# Each sign is reached by a specific decision at a specific DP.
# Format: sign_number -> (decision_point, decision_to_reach_it)
SIGN_LOCATION: dict[int, tuple[int, Decision]] = {
    1: (1, Decision.OUTER),
    4: (1, Decision.INNER),
    2: (2, Decision.OUTER),
    3: (2, Decision.INNER),
}

# Clockwise visit order for each path type.
# Maps path tuple (dp1, dp2) to the ordered list of signs visited.
_PATH_SIGNS: dict[tuple[Decision, Decision | None], list[int]] = {
    (Decision.INNER, None): [4],
    (Decision.OUTER, Decision.OUTER): [1, 2, 3, 4],
    (Decision.OUTER, Decision.INNER): [1, 3, 4],
}


def signs_for_path(dp1: Decision, dp2: Decision | None = None) -> list[int]:
    """Return the ordered list of signs visited for a given path."""
    return list(_PATH_SIGNS[(dp1, dp2)])


def compute_route(pickup: int, delivery: int) -> list[LapPlan]:
    """Compute the optimal route for a (pickup, delivery) pair.

    Returns a list of 1-2 LapPlans. Non-mission decision points default
    to INNER (shorter distance).
    """
    if pickup == delivery:
        raise ValueError("Pickup and delivery must be different signs")
    if pickup not in SIGN_LOCATION or delivery not in SIGN_LOCATION:
        raise ValueError(f"Invalid sign number: pickup={pickup}, delivery={delivery}")

    # Try all single-lap paths, prefer the shortest one where pickup < delivery
    candidates = []
    for (dp1, dp2), signs in _PATH_SIGNS.items():
        if pickup in signs and delivery in signs:
            pi = signs.index(pickup)
            di = signs.index(delivery)
            if pi < di:
                candidates.append((len(signs), LapPlan(dp1=dp1, dp2=dp2)))

    if candidates:
        candidates.sort(key=lambda c: c[0])
        return [candidates[0][1]]

    # Need two laps: one for pickup, one for delivery.
    # Find the shortest path that visits the pickup sign.
    pickup_lap = _shortest_path_for_sign(pickup)
    # Find the shortest path that visits the delivery sign.
    delivery_lap = _shortest_path_for_sign(delivery)

    return [pickup_lap, delivery_lap]


def _shortest_path_for_sign(sign: int) -> LapPlan:
    """Find the shortest path (fewest signs visited) that includes a given sign."""
    best: LapPlan | None = None
    best_count = float("inf")

    for (dp1, dp2), signs in _PATH_SIGNS.items():
        if sign in signs and len(signs) < best_count:
            best = LapPlan(dp1=dp1, dp2=dp2)
            best_count = len(signs)

    assert best is not None
    return best
