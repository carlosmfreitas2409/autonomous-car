"""Tests for the pre-computed route table — validates all 12 (pickup, delivery) combos."""

import pytest

from src.autonomous.route_table import (
    Decision,
    LapPlan,
    compute_route,
    signs_for_path,
)

I = Decision.INNER
O = Decision.OUTER


class TestSignsForPath:
    def test_inner_only(self):
        assert signs_for_path(I) == [4]

    def test_outer_outer(self):
        assert signs_for_path(O, O) == [1, 2, 3, 4]

    def test_outer_inner(self):
        assert signs_for_path(O, I) == [1, 3, 4]


class TestOneLapRoutes:
    """Pickup comes before delivery in visit order — single lap."""

    def test_1_to_2(self):
        route = compute_route(1, 2)
        assert route == [LapPlan(dp1=O, dp2=O)]

    def test_1_to_3(self):
        route = compute_route(1, 3)
        assert route == [LapPlan(dp1=O, dp2=I)]

    def test_1_to_4(self):
        route = compute_route(1, 4)
        assert route == [LapPlan(dp1=O, dp2=I)]

    def test_2_to_3(self):
        route = compute_route(2, 3)
        assert route == [LapPlan(dp1=O, dp2=O)]

    def test_2_to_4(self):
        route = compute_route(2, 4)
        assert route == [LapPlan(dp1=O, dp2=O)]

    def test_3_to_4(self):
        route = compute_route(3, 4)
        assert route == [LapPlan(dp1=O, dp2=I)]


class TestTwoLapRoutes:
    """Pickup comes after delivery in clockwise order — two laps needed."""

    def test_2_to_1(self):
        route = compute_route(2, 1)
        assert len(route) == 2
        # Lap 1: need OO to reach sign 2
        assert route[0] == LapPlan(dp1=O, dp2=O)
        # Lap 2: OI is shortest path with sign 1
        assert route[1] == LapPlan(dp1=O, dp2=I)

    def test_3_to_1(self):
        route = compute_route(3, 1)
        assert len(route) == 2
        # Lap 1: OI is shortest path with sign 3
        assert route[0] == LapPlan(dp1=O, dp2=I)
        # Lap 2: OI is shortest path with sign 1
        assert route[1] == LapPlan(dp1=O, dp2=I)

    def test_3_to_2(self):
        route = compute_route(3, 2)
        assert len(route) == 2
        # Lap 1: OI is shortest path with sign 3
        assert route[0] == LapPlan(dp1=O, dp2=I)
        # Lap 2: need OO to reach sign 2
        assert route[1] == LapPlan(dp1=O, dp2=O)

    def test_4_to_1(self):
        route = compute_route(4, 1)
        assert len(route) == 2
        # Lap 1: I is shortest path with sign 4
        assert route[0] == LapPlan(dp1=I, dp2=None)
        # Lap 2: OI is shortest path with sign 1
        assert route[1] == LapPlan(dp1=O, dp2=I)

    def test_4_to_2(self):
        route = compute_route(4, 2)
        assert len(route) == 2
        # Lap 1: I is shortest path with sign 4
        assert route[0] == LapPlan(dp1=I, dp2=None)
        # Lap 2: need OO to reach sign 2
        assert route[1] == LapPlan(dp1=O, dp2=O)

    def test_4_to_3(self):
        route = compute_route(4, 3)
        assert len(route) == 2
        # Lap 1: I is shortest path with sign 4
        assert route[0] == LapPlan(dp1=I, dp2=None)
        # Lap 2: OI is shortest path with sign 3
        assert route[1] == LapPlan(dp1=O, dp2=I)


class TestPickupBeforeDelivery:
    """Verify that in every route, the pickup sign is visited before delivery."""

    @pytest.mark.parametrize(
        "pickup,delivery",
        [(p, d) for p in range(1, 5) for d in range(1, 5) if p != d],
    )
    def test_pickup_before_delivery(self, pickup, delivery):
        route = compute_route(pickup, delivery)
        visited = []
        for lap in route:
            visited.extend(signs_for_path(lap.dp1, lap.dp2))

        # Find first occurrence of pickup and delivery
        pickup_idx = visited.index(pickup)
        # delivery may appear before pickup in visited list (from an earlier lap),
        # but the FIRST occurrence after pickup is what matters
        delivery_indices = [i for i, s in enumerate(visited) if s == delivery and i > pickup_idx]
        assert len(delivery_indices) > 0, (
            f"Delivery sign {delivery} not found after pickup sign {pickup} "
            f"in visit sequence {visited}"
        )


class TestEdgeCases:
    def test_same_sign_raises(self):
        with pytest.raises(ValueError, match="different"):
            compute_route(1, 1)

    def test_invalid_sign_raises(self):
        with pytest.raises(ValueError, match="Invalid sign"):
            compute_route(0, 1)

    def test_invalid_sign_high_raises(self):
        with pytest.raises(ValueError, match="Invalid sign"):
            compute_route(1, 5)
