"""Tests for the mission planner."""

import pytest

from src.autonomous.route_table import Decision, SignAction
from src.autonomous.mission import MissionPlanner

I = Decision.INNER
O = Decision.OUTER


class TestOneLapMission:
    def test_1_to_2_decisions(self):
        mp = MissionPlanner(1, 2)
        assert mp.total_laps == 1
        assert mp.get_decision(1) == O
        assert mp.get_decision(2) == O

    def test_1_to_2_sign_visits(self):
        mp = MissionPlanner(1, 2)
        assert mp.record_sign_visit(1) == SignAction.PICKUP
        assert mp.record_sign_visit(2) == SignAction.DELIVERY
        assert mp.is_complete

    def test_signs_this_lap(self):
        mp = MissionPlanner(1, 3)
        assert mp.signs_this_lap() == [1, 3, 4]

    def test_ignore_non_mission_signs(self):
        mp = MissionPlanner(1, 2)
        assert mp.record_sign_visit(3) == SignAction.NONE
        assert mp.record_sign_visit(4) == SignAction.NONE
        assert not mp.is_complete


class TestTwoLapMission:
    def test_4_to_1_lap_structure(self):
        mp = MissionPlanner(4, 1)
        assert mp.total_laps == 2
        assert mp.current_lap == 0

    def test_4_to_1_decisions(self):
        mp = MissionPlanner(4, 1)
        # Lap 1: INNER only
        assert mp.get_decision(1) == I
        with pytest.raises(ValueError, match="not reachable"):
            mp.get_decision(2)
        # Advance to lap 2
        mp.advance_lap()
        assert mp.get_decision(1) == O
        assert mp.get_decision(2) == I

    def test_4_to_1_full_execution(self):
        mp = MissionPlanner(4, 1)
        # Lap 1: pickup at sign 4
        assert mp.record_sign_visit(4) == SignAction.PICKUP
        assert not mp.is_complete
        mp.advance_lap()
        # Lap 2: deliver at sign 1
        assert mp.record_sign_visit(1) == SignAction.DELIVERY
        assert mp.is_complete

    def test_cannot_deliver_before_pickup(self):
        mp = MissionPlanner(2, 1)
        # Sign 1 appears before sign 2 on the outer track, but delivery
        # should be NONE because pickup hasn't happened yet
        assert mp.record_sign_visit(1) == SignAction.NONE
        assert mp.record_sign_visit(2) == SignAction.PICKUP
        assert not mp.is_complete


class TestAdvanceLap:
    def test_cannot_advance_past_last_lap(self):
        mp = MissionPlanner(1, 2)  # single lap
        with pytest.raises(IndexError, match="No more laps"):
            mp.advance_lap()

    def test_advance_works_on_two_lap_route(self):
        mp = MissionPlanner(4, 1)
        mp.advance_lap()
        assert mp.current_lap == 1


class TestAllCombos:
    """Verify every (pickup, delivery) combo can complete successfully."""

    @pytest.mark.parametrize(
        "pickup,delivery",
        [(p, d) for p in range(1, 5) for d in range(1, 5) if p != d],
    )
    def test_mission_completes(self, pickup, delivery):
        mp = MissionPlanner(pickup, delivery)
        for lap_idx in range(mp.total_laps):
            if lap_idx > 0:
                mp.advance_lap()
            for sign in mp.signs_this_lap():
                mp.record_sign_visit(sign)
        assert mp.is_complete
