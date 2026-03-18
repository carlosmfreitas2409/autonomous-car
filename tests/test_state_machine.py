"""Tests for the autonomous driving state machine."""

from src.autonomous.state_machine import AutonomousState, AutonomousStateMachine


class TestStateTransitions:
    def test_initial_state(self):
        sm = AutonomousStateMachine()
        assert sm.state == AutonomousState.WAITING_FOR_START

    def test_start(self):
        sm = AutonomousStateMachine()
        sm.start()
        assert sm.state == AutonomousState.NAVIGATING

    def test_approach_sign(self):
        sm = AutonomousStateMachine()
        sm.start()
        sm.approach_sign()
        assert sm.state == AutonomousState.APPROACHING_SIGN

    def test_stop_at_sign(self):
        sm = AutonomousStateMachine()
        sm.start()
        sm.approach_sign()
        sm.stop_at_sign()
        assert sm.state == AutonomousState.STOPPED_AT_SIGN

    def test_resume(self):
        sm = AutonomousStateMachine()
        sm.start()
        sm.approach_sign()
        sm.stop_at_sign()
        sm.resume()
        assert sm.state == AutonomousState.NAVIGATING

    def test_complete(self):
        sm = AutonomousStateMachine()
        sm.start()
        sm.complete()
        assert sm.state == AutonomousState.ROUTE_COMPLETE

    def test_full_cycle(self):
        sm = AutonomousStateMachine()
        sm.start()
        # First stop (pickup)
        sm.approach_sign()
        sm.stop_at_sign()
        sm.resume()
        # Second stop (delivery)
        sm.approach_sign()
        sm.stop_at_sign()
        sm.resume()
        sm.complete()
        assert sm.state == AutonomousState.ROUTE_COMPLETE


class TestInvalidTransitions:
    def test_approach_from_waiting(self):
        sm = AutonomousStateMachine()
        sm.approach_sign()
        assert sm.state == AutonomousState.WAITING_FOR_START

    def test_stop_from_navigating(self):
        sm = AutonomousStateMachine()
        sm.start()
        sm.stop_at_sign()
        assert sm.state == AutonomousState.NAVIGATING

    def test_resume_from_navigating(self):
        sm = AutonomousStateMachine()
        sm.start()
        sm.resume()
        assert sm.state == AutonomousState.NAVIGATING


class TestStopTimer:
    def test_timer_resets_on_stop(self):
        sm = AutonomousStateMachine()
        sm.start()
        sm.approach_sign()
        sm.stop_at_sign()
        assert sm.stop_elapsed == 0.0

    def test_timer_accumulates(self):
        sm = AutonomousStateMachine()
        sm.start()
        sm.approach_sign()
        sm.stop_at_sign()
        assert not sm.update_stop_timer(1.0, 4.0)
        assert not sm.update_stop_timer(1.0, 4.0)
        assert not sm.update_stop_timer(1.0, 4.0)
        assert sm.update_stop_timer(1.0, 4.0)

    def test_timer_exact_boundary(self):
        sm = AutonomousStateMachine()
        sm.start()
        sm.approach_sign()
        sm.stop_at_sign()
        assert sm.update_stop_timer(4.0, 4.0)

    def test_timer_ignored_in_wrong_state(self):
        sm = AutonomousStateMachine()
        sm.start()
        assert not sm.update_stop_timer(10.0, 4.0)
