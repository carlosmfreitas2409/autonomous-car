"""Unit tests for the bicycle model physics engine."""

from src.simulation.physics import BicyclePhysics


class TestStationary:
    def test_stationary(self):
        physics = BicyclePhysics()
        physics.update(0, 0, 1.0 / 60.0)
        state = physics.get_state()
        assert state.x == 0.0
        assert state.y == 0.0
        assert state.heading == 0.0
        assert state.speed == 0.0


class TestForward:
    def test_forward(self):
        physics = BicyclePhysics()
        for _ in range(300):
            physics.update(0.8, 0.0, 1.0 / 60.0)
        state = physics.get_state()
        assert state.x > 0.0
        assert abs(state.y) < 1e-6
        assert state.speed > 0.0


class TestTurn:
    def test_turn(self):
        """Steering input causes heading to change."""
        physics = BicyclePhysics()
        for _ in range(100):
            physics.update(0.5, 0.5, 1.0 / 60.0)
        state = physics.get_state()
        assert state.heading != 0.0

    def test_no_spin_in_place(self):
        """With zero throttle, steering alone should not move the car."""
        physics = BicyclePhysics()
        for _ in range(100):
            physics.update(0.0, 1.0, 1.0 / 60.0)
        state = physics.get_state()
        assert abs(state.x) < 1e-6
        assert abs(state.y) < 1e-6


class TestFriction:
    def test_friction(self):
        physics = BicyclePhysics()
        for _ in range(120):
            physics.update(1.0, 0.0, 1.0 / 60.0)
        speed_before = physics.get_state().speed
        assert speed_before > 0.0

        for _ in range(300):
            physics.update(0.0, 0.0, 1.0 / 60.0)
        speed_after = physics.get_state().speed
        assert speed_after < speed_before
