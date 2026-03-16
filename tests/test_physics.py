"""Unit tests for the differential-drive physics engine."""

import math

from src.simulation.physics import DifferentialDrivePhysics


class TestStationary:
    """No power applied means no movement."""

    def test_stationary(self):
        physics = DifferentialDrivePhysics()
        physics.update(0, 0, 1.0 / 60.0)
        state = physics.get_state()
        assert state.x == 0.0
        assert state.y == 0.0
        assert state.heading == 0.0
        assert state.speed == 0.0


class TestForward:
    """Equal positive power on both wheels drives forward along heading."""

    def test_forward(self):
        physics = DifferentialDrivePhysics()
        # Drive forward for many steps
        for _ in range(300):
            physics.update(200, 200, 1.0 / 60.0)
        state = physics.get_state()
        # Should have moved positively in x (heading 0 = east)
        assert state.x > 0.0
        # y should stay near zero (straight line)
        assert abs(state.y) < 1e-6
        assert state.speed > 0.0


class TestTurn:
    """Unequal power causes heading to change."""

    def test_turn(self):
        physics = DifferentialDrivePhysics()
        # Right wheel faster → turns left (positive heading change)
        for _ in range(100):
            physics.update(50, 200, 1.0 / 60.0)
        state = physics.get_state()
        assert state.heading != 0.0


class TestFriction:
    """After removing power, friction causes the vehicle to decelerate."""

    def test_friction(self):
        physics = DifferentialDrivePhysics()
        # Build up speed
        for _ in range(120):
            physics.update(255, 255, 1.0 / 60.0)
        speed_before = physics.get_state().speed
        assert speed_before > 0.0

        # Cut power and coast
        for _ in range(300):
            physics.update(0, 0, 1.0 / 60.0)
        speed_after = physics.get_state().speed
        assert speed_after < speed_before
