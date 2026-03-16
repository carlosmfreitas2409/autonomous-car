"""Unit tests for the safety subsystem (EStopManager & CommandWatchdog)."""

import time

from src.safety.emergency_stop import EStopManager
from src.safety.watchdog import CommandWatchdog


# --- EStopManager ---

def test_estop_initial_state():
    """EStopManager starts in RUNNING (not active)."""
    estop = EStopManager()
    assert not estop.is_active()


def test_estop_activate():
    """activate() transitions to STOPPED (is_active True)."""
    estop = EStopManager()
    estop.activate()
    assert estop.is_active()


def test_estop_reset():
    """After activate, reset() transitions back to RUNNING."""
    estop = EStopManager()
    estop.activate()
    assert estop.is_active()
    estop.reset()
    assert not estop.is_active()


# --- CommandWatchdog ---

def test_watchdog_not_expired_after_feed():
    """Immediately after feed(), the watchdog should not be expired."""
    wd = CommandWatchdog(timeout=1.0)
    wd.feed()
    assert not wd.is_expired()


def test_watchdog_expires():
    """If not fed for longer than timeout, the watchdog expires."""
    wd = CommandWatchdog(timeout=0.05)
    time.sleep(0.1)
    assert wd.is_expired()
