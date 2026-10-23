"""E-stop, the latched manual mode and the dashboard's drive lease."""

from supervisor import LEASE_S, DriveInput, Mode, Source, Supervisor


def _dashboard_manual(now=0.0):
    s = Supervisor()
    s.request_estop(False, Source.DASHBOARD)
    s.request_mode(Mode.MANUAL, Source.DASHBOARD)
    return s


def test_boots_estopped_and_autonomous_like_the_gui_did():
    s = Supervisor()
    assert s.estopped and s.autonomous


def test_gui_and_pad_toggles_behave_as_before():
    s = Supervisor()
    s.toggle_estop(Source.PAD)
    assert not s.estopped
    s.toggle_estop(Source.GUI)
    assert s.estopped
    s.toggle_mode(Source.GUI)
    assert s.mode is Mode.MANUAL and s.driver is Source.PAD
    s.toggle_mode(Source.PAD)
    assert s.autonomous


def test_estop_request_is_a_target_state_not_a_toggle():
    s = Supervisor()
    s.request_estop(True, Source.DASHBOARD)
    s.request_estop(True, Source.DASHBOARD)
    assert s.estopped
    s.request_estop(False, Source.DASHBOARD)
    s.request_estop(False, Source.DASHBOARD)
    assert not s.estopped


def test_pad_driver_passes_the_pad_through_untouched():
    s = Supervisor()
    s.toggle_mode(Source.PAD)
    out = s.manual((0.3, 0.7), None, 10.0)
    assert (out.steering, out.throttle) == (0.3, 0.7)
    assert not out.lease_lost


def test_dashboard_manual_needs_neutral_before_it_drives():
    s = _dashboard_manual()
    held = s.manual((0.0, 0.0), DriveInput(0.2, 0.8, 1.0), 1.0)
    assert held.throttle == 0.0 and held.waiting_neutral
    s.manual((0.0, 0.0), DriveInput(0.2, 0.0, 1.03), 1.03)
    out = s.manual((0.0, 0.0), DriveInput(0.2, 0.8, 1.06), 1.06)
    assert out.throttle == 0.8 and out.steering == 0.2


def test_releasing_the_stick_stops_but_stays_manual():
    s = _dashboard_manual()
    s.manual((0.0, 0.0), DriveInput(0.0, 0.0, 1.0), 1.0)
    s.manual((0.0, 0.0), DriveInput(0.0, 0.6, 1.1), 1.1)
    out = s.manual((0.0, 0.0), DriveInput(0.0, 0.0, 1.2), 1.2)
    assert out.throttle == 0.0
    assert s.mode is Mode.MANUAL


def test_lost_link_stops_holds_steering_and_never_returns_to_autonomous():
    s = _dashboard_manual()
    s.manual((0.0, 0.0), DriveInput(0.0, 0.0, 1.0), 1.0)
    s.manual((0.0, 0.0), DriveInput(-0.4, 0.6, 1.1), 1.1)
    out = s.manual((0.0, 0.0), DriveInput(-0.4, 0.6, 1.1), 1.1 + LEASE_S + 0.01)
    assert out.lease_lost and out.throttle == 0.0 and out.steering == -0.4
    assert s.mode is Mode.MANUAL and s.lease_lost
    # Back with the trigger still held: no lurch until it has been released.
    back = s.manual((0.0, 0.0), DriveInput(-0.4, 0.6, 5.0), 5.0)
    assert back.throttle == 0.0 and back.waiting_neutral and not back.lease_lost
    s.manual((0.0, 0.0), DriveInput(0.0, 0.0, 5.03), 5.03)
    assert s.manual((0.0, 0.0), DriveInput(0.0, 0.5, 5.06), 5.06).throttle == 0.5


def test_no_drive_message_at_all_is_a_lost_lease():
    s = _dashboard_manual()
    out = s.manual((0.0, 0.0), None, 3.0)
    assert out.lease_lost and out.throttle == 0.0


def test_the_pad_in_hand_takes_over_from_the_dashboard():
    s = _dashboard_manual()
    s.manual((0.0, 0.0), DriveInput(0.0, 0.0, 1.0), 1.0)
    out = s.manual((0.5, 0.0), DriveInput(0.0, 0.9, 1.05), 1.05)
    assert s.driver is Source.PAD and (out.steering, out.throttle) == (0.5, 0.0)


def test_resume_from_estop_rearms_the_neutral_check():
    s = _dashboard_manual()
    s.manual((0.0, 0.0), DriveInput(0.0, 0.0, 1.0), 1.0)
    s.request_estop(True, Source.DASHBOARD)
    s.request_estop(False, Source.DASHBOARD)
    assert s.manual((0.0, 0.0), DriveInput(0.0, 0.7, 2.0), 2.0).throttle == 0.0


def test_only_an_explicit_request_returns_to_autonomous():
    s = _dashboard_manual()
    for t in (1.0, 2.0, 9.0):
        s.manual((0.0, 0.0), None, t)
    assert s.mode is Mode.MANUAL
    s.request_mode(Mode.AUTONOMOUS, Source.DASHBOARD)
    assert s.autonomous and not s.lease_lost
