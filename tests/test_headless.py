from pathlib import Path

import dearpygui.dearpygui as dpg
import pytest

import gui
from config import Config
from headless import HeadlessGUI
from gui import TunerGUI

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture
def tuner(monkeypatch):
    monkeypatch.chdir(ROOT)
    for name in ("create_viewport", "setup_dearpygui", "show_viewport"):
        monkeypatch.setattr(gui.dpg, name, lambda *args, **kwargs: None)
    tuner = TunerGUI(Config(), 960, 540)
    yield tuner
    dpg.destroy_context()


def test_params_are_the_tuners_bit_for_bit(tuner):
    headless = HeadlessGUI(Config()).read_params()
    expected = tuner.read_params()
    assert headless == expected
    for name, value in vars(expected).items():
        assert type(getattr(headless, name)) is type(value), name


def test_answers_every_call_the_loop_makes_of_the_tuner():
    loop_calls = {line.split("gui.")[1].split("(")[0]
                  for line in (ROOT / "src" / "main.py").read_text().splitlines() if "gui." in line}
    assert loop_calls
    assert all(callable(getattr(HeadlessGUI, name, None)) for name in loop_calls), loop_calls


def test_asks_for_nothing():
    headless = HeadlessGUI(Config())
    assert not headless.poll_estop_toggle() and not headless.poll_mode_toggle()
    assert headless.poll_tuning_request() == {}
    assert headless.poll_challenge_request() is None and headless.poll_path_request() is None
    assert headless.poll_mission_request() is None and headless.poll_dock_request() is None
    assert not headless.poll_open_request()
