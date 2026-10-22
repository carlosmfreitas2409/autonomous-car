"""The master's serial lines: the cargo node's `C` among the sonar's `S`, and
the cargo node counted as gone once its lines stop."""

import pytest

import arduino
from arduino import Arduino
from cargo import CargoReading


class FakePort:
    def __init__(self):
        self.lines: list[str] = []
        self.written: list[str] = []

    def read_lines(self, limit: int = 40) -> list[str]:
        lines, self.lines = self.lines[:limit], self.lines[limit:]
        return lines

    def write(self, message: str):
        self.written.append(message)


@pytest.fixture
def master(monkeypatch):
    clock = {"now": 100.0}
    monkeypatch.setattr(arduino.time, "monotonic", lambda: clock["now"])
    board = Arduino()
    board._master = FakePort()
    return board, clock


def test_cargo_lines_are_read_among_the_sonar(master):
    board, clock = master
    board._master.lines = ["S C 120 255 120 200", "C 412 1 1 0 0", "T 500 500 0 0 R 3", "S C 90 255 90 200"]
    sonar = board.read_sonar()
    assert sonar.nearest_cm == 90
    assert board.read_cargo(clock["now"]) == CargoReading(grams=412, stable=True, lid_closed=True,
                                                          unlocked=False, fault=False)


def test_negative_grams_and_flags(master):
    board, clock = master
    board._master.lines = ["C -3 0 0 1 1"]
    assert board.read_sonar() is None
    assert board.read_cargo(clock["now"]) == CargoReading(grams=-3, stable=False, lid_closed=False,
                                                          unlocked=True, fault=True)


@pytest.mark.parametrize("line", ["C 412 1 1 0", "C x 1 1 0 0", "C 412 1 1 0 0 9"])
def test_malformed_cargo_line_is_ignored(master, line):
    board, clock = master
    board._master.lines = [line]
    board.read_sonar()
    assert board.read_cargo(clock["now"]) is None


def test_no_cargo_node_and_a_silent_one_read_none(master):
    board, clock = master
    assert board.read_cargo(clock["now"]) is None
    board._master.lines = ["C 0 1 1 0 0"]
    board.read_sonar()
    assert board.read_cargo(clock["now"] + 0.9) is not None
    assert board.read_cargo(clock["now"] + 1.1) is None


def test_unlock_is_a_k_line(master):
    board, _ = master
    board.send_cargo(True)
    board.send_cargo(False)
    assert board._master.written == ["K 1", "K 0"]
