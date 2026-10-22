"""Dock prompts: every announcement has words, and speech never blocks or piles up."""


import pytest

import voice
from input.mission import Announcement
from voice import TEXT, Voice

ANNOUNCED = {"docked_pickup", "docked_delivery", "remind_pickup", "remind_delivery",
             "released", "blocked", "delivered",
             "docked_pickup_lid", "docked_delivery_lid", "remind_pickup_lid", "remind_delivery_lid",
             "close_lid", "empty", "still_loaded"}


def test_every_announcement_has_words():
    assert set(TEXT) == ANNOUNCED
    for key in ANNOUNCED:
        assert TEXT[key].format(letter="A")


class FakeSay:
    """Stands in for `say`: busy until `finish()`."""
    started: list = []

    def __init__(self, command, **_):
        self.command = command
        self.busy = True
        FakeSay.started.append(self)

    def poll(self):
        return None if self.busy else 0

    def terminate(self):
        self.busy = False


@pytest.fixture
def say(monkeypatch):
    FakeSay.started = []
    monkeypatch.setattr(voice.shutil, "which", lambda _: "/usr/bin/say")
    monkeypatch.setattr(voice, "_pick_voice", lambda _: "Luciana")
    monkeypatch.setattr(voice.subprocess, "Popen", FakeSay)
    return FakeSay.started


def test_a_prompt_waits_for_the_one_playing_and_only_the_latest_waits(say):
    speaker = Voice()
    speaker.announce(Announcement("docked_pickup", "A"))
    speaker.announce(Announcement("released", "A"))
    speaker.announce(Announcement("blocked", "A"))
    assert len(say) == 1 and say[0].command == ["/usr/bin/say", "-v", "Luciana", TEXT["docked_pickup"].format(letter="A")]
    say[0].busy = False
    speaker.update()
    assert len(say) == 2 and say[1].command[-1] == TEXT["blocked"]


def test_speaking_until_the_last_prompt_ends(say):
    speaker = Voice()
    assert not speaker.speaking
    speaker.announce(Announcement("docked_pickup", "A"))
    speaker.announce(Announcement("released", "A"))
    assert speaker.speaking
    say[0].busy = False
    assert speaker.speaking                                 # "released" still waits its turn
    speaker.update()
    say[1].busy = False
    assert not speaker.speaking


def test_no_say_is_silent(monkeypatch):
    monkeypatch.setattr(voice.shutil, "which", lambda _: None)
    monkeypatch.setattr(voice.subprocess, "Popen", lambda *a, **k: pytest.fail("spoke"))
    speaker = Voice()
    speaker.announce(Announcement("delivered", "C"))
    speaker.update()
    assert not speaker.speaking


def test_no_voice_flag_is_silent(say):
    speaker = Voice(enabled=False)
    speaker.announce(Announcement("delivered", "C"))
    assert say == []


def test_system_voice_names_no_voice(say):
    speaker = Voice(voice="system")
    speaker.announce(Announcement("blocked", "A"))
    assert say[0].command == ["/usr/bin/say", TEXT["blocked"]]
