from cargo import _OPEN_S, CargoReading, ManualOpen


def cargo(lid=True):
    return CargoReading(grams=0, stable=True, lid_closed=lid, unlocked=False, fault=False)


def test_opens_until_the_lid_is_lifted():
    opener = ManualOpen()
    assert not opener.update(0.0, cargo(), stopped=True)
    opener.request(0.0)
    assert opener.update(1.0, cargo(), stopped=True)
    assert not opener.update(1.1, cargo(lid=False), stopped=True)
    assert not opener.update(1.2, cargo(), stopped=True)


def test_gives_up_after_a_while():
    opener = ManualOpen()
    opener.request(0.0)
    assert opener.update(_OPEN_S - 0.1, cargo(), stopped=True)
    assert not opener.update(_OPEN_S, cargo(), stopped=True)


def test_never_once_the_car_may_move():
    opener = ManualOpen()
    opener.request(0.0)
    assert not opener.update(0.1, cargo(), stopped=False)
    assert not opener.update(0.2, cargo(), stopped=True)


def test_nothing_to_open_without_a_cargo_node():
    opener = ManualOpen()
    opener.request(0.0)
    assert not opener.update(0.1, None, stopped=True)
