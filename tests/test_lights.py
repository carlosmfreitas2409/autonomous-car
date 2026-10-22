"""The dock scenes on the lamps, and what outranks them."""

import pytest

from lights import DockScene, LightsController, StripEffect, TurnSignal


def lit(scene=None, **state):
    lights = LightsController()
    lights.update(0.0, estopped=state.pop("estopped", False), autonomous=True, scene=scene, **state)
    return lights.compute()


@pytest.mark.parametrize("scene,strip,projector", [
    (DockScene.WAITING, StripEffect.BLUE_BREATHE, True),
    (DockScene.CHECKING, StripEffect.CYAN_SWEEP, False),
    (DockScene.DELIVERED, StripEffect.GREEN, True),
    (DockScene.PARKED, StripEffect.BLUE_BREATHE, True),
])
def test_each_scene_has_one_look(scene, strip, projector):
    output = lit(scene)
    assert output.strip_effect is strip
    assert output.projector is projector and output.logo is projector


def test_no_scene_is_the_car_as_before():
    output = lit()
    assert output.strip_effect is StripEffect.CYAN
    assert not output.projector and not output.logo


def test_safety_outranks_the_scene():
    assert lit(DockScene.WAITING, estopped=True).strip_effect is StripEffect.RED_PULSE
    assert lit(DockScene.WAITING, estopped=True).projector    # the attendant is still there
    lights = LightsController()
    lights.signal_override = TurnSignal.LEFT
    lights.update(0.0, estopped=False, autonomous=True, obstacle_stopped=True, scene=DockScene.CHECKING)
    output = lights.compute()
    assert output.strip_effect is StripEffect.RED
    assert output.front_left_signal and output.front_right_signal


def test_the_operator_outranks_the_scene():
    lights = LightsController()
    lights.strip_effect = StripEffect.RAINBOW
    lights.projector = True
    lights.update(0.0, estopped=False, autonomous=True, scene=DockScene.CHECKING)
    output = lights.compute()
    assert output.strip_effect is StripEffect.RAINBOW
    assert output.projector
