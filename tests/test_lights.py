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
    assert lit(DockScene.WAITING, estopped=True).projector
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


def lights(**update):
    controller = LightsController()
    controller.update(0.5, **{"estopped": False, "autonomous": True, **update})
    return controller


def test_autonomous_driving_is_cyan_outside_the_speed_test():
    assert lights().compute().strip_effect is StripEffect.CYAN


def test_speed_test_drives_with_the_police_strip():
    assert lights(speed_test=True).compute().strip_effect is StripEffect.POLICE


def test_speed_test_keeps_the_stop_and_manual_colours():
    assert lights(speed_test=True, estopped=True).compute().strip_effect is StripEffect.RED_PULSE
    assert lights(speed_test=True, obstacle_stopped=True).compute().strip_effect is StripEffect.RED
    assert lights(speed_test=True, autonomous=False).compute().strip_effect is StripEffect.YELLOW_PULSE


def test_operator_strip_choice_wins_over_the_police_strip():
    controller = lights(speed_test=True)
    controller.strip_effect = StripEffect.RAINBOW
    assert controller.compute().strip_effect is StripEffect.RAINBOW


def test_speed_test_turns_every_lamp_on():
    output = lights(speed_test=True, estopped=True).compute()
    assert output.low_beam and output.logo and output.projector
    assert output.front_left_drl and output.front_right_drl
    assert output.back_left_position and output.back_right_position


def test_lamps_follow_the_operator_outside_the_speed_test():
    output = lights().compute()
    assert not (output.low_beam or output.logo or output.projector or output.front_left_drl)
