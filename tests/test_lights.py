"""The strip and the lamps: what each driving state shows, and the speed
test's police show on top of it."""

from lights import LightsController, StripEffect


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
    assert lights(speed_test=True, autonomous=False).compute().strip_effect is StripEffect.GREEN


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
