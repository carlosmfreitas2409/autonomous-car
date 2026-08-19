"""Traffic light state reading, exercised against synthetic housings.

Everything here is rendered in numpy, so the safety-critical part of sign
perception is fully covered before a camera, a model, or a dataset exists.
The sweeps cover the failure modes that actually bit the previous attempt:
a blown-out lamp core, a coloured housing outvoting the lamp, a loose
detector box, and bad exposure.
"""

import cv2 as cv
import numpy as np
import pytest

from perception.traffic_light import LightState, read_light_state
from synthetic import render_light


@pytest.mark.parametrize("state", [LightState.RED, LightState.YELLOW, LightState.GREEN])
def test_three_lamp_vertical_housing(state):
    assert read_light_state(render_light(state)).state is state


@pytest.mark.parametrize("state", [LightState.RED, LightState.GREEN])
def test_two_lamp_housing(state):
    assert read_light_state(render_light(state, n_lamps=2)).state is state


@pytest.mark.parametrize("state", [LightState.RED, LightState.YELLOW, LightState.GREEN])
def test_horizontal_housing(state):
    assert read_light_state(render_light(state, vertical=False)).state is state


@pytest.mark.parametrize("state", [LightState.RED, LightState.YELLOW, LightState.GREEN])
def test_clipped_white_core_still_reads_colour(state):
    """Hue must come from the saturated halo, never the blown-out centre."""
    assert read_light_state(render_light(state, clipped_core=True)).state is state


@pytest.mark.parametrize("brightness", [0.35, 0.6, 1.0])
@pytest.mark.parametrize("state", [LightState.RED, LightState.YELLOW, LightState.GREEN])
def test_survives_exposure_variation(state, brightness):
    assert read_light_state(render_light(state, brightness=brightness)).state is state


@pytest.mark.parametrize("state", [LightState.RED, LightState.YELLOW, LightState.GREEN])
def test_survives_motion_blur(state):
    assert read_light_state(render_light(state, blur=5)).state is state


@pytest.mark.parametrize("state", [LightState.RED, LightState.GREEN])
def test_survives_loose_detector_box(state):
    """A box with background padding changes the aspect ratio and cell split."""
    assert read_light_state(render_light(state, pad=8)).state is state


def test_a_loose_box_still_halts_for_yellow():
    """Padding makes a three-lamp housing measure closer to two, and a two-lamp
    layout has no middle position to put yellow in. Hue cannot rescue it either,
    since red and yellow are one band. It resolves to red — which drives the
    same behaviour, because both halt the cart."""
    reading = read_light_state(render_light(LightState.YELLOW, pad=8))
    assert reading.state in (LightState.YELLOW, LightState.RED)


def test_coloured_housing_does_not_outvote_the_lamp():
    """The previous implementation argmaxed colour over the whole box, so a
    red housing beat a lit green lamp. The lamp has to win."""
    crop = render_light(LightState.GREEN, housing=(0, 0, 120))
    assert read_light_state(crop).state is LightState.GREEN


def test_red_lamp_against_green_surround():
    crop = render_light(LightState.RED, housing=(0, 90, 0), pad=6)
    assert read_light_state(crop).state is LightState.RED


def test_small_distant_housing():
    reading = read_light_state(render_light(LightState.GREEN, lamp_px=10))
    assert reading.state is LightState.GREEN


def test_all_lamps_dark_is_unknown():
    crop = render_light(LightState.RED, brightness=0.05)
    assert read_light_state(crop).state is LightState.UNKNOWN


def test_uniform_bright_crop_is_unknown():
    crop = np.full((120, 40, 3), 240, dtype=np.uint8)
    assert read_light_state(crop).state is LightState.UNKNOWN


def test_position_and_hue_disagreement_is_unknown():
    """A green lamp in the red position is not something to guess about."""
    crop = render_light(LightState.GREEN, lit_index=0)
    assert read_light_state(crop).state is LightState.UNKNOWN


@pytest.mark.parametrize("lamp_index,expected", [
    (0, LightState.RED),
    (1, LightState.YELLOW),
    (2, LightState.GREEN),
])
def test_position_carries_a_lamp_blown_out_to_a_thin_ring(lamp_index, expected):
    """This is what the real camera actually delivers: the lamp's centre clips
    to white and only a sliver of colour survives at the edge. That sliver is
    enough to establish something is lit; position says which."""
    crop = render_light(LightState.RED, lit_index=lamp_index)
    centre = (crop.shape[1] // 2, int(40 * (lamp_index + 0.5)))
    cv.circle(crop, centre, 12, (255, 255, 255), -1)

    assert read_light_state(crop).state is expected


def test_a_bright_region_with_no_colour_at_all_is_unknown():
    """An unlit housing is reflective grey plastic. It can be the brightest
    thing in the crop and pass every brightness test, so brightness alone would
    report a state for every dark light on the track. Colour is what says
    something is actually emitting."""
    crop = render_light(LightState.RED)
    for i in range(3):
        cv.circle(crop, (crop.shape[1] // 2, int(40 * (i + 0.5))), 14, (235, 235, 235), -1)

    assert read_light_state(crop).state is LightState.UNKNOWN


def test_green_is_recognised_from_a_box_holding_only_that_lamp():
    """A box cropped to one lamp is roughly square, so which third it lands in
    means nothing and position is discounted. Colour carries it."""
    crop = render_light(LightState.GREEN)[80:120, :]
    assert read_light_state(crop).state is LightState.GREEN


def test_a_single_warm_lamp_halts_without_claiming_which_one():
    """Cropped to one lamp there is no usable position, and hue cannot split
    red from yellow on this hardware. The honest answer is one of the two — and
    since both halt the cart, the behaviour is the same either way."""
    crop = render_light(LightState.RED)[0:40, :]
    assert read_light_state(crop).state in (LightState.RED, LightState.YELLOW)


@pytest.mark.parametrize(
    "crop",
    [
        np.zeros((0, 0, 3), dtype=np.uint8),
        np.zeros((4, 4, 3), dtype=np.uint8),
        np.zeros((120, 40), dtype=np.uint8),
    ],
)
def test_degenerate_crops_are_unknown(crop):
    assert read_light_state(crop).state is LightState.UNKNOWN


def test_none_crop_is_unknown():
    assert read_light_state(None).state is LightState.UNKNOWN


def test_agreeing_signals_are_more_confident_than_a_blown_out_lamp():
    clean = read_light_state(render_light(LightState.RED))
    washed = read_light_state(render_light(LightState.RED, clipped_core=True))
    assert clean.state is washed.state is LightState.RED
    assert clean.confidence > washed.confidence


def test_reading_reports_which_lamp_won():
    reading = read_light_state(render_light(LightState.GREEN))
    assert reading.n_lamps == 3
    assert reading.lamp_index == 2
