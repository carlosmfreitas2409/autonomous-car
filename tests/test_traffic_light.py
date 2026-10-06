"""The light reader, against drawn crops - no camera, no model.

Every case here is one that has actually gone wrong: an unlit housing read as
RED, a yellow read as red because the box was sliced into thirds, a stage lamp
read as a light.
"""

import numpy as np
import pytest

from perception.traffic_light import LightState, find_cell_stack, read_light_state
from tests.synthetic import (blank_crop, delivery_sign_crop, flooded_crop, glare_crop,
                             lamp_pair_crop, traffic_light_crop, warm_room)


@pytest.mark.parametrize("state,expected", [
    ("red", LightState.RED),
    ("yellow", LightState.YELLOW),
    ("green", LightState.GREEN),
])
def test_reads_the_lit_lamp(state, expected):
    reading = read_light_state(traffic_light_crop(state))
    assert reading.state is expected
    assert reading.confidence > 0.4


def test_yellow_is_not_red():
    """The previous reader sliced the box into three bands, and a 2.3:1 housing
    rounds to two of them - so the middle lamp had nowhere to be and every
    yellow read as red."""
    assert read_light_state(traffic_light_crop("yellow")).state is LightState.YELLOW


def test_unlit_housing_is_unknown():
    reading = read_light_state(traffic_light_crop("off"))
    assert reading.state is LightState.UNKNOWN
    assert reading.cells is not None, "the stack is there; it is simply not lit"


def test_unlit_housing_under_warm_light_is_still_unknown():
    """M2: the venue's stage lighting gives grey covers real saturation, and a
    colour-fraction gate called five of six unlit housings RED."""
    reading = read_light_state(warm_room(traffic_light_crop("off")))
    assert reading.state is LightState.UNKNOWN


def test_lamp_pair_is_not_a_light():
    assert read_light_state(lamp_pair_crop()).state is LightState.UNKNOWN


def test_blank_crop_is_not_a_light():
    reading = read_light_state(blank_crop())
    assert reading.state is LightState.UNKNOWN
    assert reading.cells is None


def test_a_sign_is_not_a_light():
    assert read_light_state(delivery_sign_crop("A")).state is LightState.UNKNOWN


def test_tiny_crop_refuses_rather_than_guesses():
    reading = read_light_state(np.zeros((6, 3, 3), np.uint8))
    assert reading.state is LightState.UNKNOWN
    assert "small" in reading.reason


@pytest.mark.parametrize("radius", [12, 22, 34, 60])
def test_cells_are_found_at_every_range(radius):
    stack = find_cell_stack(traffic_light_crop("off", radius=radius))
    assert stack is not None
    assert stack.found == 3
    # The fit is in the crop's own pixels, which are not the fixture's once the
    # reader has resampled - so check the shape of the answer, not its scale.
    spacing = stack.centres[1] - stack.centres[0]
    assert 1.8 < spacing / stack.radius < 2.6


def test_a_clipped_housing_still_reads():
    """At the stop line the housing runs off the top of the frame. That is the
    one moment the answer matters most."""
    crop = traffic_light_crop("red")
    clipped = crop[crop.shape[0] // 4:]
    reading = read_light_state(clipped)
    assert reading.state in (LightState.RED, LightState.UNKNOWN)
    assert reading.state is not LightState.GREEN


@pytest.mark.parametrize("state", ["red", "yellow", "green", "off"])
def test_never_swaps_red_and_green(state):
    """Whatever else goes wrong, stopping for a green light or driving through
    a red one must not."""
    reading = read_light_state(warm_room(traffic_light_crop(state), 0.5))
    if state == "red":
        assert reading.state is not LightState.GREEN
    elif state == "green":
        assert reading.state is not LightState.RED


def test_darkness_does_not_change_the_answer():
    """The measurement is a comparison inside one crop, so the hall's exposure
    should cancel out."""
    bright = traffic_light_crop("green", background=140)
    dark = np.clip(bright.astype(np.float32) * 0.45, 0, 255).astype(np.uint8)
    assert read_light_state(bright).state is read_light_state(dark).state


# --------------------------------------------------------------------------
# the blown-core fallback: what it may do, and what it may never do
# --------------------------------------------------------------------------

def test_a_lamp_that_erased_its_own_housing_still_reads_yellow():
    """Yellow is the middle lamp, so its flood swallows the covers on both
    sides and no three-cell stack survives. Measured on the approach clip, the
    main reader returns UNKNOWN for the whole yellow phase."""
    crop = flooded_crop("yellow")
    assert find_cell_stack(crop) is None, "fixture must defeat the cell finder"
    reading = read_light_state(crop)
    assert reading.state is LightState.YELLOW
    assert reading.cells is not None and reading.cells.assumed


def test_the_fallback_may_only_ever_say_yellow():
    """It reads position alone, and position alone is not trustworthy when the
    housing is clipped. Restricting it to yellow bounds the damage: the worst
    it can do is stop the car needlessly, never wave it through a red."""
    for position in (0.22, 0.84):          # where red and green would sit
        reading = read_light_state(glare_crop(position=position))
        assert reading.state is not LightState.RED
        assert reading.state is not LightState.GREEN


def test_the_fallback_reads_a_centred_core():
    assert read_light_state(glare_crop(position=0.53)).state is LightState.YELLOW


def test_the_fallback_needs_something_actually_clipping():
    """An unlit housing reaches 183 grey at the 97th percentile; a lit lamp
    reaches 254. Without that, there is nothing to read."""
    dim = np.clip(glare_crop(position=0.53).astype(np.float32) * 0.6, 0, 255).astype(np.uint8)
    assert read_light_state(dim).state is LightState.UNKNOWN


def test_the_fallback_refuses_a_box_that_is_all_lamp():
    """One cell is two radii of a housing 6.98 radii tall. A bright thing
    filling the box is not a traffic light."""
    reading = read_light_state(glare_crop(position=0.5, radius_fraction=0.42))
    assert reading.state is LightState.UNKNOWN


def test_the_fallback_never_fires_when_the_stack_is_readable():
    """It is a last resort, not a shortcut."""
    reading = read_light_state(traffic_light_crop("red"))
    assert reading.state is LightState.RED
    assert reading.cells is not None and not reading.cells.assumed


def test_a_red_glow_is_never_read_as_yellow():
    """lighting2/cam0_000360-370: a phantom cell in the glow above the housing
    made the red lamp the middle cell, and position alone called it YELLOW -
    which the policy lets roll through at crawl speed."""
    reading = read_light_state(traffic_light_crop("yellow", lamp_colour="red"))
    assert reading.state is LightState.RED


def test_a_colourless_halo_is_not_overruled_by_the_room():
    """A red-lit stage behind a lamp whose halo says nothing is evidence about
    the stage, not the lamp."""
    crop = traffic_light_crop("yellow", lamp_colour="white")
    crop[:12] = crop[-12:] = (40, 40, 220)
    assert read_light_state(crop).state is not LightState.RED
