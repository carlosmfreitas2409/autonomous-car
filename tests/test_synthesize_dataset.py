"""The compositor's stage-fixture distractors.

The fixtures are the one negative taken from the venue itself rather than
drawn, and they are only worth anything if they load and actually land in the
frame - a silently empty folder would retrain the same spotlight confusion.
"""

import random
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))

from synthesize_dataset import draw_distractors, load_fixtures  # noqa: E402


def test_every_fixture_loads_with_an_alpha_channel():
    fixtures = load_fixtures(ROOT / "assets" / "distractor_cutouts")
    assert len(fixtures) == 12
    assert all(f.ndim == 3 and f.shape[2] == 4 for f in fixtures)


def test_fixtures_are_pasted_when_given():
    fixtures = load_fixtures(ROOT / "assets" / "distractor_cutouts")
    changed = 0
    for seed in range(20):
        with_fixtures = np.full((720, 1280, 3), 30, np.uint8)
        without = with_fixtures.copy()
        draw_distractors(with_fixtures, random.Random(seed), fixtures)
        draw_distractors(without, random.Random(seed))
        changed += not np.array_equal(with_fixtures, without)
    assert changed > 0


def test_no_fixtures_keeps_the_old_distractors():
    """Called without fixtures - as tools/probe_signs.py does - the rng stream
    and so every frame is exactly what it was before fixtures existed."""
    a = np.full((720, 1280, 3), 30, np.uint8)
    b = a.copy()
    draw_distractors(a, random.Random(3))
    draw_distractors(b, random.Random(3), ())
    assert np.array_equal(a, b)
