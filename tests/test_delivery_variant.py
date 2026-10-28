import cv2 as cv
import pytest

from perception.delivery_variant import MIN_CROP_HEIGHT, read_delivery_variant, segment_face
from tests.synthetic import blank_crop, delivery_sign_crop


@pytest.mark.parametrize("variant", ["A", "B", "C"])
def test_reads_each_variant(variant):
    reading = read_delivery_variant(delivery_sign_crop(variant, height=160))
    assert reading.variant == variant
    assert reading.confidence > 0.3


@pytest.mark.parametrize("variant", ["A", "B", "C"])
def test_reads_each_variant_at_the_floor(variant):
    crop = delivery_sign_crop(variant, height=MIN_CROP_HEIGHT + 4)
    assert read_delivery_variant(crop).variant == variant


@pytest.mark.parametrize("variant,sides", [("B", 5), ("A", 6), ("C", 7)])
def test_counts_the_sides(variant, sides):
    reading = read_delivery_variant(delivery_sign_crop(variant, height=160))
    assert reading.sides == sides


@pytest.mark.parametrize("variant,holes", [("A", 1), ("B", 2), ("C", 0)])
def test_counts_the_counters(variant, holes):
    reading = read_delivery_variant(delivery_sign_crop(variant, height=160))
    assert reading.holes == holes


@pytest.mark.parametrize("rotation", [0, 13, 27, 40])
def test_rotation_only_moves_the_phase(rotation):
    reading = read_delivery_variant(delivery_sign_crop("A", height=160, rotation=rotation))
    assert reading.sides == 6


def test_white_background_does_not_leak():
    reading = read_delivery_variant(delivery_sign_crop("B", height=160, background=250))
    assert reading.variant == "B"


def test_black_background():
    assert read_delivery_variant(delivery_sign_crop("C", height=160, background=5)).variant == "C"


def test_below_the_floor_refuses():
    crop = delivery_sign_crop("A", height=MIN_CROP_HEIGHT // 3)
    reading = read_delivery_variant(crop)
    assert reading.variant is None
    assert "floor" in reading.reason


def test_blank_crop_refuses():
    reading = read_delivery_variant(blank_crop(160, 160))
    assert reading.variant is None


def test_disagreement_refuses_rather_than_picks():
    crop = delivery_sign_crop("A", height=200)
    height, width = crop.shape[:2]
    cv.rectangle(crop, (int(width * 0.3), int(height * 0.3)),
                 (int(width * 0.7), int(height * 0.7)), (246, 246, 246), -1)
    cv.putText(crop, "C", (int(width * 0.33), int(height * 0.68)),
               cv.FONT_HERSHEY_DUPLEX, 3.0, (12, 12, 12), 8)
    reading = read_delivery_variant(crop)
    assert reading.variant is None or reading.variant == "A"
    if reading.variant is None:
        assert "says" in reading.reason


def test_segmentation_finds_a_face_of_a_plausible_size():
    crop = delivery_sign_crop("A", height=160)
    face = segment_face(crop)
    assert face is not None
    covered = float((face > 0).sum()) / face.size
    assert 0.3 < covered < 0.85
