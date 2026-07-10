from dataclasses import dataclass

import pytest

from surveillance_vehicle.geometry import (
    calculate_bounding_box,
    calculate_shoulder_center,
    clamp,
    linear_map,
    map_x_to_servo_angle,
)


@dataclass
class Landmark:
    x: float
    y: float
    visibility: float = 1.0


def test_clamp() -> None:
    assert clamp(-2, 0, 10) == 0
    assert clamp(4, 0, 10) == 4
    assert clamp(20, 0, 10) == 10


def test_linear_map_and_clamping() -> None:
    assert linear_map(50, 0, 100, 0, 180) == pytest.approx(90)
    assert linear_map(200, 0, 100, 0, 180) == pytest.approx(180)


def test_map_x_to_servo_angle() -> None:
    assert map_x_to_servo_angle(0, 640, 55, 180, invert=False) == 55
    assert map_x_to_servo_angle(640, 640, 55, 180, invert=False) == 180
    assert map_x_to_servo_angle(0, 640, 55, 180, invert=True) == 180


def test_bounding_box_ignores_low_visibility_points() -> None:
    landmarks = [
        Landmark(0.2, 0.3),
        Landmark(0.8, 0.9),
        Landmark(0.0, 0.0, visibility=0.1),
    ]
    assert calculate_bounding_box(landmarks, 100, 200) == (20, 60, 80, 180)


def test_shoulder_center() -> None:
    landmarks = [Landmark(0.0, 0.0) for _ in range(13)]
    landmarks[11] = Landmark(0.4, 0.3)
    landmarks[12] = Landmark(0.6, 0.5)
    assert calculate_shoulder_center(landmarks, 640, 480) == (320, 192)
