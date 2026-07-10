"""Pure geometry helpers used by the tracking pipeline.

Keeping these functions independent of OpenCV, MediaPipe and serial hardware makes
core behaviour easy to test in CI and reason about during interviews.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol


class LandmarkLike(Protocol):
    """Small subset of a MediaPipe landmark used by this module."""

    x: float
    y: float
    visibility: float


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp *value* to the inclusive range ``[lower, upper]``."""
    if lower > upper:
        raise ValueError("lower must not be greater than upper")
    return max(lower, min(value, upper))


def linear_map(
    value: float,
    source_min: float,
    source_max: float,
    target_min: float,
    target_max: float,
    *,
    clamp_input: bool = True,
) -> float:
    """Map a value from one numeric interval to another."""
    if source_min == source_max:
        raise ValueError("source range cannot have zero width")

    mapped_value = value
    if clamp_input:
        mapped_value = clamp(mapped_value, min(source_min, source_max), max(source_min, source_max))

    ratio = (mapped_value - source_min) / (source_max - source_min)
    return target_min + ratio * (target_max - target_min)


def calculate_bounding_box(
    landmarks: Sequence[LandmarkLike],
    frame_width: int,
    frame_height: int,
    *,
    visibility_threshold: float = 0.5,
) -> tuple[int, int, int, int] | None:
    """Return a pixel-space box around sufficiently visible pose landmarks."""
    visible = [point for point in landmarks if point.visibility >= visibility_threshold]
    if not visible:
        return None

    x_values = [clamp(point.x, 0.0, 1.0) for point in visible]
    y_values = [clamp(point.y, 0.0, 1.0) for point in visible]

    return (
        int(min(x_values) * frame_width),
        int(min(y_values) * frame_height),
        int(max(x_values) * frame_width),
        int(max(y_values) * frame_height),
    )


def calculate_shoulder_center(
    landmarks: Sequence[LandmarkLike],
    frame_width: int,
    frame_height: int,
    *,
    left_shoulder_index: int = 11,
    right_shoulder_index: int = 12,
    visibility_threshold: float = 0.5,
) -> tuple[int, int] | None:
    """Estimate torso centre using the midpoint between both shoulders."""
    if len(landmarks) <= max(left_shoulder_index, right_shoulder_index):
        return None

    left = landmarks[left_shoulder_index]
    right = landmarks[right_shoulder_index]
    if min(left.visibility, right.visibility) < visibility_threshold:
        return None

    center_x = clamp((left.x + right.x) / 2.0, 0.0, 1.0)
    center_y = clamp((left.y + right.y) / 2.0, 0.0, 1.0)
    return int(center_x * frame_width), int(center_y * frame_height)


def map_x_to_servo_angle(
    x_coordinate: float,
    frame_width: int,
    servo_min: int,
    servo_max: int,
    *,
    invert: bool = True,
) -> int:
    """Convert horizontal image position into a bounded servo angle."""
    if frame_width <= 0:
        raise ValueError("frame_width must be positive")
    if not (0 <= servo_min <= 180 and 0 <= servo_max <= 180):
        raise ValueError("servo limits must be between 0 and 180 degrees")
    if servo_min >= servo_max:
        raise ValueError("servo_min must be smaller than servo_max")

    target_min, target_max = (servo_max, servo_min) if invert else (servo_min, servo_max)
    angle = linear_map(
        x_coordinate,
        0.0,
        float(frame_width),
        float(target_min),
        float(target_max),
        clamp_input=True,
    )
    return int(round(clamp(angle, 0.0, 180.0)))
