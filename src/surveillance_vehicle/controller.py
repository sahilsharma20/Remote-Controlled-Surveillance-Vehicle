"""Real-time camera, pose-estimation and servo-control pipeline."""

from __future__ import annotations

import logging
import time
from contextlib import nullcontext
from dataclasses import dataclass

from .geometry import calculate_bounding_box, calculate_shoulder_center, map_x_to_servo_angle
from .serial_io import ServoLink

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class TrackingConfig:
    serial_port: str | None = None
    baud_rate: int = 9600
    camera_index: int = 0
    frame_width: int = 640
    frame_height: int = 480
    servo_min: int = 55
    servo_max: int = 180
    update_interval: float = 0.25
    min_detection_confidence: float = 0.5
    min_tracking_confidence: float = 0.5
    mirror_input: bool = True
    invert_servo: bool = True
    dry_run: bool = False


def run_tracking(config: TrackingConfig) -> None:
    """Run the tracking loop until the user presses ``q`` or ``Esc``."""
    try:
        import cv2
        import mediapipe as mp
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise RuntimeError(
            "OpenCV and MediaPipe are required. Install the project dependencies first."
        ) from exc

    capture = cv2.VideoCapture(config.camera_index)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.frame_width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.frame_height)
    if not capture.isOpened():
        raise RuntimeError(f"Unable to open camera index {config.camera_index}")

    serial_context = (
        nullcontext(None)
        if config.dry_run
        else ServoLink(config.serial_port or "", config.baud_rate)
    )

    pose_module = mp.solutions.pose
    drawing = mp.solutions.drawing_utils

    LOGGER.info("Tracking started. Press q or Esc to stop.")
    last_update = 0.0
    last_angle: int | None = None

    try:
        with serial_context as servo_link, pose_module.Pose(
            min_detection_confidence=config.min_detection_confidence,
            min_tracking_confidence=config.min_tracking_confidence,
        ) as pose:
            while True:
                ok, frame = capture.read()
                if not ok:
                    LOGGER.warning("Camera frame could not be read; stopping")
                    break

                frame = cv2.resize(frame, (config.frame_width, config.frame_height))
                if config.mirror_input:
                    frame = cv2.flip(frame, 1)

                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                rgb_frame.flags.writeable = False
                results = pose.process(rgb_frame)
                rgb_frame.flags.writeable = True

                display_frame = frame.copy()
                status = "No person detected"

                if results.pose_landmarks:
                    landmarks = results.pose_landmarks.landmark
                    drawing.draw_landmarks(
                        display_frame,
                        results.pose_landmarks,
                        pose_module.POSE_CONNECTIONS,
                    )

                    box = calculate_bounding_box(
                        landmarks,
                        config.frame_width,
                        config.frame_height,
                    )
                    if box is not None:
                        min_x, min_y, max_x, max_y = box
                        cv2.rectangle(
                            display_frame,
                            (min_x, min_y),
                            (max_x, max_y),
                            (0, 255, 0),
                            2,
                        )

                    target = calculate_shoulder_center(
                        landmarks,
                        config.frame_width,
                        config.frame_height,
                    )
                    if target is not None:
                        target_x, target_y = target
                        cv2.circle(display_frame, (target_x, target_y), 7, (255, 0, 0), -1)

                        angle = map_x_to_servo_angle(
                            target_x,
                            config.frame_width,
                            config.servo_min,
                            config.servo_max,
                            invert=config.invert_servo,
                        )
                        status = f"Target x={target_x} | Pan={angle} deg"

                        now = time.monotonic()
                        if now - last_update >= config.update_interval and angle != last_angle:
                            if config.dry_run:
                                LOGGER.info("[dry-run] pan angle=%d", angle)
                            else:
                                assert servo_link is not None
                                servo_link.send_angle(angle)
                            last_update = now
                            last_angle = angle

                cv2.putText(
                    display_frame,
                    status,
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    display_frame,
                    "DRY RUN" if config.dry_run else "HARDWARE MODE",
                    (12, config.frame_height - 16),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                cv2.imshow("Vision-Guided Surveillance Rover", display_frame)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
    finally:
        capture.release()
        cv2.destroyAllWindows()
        LOGGER.info("Tracking stopped and resources released")
