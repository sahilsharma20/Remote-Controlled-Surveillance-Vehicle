# Recruiter and Interview Walkthrough

## 60-second explanation

This project is a computer-vision robotics prototype that follows a person horizontally. A camera supplies frames to OpenCV, MediaPipe Pose estimates body landmarks, and the software calculates the midpoint between both shoulders as a stable tracking target. That horizontal pixel coordinate is mapped into a safe servo-angle range and transmitted to an Arduino over serial communication. The Arduino then rotates a pan servo so the mounted camera or demonstration rig follows the person. I refactored the prototype into modular, testable components and added a dry-run mode so the vision pipeline can be demonstrated without hardware.

## Problem statement

Static surveillance cameras have a limited field of view. The prototype explores how inexpensive computer vision and embedded hardware can create a pan-tracking platform that keeps a detected person near the centre of view.

## My technical contribution

- Integrated OpenCV camera capture with MediaPipe Pose.
- Converted normalized pose landmarks into pixel coordinates.
- Designed image-coordinate to servo-angle mapping.
- Connected Python and Arduino through pySerial.
- Built and tested a four-wheel physical prototype with a pan mechanism.
- Improved the repository with modular code, CLI configuration, tests and documentation.

## Important design choices

- Shoulder midpoint is more stable than tracking only one shoulder.
- Servo outputs are bounded to configurable mechanical limits.
- Serial commands are rate-limited to reduce unnecessary actuator movement.
- Dry-run mode separates computer-vision debugging from hardware debugging.
- Hardware-specific values are CLI arguments rather than hard-coded constants.

## Challenges and solutions

| Challenge | Approach |
|---|---|
| Camera and servo coordinate systems moved in opposite directions | Added an explicit invert option |
| Pose detection occasionally disappeared | Used visibility checks and skipped unsafe updates |
| Fixed serial-port path reduced portability | Added CLI configuration and port discovery |
| Broad exception handling hid faults | Replaced it with explicit validation and structured cleanup |
| Servo could be over-commanded | Added clamping, rate limiting and change detection |

## Honest limitations

- The current model is a single-camera, 2D pan-tracking prototype.
- It does not estimate physical distance or depth.
- Wheel movement and remote network control are outside the checked-in software scope.
- Tracking quality depends on lighting, visibility and camera placement.
- The project has not been benchmarked as a production surveillance system.

## Strong future enhancements

1. Exponential smoothing and servo deadband to reduce jitter.
2. Multi-person selection and persistent target IDs.
3. Pan-tilt control with two independent servos.
4. Web dashboard for telemetry and manual override.
5. Distance estimation or depth-camera integration.
6. ROS 2 nodes for camera, perception and actuator control.
7. Automated hardware-in-the-loop tests.
