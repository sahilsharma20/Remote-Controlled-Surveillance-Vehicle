# System Architecture

## Processing pipeline

```mermaid
flowchart LR
    A[Camera frame] --> B[OpenCV capture and resize]
    B --> C[MediaPipe Pose inference]
    C --> D[Visible landmark filtering]
    D --> E[Shoulder midpoint target]
    E --> F[Pixel-to-angle mapping]
    F --> G[Rate limit and clamp]
    G --> H[pySerial command]
    H --> I[Arduino]
    I --> J[Pan servo]
```

## Module responsibilities

| Module | Responsibility |
|---|---|
| `geometry.py` | Pure coordinate calculations, clamping and angle mapping |
| `serial_io.py` | Serial lifecycle, port discovery and command transport |
| `controller.py` | Camera loop, MediaPipe inference, overlays and update scheduling |
| `cli.py` | Argument parsing, validation and execution modes |
| `servo_controller.ino` | Parse angle commands and actuate the pan servo |

## Key engineering decisions

### Shoulder midpoint instead of one landmark

The original prototype tracked a single shoulder and applied a fixed pixel offset. The refactored implementation uses the midpoint of the left and right shoulders. This is easier to explain, less dependent on body orientation and removes a magic offset from the control loop.

### Bounded mapping

Image coordinates are clamped before mapping, and servo limits must remain inside 0–180 degrees. These checks reduce the risk of sending commands outside the actuator's expected range.

### Rate-limited serial updates

Vision inference can run faster than a hobby servo should be commanded. The controller therefore sends a new angle only after the configured update interval and only when the angle changes.

### Dry-run mode

`--dry-run` makes the complete vision path testable without Arduino hardware. This improves portability, debugging and recruiter demos.

## Current scope

The checked-in software implements camera-based human pose tracking and pan-servo actuation. Wheel-drive control, remote networking and autonomous navigation are not implemented in this repository and should not be presented as completed features.
