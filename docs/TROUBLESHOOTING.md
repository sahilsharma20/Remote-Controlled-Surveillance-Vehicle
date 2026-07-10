# Troubleshooting

## Camera does not open

- Close applications already using the camera.
- Try another index: `--camera 1`.
- On macOS or Windows, grant camera permission to Terminal, VS Code or the Python interpreter.

## No person is detected

- Keep shoulders and upper body visible.
- Improve front lighting and reduce backlight.
- Increase the distance from the camera.
- Avoid heavy motion blur.

## Servo moves in the wrong direction

Use `--no-invert`, or reverse the mechanical orientation of the mount.

## Servo hits a mechanical stop

Use narrower limits, for example:

```bash
surveillance-vehicle --serial-port /dev/cu.usbmodem101 --servo-min 65 --servo-max 155
```

## Serial port is missing

Run:

```bash
python -m surveillance_vehicle --list-ports
```

Then confirm the cable supports data, the board driver is installed and no serial monitor is holding the port.

## Jittery movement

- Increase `--update-interval`.
- Narrow the angle range.
- Use a stable external servo power supply.
- Add a deadband or smoothing filter as a future enhancement.

## macOS Apple silicon installation issue

Use a 64-bit Python environment. MediaPipe publishes support for arm64 macOS, but installation can still fail when the active interpreter and package architecture do not match.
