# Hardware Setup

## Reference components

- Arduino Uno or compatible microcontroller
- Pan servo compatible with the mechanical load
- USB data cable
- Webcam or built-in laptop camera
- Four-wheel robot chassis and motor driver, when mobility is required
- Separate, correctly rated power supply for motors and servos
- Common ground between the actuator power supply and Arduino

> Exact wiring depends on the servo, board and power source. Confirm current limits from the component datasheets before powering the system.

## Basic pan-servo wiring

| Servo lead | Typical connection |
|---|---|
| Signal | Arduino digital pin 9, or the pin configured in the sketch |
| Power | External regulated supply matched to the servo |
| Ground | Supply ground and Arduino ground |

Do not power a high-load servo directly from an Arduino 5 V pin. Brownouts and USB disconnections are common when the servo draws more current than the board can provide.

## Setup sequence

1. Keep the servo mechanically unloaded during the first test.
2. Open `arduino/servo_controller/servo_controller.ino` in Arduino IDE.
3. Confirm `SERVO_PIN`, baud rate and safe angle limits.
4. Upload the sketch to the board.
5. List ports with `python -m surveillance_vehicle --list-ports`.
6. Test vision only with `python -m surveillance_vehicle --dry-run`.
7. Connect the mechanism and run with the detected serial port.
8. Reduce `--servo-min` and `--servo-max` if the mount approaches a hard stop.

## Platform-specific serial-port examples

- macOS: `/dev/cu.usbmodem101` or `/dev/cu.usbserial-*`
- Linux: `/dev/ttyACM0` or `/dev/ttyUSB0`
- Windows: `COM3`, `COM4`, and similar

Port names vary by machine. Use the built-in port listing command rather than hard-coding one into the source code.
