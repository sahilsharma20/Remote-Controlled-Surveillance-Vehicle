"""Command-line interface for the surveillance rover."""

from __future__ import annotations

import argparse
import logging

from .controller import TrackingConfig, run_tracking
from .serial_io import list_serial_ports


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="surveillance-vehicle",
        description="Track a human torso with MediaPipe and control an Arduino pan servo.",
    )
    parser.add_argument(
        "--serial-port",
        help="Arduino serial port, for example COM3 or /dev/cu.usbmodem101",
    )
    parser.add_argument("--baud", type=int, default=9600, help="Serial baud rate (default: 9600)")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index (default: 0)")
    parser.add_argument("--width", type=int, default=640, help="Processing width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="Processing height (default: 480)")
    parser.add_argument("--servo-min", type=int, default=55, help="Minimum safe servo angle")
    parser.add_argument("--servo-max", type=int, default=180, help="Maximum safe servo angle")
    parser.add_argument(
        "--update-interval",
        type=float,
        default=0.25,
        help="Minimum seconds between serial updates (default: 0.25)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run vision tracking without Arduino hardware",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List detected serial ports and exit",
    )
    parser.add_argument("--no-mirror", action="store_true", help="Do not mirror the camera image")
    parser.add_argument(
        "--no-invert",
        action="store_true",
        help="Do not invert image-to-servo direction",
    )
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s | %(message)s",
    )

    if args.list_ports:
        ports = list_serial_ports()
        if not ports:
            print("No serial ports detected.")
        for device, description in ports:
            print(f"{device:<28} {description}")
        return

    if not args.dry_run and not args.serial_port:
        parser.error("--serial-port is required unless --dry-run is enabled")

    config = TrackingConfig(
        serial_port=args.serial_port,
        baud_rate=args.baud,
        camera_index=args.camera,
        frame_width=args.width,
        frame_height=args.height,
        servo_min=args.servo_min,
        servo_max=args.servo_max,
        update_interval=args.update_interval,
        mirror_input=not args.no_mirror,
        invert_servo=not args.no_invert,
        dry_run=args.dry_run,
    )
    run_tracking(config)


if __name__ == "__main__":
    main()
