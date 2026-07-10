"""Serial communication helpers for sending pan-servo commands."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any

LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class ServoLink:
    """Small context-managed wrapper around a pySerial connection."""

    port: str
    baud_rate: int = 9600
    timeout: float = 1.0
    startup_delay: float = 2.0
    _connection: Any = None

    def open(self) -> "ServoLink":
        try:
            import serial
        except ImportError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError("pyserial is required for hardware mode") from exc

        LOGGER.info("Opening serial connection: %s @ %d baud", self.port, self.baud_rate)
        self._connection = serial.Serial(
            self.port,
            self.baud_rate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        time.sleep(self.startup_delay)
        return self

    def send_angle(self, angle: int) -> None:
        """Send an integer angle as a newline-terminated ASCII command."""
        if self._connection is None:
            raise RuntimeError("serial connection is not open")
        if not 0 <= angle <= 180:
            raise ValueError("servo angle must be between 0 and 180 degrees")

        payload = f"{angle}\n".encode("ascii")
        self._connection.write(payload)
        self._connection.flush()
        LOGGER.debug("Sent servo angle: %d", angle)

    def close(self) -> None:
        if self._connection is not None:
            self._connection.close()
            self._connection = None
            LOGGER.info("Serial connection closed")

    def __enter__(self) -> "ServoLink":
        return self.open()

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()


def list_serial_ports() -> list[tuple[str, str]]:
    """Return available serial device names and human-readable descriptions."""
    try:
        from serial.tools import list_ports
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise RuntimeError("pyserial is required to enumerate serial ports") from exc

    return [(port.device, port.description or "Unknown device") for port in list_ports.comports()]
