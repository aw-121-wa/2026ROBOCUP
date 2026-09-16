#!/usr/bin/env python3
"""Minimal Hiwonder/Lobot action-group UART driver used for RDK X5 tests.

Protocol is copied from the project's STM32 App/servo_action.c:
  TX run group: 55 55 05 06 GROUP REPEAT_L REPEAT_H
  RX complete : 55 55 05 08 GROUP ... ...

This module intentionally implements only the action-group functionality
needed by the direct-control test.
"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Iterable, List, Optional

try:
    import serial  # type: ignore
except ImportError:  # Allows parser/unit checks without pyserial installed.
    serial = None

HEADER = 0x55
CMD_RUN_GROUP = 0x06
CMD_GROUP_COMPLETE = 0x08


@dataclass(frozen=True)
class HiwonderFrame:
    raw: bytes
    length: int
    command: int
    data: bytes

    @property
    def group(self) -> Optional[int]:
        return self.data[0] if self.data else None


class HiwonderFrameParser:
    """Byte-stream parser matching the STM32 parser's length semantics."""

    def __init__(self, max_length: int = 32):
        self.max_length = max_length
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, chunk: bytes) -> List[HiwonderFrame]:
        if chunk:
            self._buffer.extend(chunk)
        frames: List[HiwonderFrame] = []

        while True:
            # Resynchronise on 55 55.
            idx = self._buffer.find(b"\x55\x55")
            if idx < 0:
                # Keep a trailing 0x55 in case the next chunk starts with 0x55.
                if self._buffer[-1:] == b"\x55":
                    self._buffer[:] = b"\x55"
                else:
                    self._buffer.clear()
                break
            if idx > 0:
                del self._buffer[:idx]

            if len(self._buffer) < 3:
                break

            length = self._buffer[2]
            # Existing STM32 code accepts length >= 2. Limit upper bound to
            # avoid a corrupted byte keeping the parser stuck indefinitely.
            if length < 2 or length > self.max_length:
                del self._buffer[0]
                continue

            total = 2 + length  # two 0x55 headers + 'length' bytes
            if len(self._buffer) < total:
                break

            raw = bytes(self._buffer[:total])
            del self._buffer[:total]
            command = raw[3]
            data = raw[4:]
            frames.append(
                HiwonderFrame(raw=raw, length=length, command=command, data=data)
            )

        return frames


def build_run_group_frame(group: int, repeat_count: int = 1) -> bytes:
    if not 0 <= group <= 255:
        raise ValueError("group must be 0..255")
    if not 1 <= repeat_count <= 0xFFFF:
        raise ValueError("repeat_count must be 1..65535")
    return bytes(
        [
            HEADER,
            HEADER,
            0x05,
            CMD_RUN_GROUP,
            group,
            repeat_count & 0xFF,
            (repeat_count >> 8) & 0xFF,
        ]
    )


class HiwonderActionBoard:
    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        read_timeout_s: float = 0.05,
    ):
        self.port = port
        self.baudrate = baudrate
        self.read_timeout_s = read_timeout_s
        self._serial = None
        self._parser = HiwonderFrameParser()

    @property
    def is_open(self) -> bool:
        return bool(self._serial is not None and self._serial.is_open)

    def open(self) -> None:
        if self.is_open:
            return
        if serial is None:
            raise RuntimeError("pyserial is not installed: import serial failed")
        self._serial = serial.Serial(
            self.port,
            self.baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=1,
            timeout=self.read_timeout_s,
            write_timeout=0.5,
        )
        self._parser.reset()
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None
        self._parser.reset()

    def __enter__(self) -> "HiwonderActionBoard":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def start_group(self, group: int, repeat_count: int = 1) -> bytes:
        if not self.is_open:
            raise RuntimeError("serial port is not open")
        frame = build_run_group_frame(group, repeat_count)
        # Mirror STM32 behavior: discard a stale completion before a new run.
        self._serial.reset_input_buffer()
        self._parser.reset()
        self._serial.write(frame)
        self._serial.flush()
        return frame

    def wait_group_complete(self, group: int, timeout_s: float) -> HiwonderFrame:
        if not self.is_open:
            raise RuntimeError("serial port is not open")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            chunk = self._serial.read(64)
            for frame in self._parser.feed(chunk):
                print(f"RX frame: {frame.raw.hex(' ')}")
                if (
                    frame.length == 5
                    and frame.command == CMD_GROUP_COMPLETE
                    and frame.group == group
                ):
                    return frame
        raise TimeoutError(
            f"no completion frame for action group {group} within {timeout_s:.1f}s"
        )

    def run_group(
        self,
        group: int,
        repeat_count: int = 1,
        timeout_s: float = 20.0,
    ) -> HiwonderFrame:
        tx = self.start_group(group, repeat_count)
        print(f"TX frame: {tx.hex(' ')}")
        return self.wait_group_complete(group, timeout_s)
