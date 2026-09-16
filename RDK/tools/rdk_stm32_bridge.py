#!/usr/bin/env python3
"""Boot-time bridge: wait for STM32 commands and launch the validated disc task.

Protocol on STM32 link (/dev/ttyUSB0 @ 115200 by default):
  STM32 -> RDK:  PING\r\n
  RDK   -> STM32: PONG\r\n

  STM32 -> RDK:  DISC_START\r\n
  RDK   -> STM32: DISC_ACK\r\n
  (run existing validated G101 + red-disc vision + G102x5 task)
  RDK   -> STM32: DISC_DONE\r\n on exit code 0
  RDK   -> STM32: DISC_ERROR\r\n on non-zero exit code

The existing vision/servo task is intentionally launched as a child process so
its tested behavior and calibration remain unchanged.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import os
import subprocess
import sys
import time
from typing import Callable, Sequence

try:
    import serial  # type: ignore
except ImportError:
    serial = None


def normalize_command(raw: bytes | str) -> str:
    if isinstance(raw, (bytes, bytearray)):
        raw = raw.decode("ascii", errors="ignore")
    return raw.strip().upper()


def build_disc_command(project_root: Path) -> list[str]:
    script = project_root / "tools" / "vision_servo_direct_test.py"
    config = project_root / "rdk_vision" / "config.yaml"
    return [
        sys.executable,
        str(script),
        "--config",
        str(config),
        "--color",
        "red",
        "--servo-port",
        "/dev/ttyS1",
        "--servo-baud",
        "9600",
        "--prep-group",
        "101",
        "--trigger-group",
        "102",
        "--repeat",
        "1",
        "--max-actions",
        "5",
        "--trigger-x",
        "380",
        "--servo-timeout",
        "30",
    ]


class BridgeCore:
    """Pure command core so the serial bridge can be unit tested."""

    def __init__(self, send_line: Callable[[str], None], run_disc: Callable[[], int]):
        self._send_line = send_line
        self._run_disc = run_disc

    def handle(self, command: str) -> None:
        command = normalize_command(command)
        if command == "PING":
            self._send_line("PONG")
            return
        if command == "DISC_START":
            self._send_line("DISC_ACK")
            try:
                rc = int(self._run_disc())
            except Exception as exc:  # noqa: BLE001 - service boundary
                print(f"DISC task exception: {exc!r}", flush=True)
                rc = 1
            self._send_line("DISC_DONE" if rc == 0 else "DISC_ERROR")
            return
        if command:
            self._send_line("ERR_UNKNOWN")


def run_disc_subprocess(project_root: Path) -> int:
    cmd = build_disc_command(project_root)
    print("Launching disc task:", flush=True)
    print("  " + " ".join(cmd), flush=True)
    env = dict(os.environ)
    env["PYTHONUNBUFFERED"] = "1"
    completed = subprocess.run(cmd, cwd=project_root, env=env, check=False)
    print(f"Disc task exited rc={completed.returncode}", flush=True)
    return int(completed.returncode)


def open_serial(port: str, baudrate: int, timeout: float):
    if serial is None:
        raise RuntimeError("pyserial is not installed")
    return serial.Serial(
        port,
        baudrate,
        bytesize=8,
        parity=serial.PARITY_NONE,
        stopbits=1,
        timeout=timeout,
        write_timeout=1.0,
    )


def service_loop(
    project_root: Path,
    port: str,
    baudrate: int,
    reconnect_delay_s: float,
) -> None:
    while True:
        ser = None
        try:
            print(f"Opening STM32 link {port} @ {baudrate}...", flush=True)
            ser = open_serial(port, baudrate, timeout=0.1)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print("STM32 bridge ready; waiting for PING / DISC_START", flush=True)

            def send_line(text: str) -> None:
                payload = (text + "\r\n").encode("ascii")
                ser.write(payload)
                ser.flush()
                print(f"STM32 TX: {text}", flush=True)

            core = BridgeCore(
                send_line=send_line,
                run_disc=lambda: run_disc_subprocess(project_root),
            )
            rx = bytearray()

            while True:
                chunk = ser.read(64)
                if not chunk:
                    continue
                rx.extend(chunk)
                while b"\n" in rx:
                    raw_line, _, remainder = rx.partition(b"\n")
                    rx[:] = remainder
                    command = normalize_command(raw_line.rstrip(b"\r"))
                    if not command:
                        continue
                    print(f"STM32 RX: {command}", flush=True)
                    core.handle(command)

        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001 - reconnect boundary
            print(f"STM32 bridge serial error: {exc!r}", flush=True)
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
        print(f"Retrying {port} in {reconnect_delay_s:.1f}s...", flush=True)
        time.sleep(reconnect_delay_s)


def main(argv: Sequence[str] | None = None) -> int:
    default_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="STM32 -> RDK disc-task boot bridge")
    parser.add_argument("--project-root", default=str(default_root))
    parser.add_argument("--stm32-port", default="/dev/ttyUSB0")
    parser.add_argument("--stm32-baud", type=int, default=115200)
    parser.add_argument("--reconnect-delay", type=float, default=1.0)
    args = parser.parse_args(argv)

    project_root = Path(args.project_root).resolve()
    service_loop(
        project_root=project_root,
        port=args.stm32_port,
        baudrate=args.stm32_baud,
        reconnect_delay_s=args.reconnect_delay,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
