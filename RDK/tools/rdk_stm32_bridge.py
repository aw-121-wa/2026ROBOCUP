#!/usr/bin/env python3
"""Concurrent STM32 bridge for the validated disc task and RFID gate."""
from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys
import threading
import time
from typing import Callable, Sequence

ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = Path(__file__).resolve().parent
for path in (ROOT, TOOLS_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

try:
    import serial  # type: ignore
except ImportError:
    serial = None

from disc_rfid_gate import DiscRfidGate
from vision_servo_direct_test import build_parser as build_disc_parser, run_disc_task

DEFAULT_DISC_TIMEOUT_S = 60.0


def normalize_command(raw: bytes | str) -> str:
    if isinstance(raw, (bytes, bytearray)):
        raw = raw.decode("ascii", errors="ignore")
    return raw.strip().upper()


def build_disc_arguments(project_root: Path):
    return build_disc_parser().parse_args(
        [
            "--config", str(project_root / "rdk_vision" / "config.yaml"),
            "--color", "red",
            "--servo-port", "/dev/ttyS1",
            "--servo-baud", "9600",
            "--prep-group", "101",
            "--trigger-group", "102",
            "--repeat", "1",
            "--max-actions", "5",
            "--trigger-x", "380",
            "--servo-timeout", "30",
        ]
    )


def run_disc_in_process(project_root: Path, *, rfid_gate, on_action_complete) -> int:
    return run_disc_task(
        build_disc_arguments(project_root),
        rfid_gate=rfid_gate,
        on_action_complete=on_action_complete,
    )


class SerialLineWriter:
    """Serialize complete CRLF frames from the main and disc threads."""

    def __init__(self, serial_port, log=print):
        self._serial = serial_port
        self._log = log
        self._lock = threading.Lock()
        self._error = None

    def __call__(self, text: str) -> None:
        payload = (text + "\r\n").encode("ascii")
        with self._lock:
            try:
                self._serial.write(payload)
                self._serial.flush()
            except Exception as exc:
                self._error = exc
                raise
        self._log(f"STM32 TX: {text}")

    def raise_if_failed(self) -> None:
        with self._lock:
            error = self._error
        if error is not None:
            raise RuntimeError(f"serial TX failed: {error!r}") from error


class BridgeCore:
    """Non-blocking command router with exactly one disc worker."""

    def __init__(
        self,
        send_line: Callable[[str], None],
        run_disc: Callable[..., int],
        *,
        clock=time.monotonic,
        disc_timeout_s: float = DEFAULT_DISC_TIMEOUT_S,
    ):
        self._send_line = send_line
        self._run_disc = run_disc
        self._clock = clock
        self._disc_timeout_s = float(disc_timeout_s)
        self._lock = threading.Lock()
        self._worker = None
        self._gate = None
        self._deadline = None

    @property
    def gate(self):
        with self._lock:
            return self._gate

    @property
    def disc_active(self) -> bool:
        with self._lock:
            return self._worker is not None

    def _action_complete(self, index: int) -> None:
        self._send_line(f"DISC_ACTION_DONE {index}")

    def _worker_main(self, gate: DiscRfidGate) -> None:
        try:
            try:
                rc = int(
                    self._run_disc(
                        rfid_gate=gate,
                        on_action_complete=self._action_complete,
                    )
                )
            except Exception as exc:  # noqa: BLE001 - service boundary
                print(f"DISC task exception: {exc!r}", flush=True)
                rc = 1
            success = rc == 0 and gate.is_complete()
            if not success:
                gate.cancel()
            try:
                self._send_line("DISC_DONE" if success else "DISC_ERROR")
            except Exception as exc:  # transport is owned by the serial loop
                print(f"DISC terminal TX failed: {exc!r}", flush=True)
        finally:
            with self._lock:
                if self._gate is gate:
                    self._worker = None
                    self._gate = None
                    self._deadline = None

    def handle(self, command: str) -> None:
        command = normalize_command(command)
        if command == "PING":
            self._send_line("PONG")
            return
        if command == "DISC_START":
            with self._lock:
                if self._worker is not None:
                    print("Duplicate DISC_START ignored: disc task is active", flush=True)
                    return
                gate = DiscRfidGate(max_actions=5)
                worker = threading.Thread(
                    target=self._worker_main,
                    args=(gate,),
                    name="disc-task",
                    daemon=True,
                )
                self._gate = gate
                self._worker = worker
                self._deadline = self._clock() + self._disc_timeout_s
            self._send_line("DISC_ACK")
            worker.start()
            return
        if command == "DISC_CANCEL":
            gate = self.gate
            if gate is None:
                print("Protocol warning: ignored DISC_CANCEL without active task", flush=True)
            else:
                gate.cancel()
                print("DISC CANCELLED: ACTION GATE CLOSED", flush=True)
            return
        match = re.fullmatch(r"DISC_RFID_OK ([1-5])", command)
        if match:
            index = int(match.group(1))
            gate = self.gate
            if gate is None or not gate.on_rfid_confirmed(index):
                print(f"Protocol warning: ignored {command}", flush=True)
            else:
                print(f"RFID CONFIRMED #{index}", flush=True)
                if index < gate.max_actions:
                    print("ACTION GATE OPEN", flush=True)
            return
        if command:
            self._send_line("ERR_UNKNOWN")

    def cancel_active(self) -> None:
        gate = self.gate
        if gate is not None:
            gate.cancel()

    def tick(self) -> None:
        with self._lock:
            gate = self._gate
            deadline = self._deadline
            if gate is not None and deadline is not None and self._clock() >= deadline:
                self._deadline = None
            else:
                gate = None
        if gate is not None:
            print("DISC overall timeout: action gate closed", flush=True)
            gate.cancel()

    def shutdown(self) -> None:
        self.cancel_active()
        with self._lock:
            worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join()

    def wait_for_idle(self, timeout_s: float) -> bool:
        with self._lock:
            worker = self._worker
        if worker is None:
            return True
        worker.join(timeout_s)
        return not worker.is_alive()


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


def service_loop(project_root: Path, port: str, baudrate: int, reconnect_delay_s: float) -> None:
    while True:
        ser = None
        core = None
        try:
            print(f"Opening STM32 link {port} @ {baudrate}...", flush=True)
            ser = open_serial(port, baudrate, timeout=0.1)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print("STM32 bridge ready; waiting for PING / DISC_START", flush=True)
            send_line = SerialLineWriter(ser, log=lambda text: print(text, flush=True))
            core = BridgeCore(
                send_line=send_line,
                run_disc=lambda **kwargs: run_disc_in_process(project_root, **kwargs),
            )
            rx = bytearray()
            while True:
                send_line.raise_if_failed()
                core.tick()
                chunk = ser.read(64)
                if not chunk:
                    continue
                rx.extend(chunk)
                while b"\n" in rx:
                    raw_line, _, remainder = rx.partition(b"\n")
                    rx[:] = remainder
                    command = normalize_command(raw_line.rstrip(b"\r"))
                    if command:
                        print(f"STM32 RX: {command}", flush=True)
                        core.handle(command)
        except KeyboardInterrupt:
            raise
        except Exception as exc:  # noqa: BLE001 - reconnect boundary
            print(f"STM32 bridge serial error: {exc!r}", flush=True)
        finally:
            if core is not None:
                core.shutdown()
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
    service_loop(Path(args.project_root).resolve(), args.stm32_port, args.stm32_baud, args.reconnect_delay)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
