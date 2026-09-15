#!/usr/bin/env python3
"""PATH bridge. Overlay onto the existing vision project; keep its calibration.

Q session sequence HELLO|STOP|GROUP|VISION|DISC argument\n
R session sequence ACK|DONE|NONE|ERROR\n
DISC is ONE detection + unchanged G102; STM32 owns counting and turntable.
STOP cancels recognition, but waits for an already-running action completion.
"""
import argparse
from collections import OrderedDict, deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import signal
import sys
import threading
import time


class LineReader:
    def __init__(self):
        self.buffer = bytearray()
        self.discard = False
        self.last = 0.0

    def feed(self, data, now):
        if self.buffer and now - self.last > 0.5:
            self.discard = True
        if data:
            self.last = now
        lines = []
        for byte in data:
            if byte in (10, 13):
                if self.buffer and not self.discard:
                    lines.append(self.buffer.decode("ascii"))
                self.buffer.clear()
                self.discard = False
            elif self.discard:
                continue
            elif byte < 32 or byte > 126 or len(self.buffer) >= 95:
                self.discard = True
            else:
                self.buffer.append(byte)
        return lines


class Engine:
    """Main-thread protocol owner; submit runs one cancellable hardware job."""
    def __init__(self, submit, send):
        self.submit, self.send = submit, send
        self.session = None
        self.highest = 0
        self.retired = deque(maxlen=32)
        self.cache = OrderedDict()
        self.active = None
        self.stop_key = None
        self.faulted = False
        self.last_seen = 0.0

    def _reply(self, key, status):
        self.send(f"R {key[0]} {key[1]} {status}")

    def _finish(self, key, status):
        request = self.cache[key][0]
        self.cache[key] = (request, status)
        self._reply(key, status)

    def poll(self):
        if self.active is not None and time.monotonic() - self.last_seen > 2.0:
            self.cancel_active()
        if self.active is None or not self.active[1].done():
            return
        key, future, _ = self.active
        try:
            status = future.result()
            if status not in ("DONE", "NONE"):
                status = "ERROR"
        except Exception as exc:
            print(f"Hardware job failed: {exc}", file=sys.stderr, flush=True)
            status = "ERROR"
        if status == "ERROR":
            # Completion is uncertain. Never authorize subsequent base motion.
            self.faulted = True
        self._finish(key, status)
        self.active = None
        if self.stop_key is not None:
            self._finish(self.stop_key, "ERROR" if self.faulted else "DONE")
            self.stop_key = None

    def cancel_active(self):
        if self.active is not None:
            self.active[2].set()

    def accept(self, line):
        self.poll()
        words = line.split()
        if len(words) != 5 or words[0] != "Q":
            return
        if any(not word.isascii() or not word.isdecimal() for word in (words[1], words[2], words[4])):
            return
        sid, seq, arg = int(words[1]), int(words[2]), int(words[4])
        if not (1 <= sid <= 0xffffffff and 1 <= seq <= 0xffffffff):
            return
        key, op = (sid, seq), words[3]
        request = (op, arg)
        cached = self.cache.get(key)
        if cached is not None:
            if (cached[0] == request and self.active is not None and
                    key == self.active[0]):
                self.last_seen = time.monotonic()
            self._reply(key, cached[1] if cached[0] == request else "ERROR")
            return
        valid = ((op in ("HELLO", "STOP") and arg == 0) or
                 (op == "GROUP" and 0 <= arg <= 255) or
                 (op in ("DISC", "VISION") and 1 <= arg <= 60000))
        if not valid or self.faulted:
            self._reply(key, "ERROR")
            return
        if op == "HELLO" and sid != self.session:
            if (self.active is not None or sid in self.retired or
                    len(self.retired) >= self.retired.maxlen):
                self._reply(key, "ERROR")
                return
            if self.session is not None:
                self.retired.append(self.session)
            self.session, self.highest = sid, 0
            self.cache.clear()
        if sid != self.session or seq <= self.highest:
            self._reply(key, "ERROR")
            return
        self.highest = seq
        # Only evict completed history, never active/STOP acknowledgements.
        while len(self.cache) >= 16:
            victim = next((k for k, v in self.cache.items() if v[1] != "ACK"), None)
            if victim is None:
                self._reply(key, "ERROR")
                return
            del self.cache[victim]
        self.cache[key] = (request, "ACK")
        if op == "STOP":
            if self.active is None:
                self._finish(key, "DONE")
            elif self.stop_key is not None:
                self._finish(key, "ERROR")
            else:
                self.cancel_active()
                self.stop_key = key
                self.last_seen = time.monotonic()
                self._reply(key, "ACK")
        elif self.active is not None:
            self._finish(key, "ERROR")
        elif op == "HELLO":
            self._finish(key, "DONE")
        else:
            cancel = threading.Event()
            try:
                future = self.submit(op, arg, cancel)
            except Exception:
                self.faulted = True
                self._finish(key, "ERROR")
                return
            self.active = (key, future, cancel)
            self.last_seen = time.monotonic()
            self._reply(key, "ACK")


class VisionHardware:
    def __init__(self, args):
        root = Path(args.project_root).resolve()
        sys.path[:0] = [str(root), str(root / "tools")]
        from rdk_vision.config import load_config
        from rdk_vision.camera import LatestFrameCamera
        from rdk_vision.ball_detector import BallDetector
        from disc_rtl_trigger import RightToLeftDiscTrigger
        from hiwonder_action import HiwonderActionBoard
        config_path = Path(args.config)
        if not config_path.is_absolute():
            config_path = root / config_path
        self.config = load_config(str(config_path))
        cfg = self.config
        if not cfg.roi.x < args.trigger_x < cfg.roi.x + cfg.roi.width:
            raise ValueError("trigger-x must be inside saved ROI")
        self.detector = BallDetector(cfg)
        self.gate = RightToLeftDiscTrigger(
            roi_right=cfg.roi.x + cfg.roi.width, trigger_x=args.trigger_x,
            reference_width=cfg.ball.reference_width,
            reference_height=cfg.ball.reference_height,
            aspect_max=max(1.35, cfg.ball.aspect_ratio_max),
            fill_min=max(0.45, cfg.ball.fill_ratio_min - 0.05),
            fill_max=min(0.95, cfg.ball.fill_ratio_max + 0.05))
        self.camera = LatestFrameCamera(cfg.camera)
        self.board = HiwonderActionBoard(args.arm_port, args.arm_baud)
        self.color = args.color
        self.action_timeout = args.action_timeout
        print(f"Config: {config_path}; ROI={cfg.roi}; trigger_x={args.trigger_x}", flush=True)

    def open(self):
        # Verify images BEFORE any preparation action is accepted.
        self.camera.start()
        if not self.camera.wait_until_ready(self.config.camera.startup_timeout_ms):
            raise TimeoutError("camera startup failed")
        self._check_frame(self.camera.get_latest())
        self.board.open()

    def _check_frame(self, frame):
        if frame is None:
            raise TimeoutError("camera has no frame")
        h, w = frame.frame.shape[:2]
        roi = self.config.roi
        if (w != self.config.camera.width or h != self.config.camera.height or
                roi.x < 0 or roi.y < 0 or roi.x + roi.width > w or roi.y + roi.height > h):
            raise ValueError("actual camera resolution / ROI differs from configured geometry")

    def run(self, op, arg, cancel):
        if cancel.is_set():
            return "NONE"
        if op == "GROUP":
            self.board.run_group(arg, timeout_s=self.action_timeout)
            return "DONE"
        deadline = time.monotonic() + arg / 1000.0
        snapshot = self.camera.get_latest()
        last_id = snapshot.frame_id if snapshot is not None else -1
        fresh_at, log_at = time.monotonic(), 0.0
        while not cancel.is_set():
            now = time.monotonic()
            if now >= deadline:
                return "NONE"
            snapshot = self.camera.get_latest()
            if snapshot is None or snapshot.frame_id <= last_id:
                if now - fresh_at >= 1.0:
                    raise TimeoutError("camera stopped producing new frames")
                cancel.wait(0.005)
                continue
            last_id = snapshot.frame_id
            if now - snapshot.timestamp > self.config.camera.stale_ms / 1000.0:
                if now - fresh_at >= 1.0:
                    raise TimeoutError("camera frames remain stale")
                continue
            fresh_at = now
            self._check_frame(snapshot)
            result = self.detector.detect(snapshot.frame, self.color)
            event = self.gate.update(result) if op == "DISC" else None
            hit = bool(event) if op == "DISC" else result.valid
            if hit or now - log_at >= 0.5:
                print(f"{op} frame={last_id} valid={result.valid} reason={result.reason} "
                      f"bbox={result.bbox} aspect={result.aspect_ratio:.3f} "
                      f"fill={result.fill_ratio:.3f} event={event}", flush=True)
                log_at = now
            if hit:
                # Recheck after detector work: never start a new action past its deadline.
                if cancel.is_set() or time.monotonic() >= deadline:
                    return "NONE"
                if op == "DISC":
                    self.board.run_group(102, timeout_s=self.action_timeout)
                return "DONE"
        return "NONE"

    def close(self):
        self.board.close()
        self.camera.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", default=".")
    parser.add_argument("--config", default="rdk_vision/config.yaml")
    parser.add_argument("--stm32-port", default="/dev/ttyUSB0")
    parser.add_argument("--stm32-baud", type=int, default=115200)
    parser.add_argument("--arm-port", default="/dev/ttyS1")
    parser.add_argument("--arm-baud", type=int, default=9600)
    parser.add_argument("--color", choices=("red", "green", "blue"), default="red")
    parser.add_argument("--trigger-x", type=int, default=380)
    parser.add_argument("--action-timeout", type=float, default=30.0)
    args = parser.parse_args()
    if not 0 < args.action_timeout <= 30:
        parser.error("action-timeout must be > 0 and <= 30 seconds")
    import serial
    hardware = VisionHardware(args)
    shutdown = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: shutdown.set())
    connection = None
    with ThreadPoolExecutor(max_workers=1) as executor:
        def send(line):
            print(line, flush=True)
            if connection is not None:
                connection.write((line + "\r\n").encode("ascii"))
        engine = Engine(lambda op, arg, cancel: executor.submit(hardware.run, op, arg, cancel), send)
        try:
            hardware.open()
            while not shutdown.is_set():
                try:
                    if connection is None:
                        connection = serial.Serial(args.stm32_port, args.stm32_baud,
                                                   timeout=0.02, write_timeout=0.2)
                        reader = LineReader()
                        print("PATH bridge ready; no action until a valid request", flush=True)
                    engine.poll()
                    for line in reader.feed(connection.read(64), time.monotonic()):
                        engine.accept(line)
                except (serial.SerialException, OSError) as exc:
                    print(f"STM32 link unavailable: {exc}", file=sys.stderr, flush=True)
                    if connection is not None:
                        connection.close()
                    connection = None
                    engine.cancel_active()
                    # Preserve transaction cache across cable reconnects.
                    shutdown.wait(0.5)
        finally:
            engine.cancel_active()
            # Do not close the servo UART while its worker awaits completion.
            executor.shutdown(wait=True)
            if connection is not None:
                connection.close()
            hardware.close()


if __name__ == "__main__":
    main()
