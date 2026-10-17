import sys
import unittest
import contextlib
import io
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
for path in (ROOT, TOOLS):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from disc_rfid_gate import DiscRfidGate
from vision_servo_direct_test import run_disc_task


ELIGIBLE = ("TRIGGER_VALID", "TRIGGER_EARLY", "TRIGGER_EDGE")


class FakeCamera:
    def __init__(self, frames, before_frame=None):
        self.frames = list(frames)
        self.before_frame = before_frame or {}
        self.index = 0
        self.stopped = False

    def start(self):
        pass

    def wait_until_ready(self, _timeout_ms):
        return True

    def get_latest(self):
        if self.index >= len(self.frames):
            return self.frames[-1] if self.frames else None
        callback = self.before_frame.get(self.index)
        if callback:
            callback()
        value = self.frames[self.index]
        self.index += 1
        return value

    def stop(self):
        self.stopped = True


class FakeDetector:
    def __init__(self):
        self.calls = 0

    def detect(self, _frame, _color):
        self.calls += 1
        return SimpleNamespace(
            valid=True,
            reason="ok",
            bbox=(275, 150, 152, 150),
            aspect_ratio=1.0,
            fill_ratio=0.68,
        )


class FakeTrigger:
    state = "ARMED"

    def __init__(self, events):
        self.events = iter(events)
        self.calls = 0

    def update(self, _result):
        self.calls += 1
        return next(self.events, None)


class FakeBoard:
    def __init__(self):
        self.started = []
        self.waited = []
        self.closed = False

    def open(self):
        pass

    def run_group(self, group, *, repeat_count, timeout_s):
        return SimpleNamespace(raw=bytes([group]))

    def start_group(self, group, repeat_count=1):
        self.started.append((group, repeat_count))
        return b"tx"

    def wait_group_complete(self, group, timeout_s):
        self.waited.append((group, timeout_s))
        return SimpleNamespace(raw=b"complete")

    def close(self):
        self.closed = True


def snapshot(frame_id, timestamp=None):
    return SimpleNamespace(
        frame=SimpleNamespace(shape=(480, 640, 3)), frame_id=frame_id,
        timestamp=float(frame_id if timestamp is None else timestamp),
    )


def args(max_actions=5):
    return SimpleNamespace(
        config="unused.yaml", color="red", servo_port="/dev/ttyS1",
        servo_baud=9600, prep_group=101, trigger_group=102, repeat=1,
        servo_timeout=30.0, clear_frames=2, max_actions=max_actions,
        trigger_x=380, trigger_visible_scale=0.70, edge_tolerance_px=5,
        early_min_width_scale=0.60, early_min_height_scale=0.65,
    )


def config(stale_ms=250):
    return SimpleNamespace(
        camera=SimpleNamespace(startup_timeout_ms=5000, stale_ms=stale_ms),
        roi=SimpleNamespace(x=92, width=352),
        ball=SimpleNamespace(
            reference_width=148, reference_height=150,
            aspect_ratio_max=1.3, fill_ratio_min=0.5, fill_ratio_max=0.9,
        ),
    )


class DiscTaskLoopTests(unittest.TestCase):
    def run_task(self, events, frames, *, gate=None, before_frame=None, now=None, max_actions=5):
        camera = FakeCamera(frames, before_frame)
        detector = FakeDetector()
        trigger = FakeTrigger(events)
        board = FakeBoard()
        completed = []
        with contextlib.redirect_stdout(io.StringIO()):
            rc = run_disc_task(
                args(max_actions), config=config(), detector=detector,
                trigger=trigger, camera=camera, board=board, rfid_gate=gate,
                on_action_complete=completed.append,
                clock=(lambda: now[0]) if now is not None else (lambda: float(camera.index - 1)),
                sleep_fn=lambda _seconds: None,
                max_idle_iterations=10,
            )
        return rc, camera, detector, trigger, board, completed

    def test_wait_rfid_processes_100_frames_and_suppresses_every_trigger_kind(self):
        gate = DiscRfidGate(5)
        events = ["TRIGGER_VALID"] + [ELIGIBLE[i % 3] for i in range(100)]
        frames = [snapshot(i) for i in range(103)]
        rc, _camera, detector, trigger, board, completed = self.run_task(events, frames, gate=gate)
        self.assertNotEqual(rc, 0)
        self.assertEqual(detector.calls, 101)
        self.assertEqual(trigger.calls, 101)
        self.assertEqual(len(board.started), 1)
        self.assertEqual(completed, [1])
        self.assertEqual(gate.waiting_rfid_index, 1)

    def test_matching_rfid_allows_action_two_on_first_fresh_eligible_frame(self):
        gate = DiscRfidGate(5)
        frames = [snapshot(i) for i in range(5)]
        before = {3: lambda: gate.on_rfid_confirmed(1, confirmed_at=3.0)}
        rc, _camera, detector, _trigger, board, completed = self.run_task(
            ["TRIGGER_VALID", "TRIGGER_VALID", "TRIGGER_VALID"],
            frames, gate=gate, before_frame=before,
        )
        self.assertNotEqual(rc, 0)
        self.assertEqual(detector.calls, 2)
        self.assertEqual(len(board.started), 2)
        self.assertEqual(completed, [1, 2])

    def test_old_waiting_trigger_is_not_latched_when_reopened_without_new_eligible_frame(self):
        gate = DiscRfidGate(5)
        frames = [snapshot(i) for i in range(6)]
        before = {4: lambda: gate.on_rfid_confirmed(1, confirmed_at=4.0)}
        _rc, _camera, _detector, _trigger, board, _completed = self.run_task(
            ["TRIGGER_VALID", "TRIGGER_VALID", None], frames,
            gate=gate, before_frame=before,
        )
        self.assertEqual(len(board.started), 1)

    def test_stale_frame_after_rfid_confirmation_cannot_execute_action(self):
        gate = DiscRfidGate(5)
        now = [10.0]
        frames = [snapshot(0, 10.0), snapshot(1, 10.0), snapshot(2, 9.0)]
        before = {2: lambda: gate.on_rfid_confirmed(1, confirmed_at=10.0)}
        _rc, _camera, detector, trigger, board, _completed = self.run_task(
            ["TRIGGER_VALID"], frames, gate=gate, before_frame=before, now=now,
        )
        self.assertEqual(len(board.started), 1)
        self.assertEqual(detector.calls, 1)
        self.assertEqual(trigger.calls, 1)

    def test_nonstale_frame_captured_before_rfid_confirmation_cannot_execute_action(self):
        gate = DiscRfidGate(5)
        now = [10.0]
        frames = [snapshot(0, 10.0), snapshot(1, 10.0),
                  snapshot(2, 10.0), snapshot(3, 9.9)]
        before = {3: lambda: gate.on_rfid_confirmed(1, confirmed_at=10.0)}
        _rc, _camera, detector, trigger, board, _completed = self.run_task(
            ["TRIGGER_VALID", "TRIGGER_VALID"], frames,
            gate=gate, before_frame=before, now=now,
        )
        self.assertEqual(len(board.started), 1)
        self.assertEqual(detector.calls, 2)
        self.assertEqual(trigger.calls, 2)

    def test_action_five_waits_while_vision_runs_then_rfid_five_finishes_without_action_six(self):
        gate = DiscRfidGate(5)
        frames = [snapshot(i) for i in range(20)]
        before = {
            3: lambda: gate.on_rfid_confirmed(1, confirmed_at=3.0),
            5: lambda: gate.on_rfid_confirmed(2, confirmed_at=5.0),
            7: lambda: gate.on_rfid_confirmed(3, confirmed_at=7.0),
            9: lambda: gate.on_rfid_confirmed(4, confirmed_at=9.0),
            15: lambda: gate.on_rfid_confirmed(5, confirmed_at=15.0),
        }
        rc, _camera, detector, trigger, board, completed = self.run_task(
            ["TRIGGER_VALID"] * 18, frames, gate=gate, before_frame=before,
        )
        self.assertEqual(rc, 0)
        self.assertEqual(completed, [1, 2, 3, 4, 5])
        self.assertEqual(len(board.started), 5)
        self.assertGreater(detector.calls, 5)
        self.assertEqual(trigger.calls, detector.calls)
        self.assertFalse(gate.can_execute_action())

    def test_standalone_without_gate_keeps_immediate_rearm_for_five_actions(self):
        frames = [snapshot(i) for i in range(11)]
        rc, _camera, _detector, _trigger, board, completed = self.run_task(
            ["TRIGGER_VALID"] * 5, frames, gate=None,
        )
        self.assertEqual(rc, 0)
        self.assertEqual(len(board.started), 5)
        self.assertEqual(completed, [1, 2, 3, 4, 5])


if __name__ == "__main__":
    unittest.main()
