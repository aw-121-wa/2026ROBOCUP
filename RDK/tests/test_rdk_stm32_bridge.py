import sys
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
for path in (ROOT, TOOLS):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from rdk_stm32_bridge import BridgeCore, SerialLineWriter, build_disc_arguments, normalize_command


class BridgeCommandTests(unittest.TestCase):
    def test_normalize_command_accepts_crlf_and_spaces(self):
        self.assertEqual(normalize_command(b"  disc_start\r\n"), "DISC_START")
        self.assertEqual(normalize_command(b"PING\n"), "PING")

    def test_disc_arguments_preserve_validated_defaults(self):
        args = build_disc_arguments(Path("/home/sunrise/licang_vision"))
        self.assertEqual(Path(args.config), Path("/home/sunrise/licang_vision/rdk_vision/config.yaml"))
        self.assertEqual((args.servo_port, args.servo_baud), ("/dev/ttyS1", 9600))
        self.assertEqual((args.prep_group, args.trigger_group), (101, 102))
        self.assertEqual((args.max_actions, args.trigger_x), (5, 380))
        self.assertEqual(args.servo_timeout, 30.0)


class BridgeCoreTests(unittest.TestCase):
    def test_overall_timeout_cancels_gate_and_stops_worker(self):
        tx, entered = [], threading.Event()
        now = [100.0]

        def run_disc(*, rfid_gate, on_action_complete):
            del on_action_complete
            entered.set()
            deadline = time.monotonic() + 1.0
            while not rfid_gate.is_cancelled():
                if time.monotonic() >= deadline:
                    return 9
                time.sleep(0.001)
            return 1

        core = BridgeCore(tx.append, run_disc, clock=lambda: now[0], disc_timeout_s=60.0)
        core.handle("DISC_START"); self.assertTrue(entered.wait(0.5))
        now[0] = 159.999; core.tick(); self.assertTrue(core.disc_active)
        now[0] = 160.0; core.tick()
        self.assertTrue(core.wait_for_idle(1.0)); self.assertEqual(tx[-1], "DISC_ERROR")

    def test_disc_cancel_closes_open_action_gate(self):
        tx, entered = [], threading.Event()

        def run_disc(*, rfid_gate, on_action_complete):
            del on_action_complete
            entered.set()
            deadline = time.monotonic() + 1.0
            while not rfid_gate.is_cancelled():
                if time.monotonic() >= deadline: return 9
                time.sleep(0.001)
            return 1

        core = BridgeCore(tx.append, run_disc)
        core.handle("DISC_START"); self.assertTrue(entered.wait(0.5))
        self.assertTrue(core.gate.can_execute_action())
        cancelled_at = time.monotonic(); core.handle("DISC_CANCEL")
        self.assertTrue(core.wait_for_idle(1.0)); self.assertEqual(tx[-1], "DISC_ERROR")
        self.assertLess(time.monotonic() - cancelled_at, 0.2)

    def test_disc_start_is_nonblocking_and_ping_works_while_active(self):
        tx, entered, release = [], threading.Event(), threading.Event()

        def run_disc(**_kwargs):
            entered.set(); release.wait(1.0); return 7

        core = BridgeCore(send_line=tx.append, run_disc=run_disc)
        started_at = time.monotonic(); core.handle("DISC_START")
        self.assertLess(time.monotonic() - started_at, 0.2)
        self.assertTrue(entered.wait(0.5))
        core.handle("PING")
        self.assertEqual(tx[:2], ["DISC_ACK", "PONG"])
        release.set(); self.assertTrue(core.wait_for_idle(1.0))
        self.assertEqual(tx[-1], "DISC_ERROR")

    def test_active_worker_receives_only_matching_rfid_confirmation(self):
        tx, action_done, allow_finish = [], threading.Event(), threading.Event()

        def run_disc(*, rfid_gate, on_action_complete):
            self.assertTrue(rfid_gate.on_action_complete(1)); on_action_complete(1)
            action_done.set(); self.assertTrue(allow_finish.wait(1.0)); return 7

        core = BridgeCore(send_line=tx.append, run_disc=run_disc)
        core.handle("DISC_START"); self.assertTrue(action_done.wait(0.5))
        core.handle("DISC_RFID_OK 2"); self.assertFalse(core.gate.can_execute_action())
        core.handle("DISC_RFID_OK 1"); self.assertTrue(core.gate.can_execute_action())
        core.handle("DISC_RFID_OK 1"); self.assertTrue(core.gate.can_execute_action())
        allow_finish.set(); self.assertTrue(core.wait_for_idle(1.0))
        self.assertIn("DISC_ACTION_DONE 1", tx)

    def test_duplicate_disc_start_does_not_start_second_worker(self):
        entered, release, calls, tx = threading.Event(), threading.Event(), [], []

        def run_disc(**_kwargs):
            calls.append(1); entered.set(); release.wait(1.0); return 7

        core = BridgeCore(tx.append, run_disc)
        core.handle("DISC_START"); self.assertTrue(entered.wait(0.5))
        core.handle("DISC_START")
        self.assertEqual(calls, [1]); self.assertEqual(tx.count("DISC_ACK"), 1)
        release.set(); self.assertTrue(core.wait_for_idle(1.0))

    def test_shutdown_waits_for_old_worker_before_reconnect_can_continue(self):
        entered, release, shutdown_done = threading.Event(), threading.Event(), threading.Event()

        def run_disc(**_kwargs):
            entered.set(); release.wait(1.0); return 7

        core = BridgeCore(lambda _line: None, run_disc)
        core.handle("DISC_START"); self.assertTrue(entered.wait(0.5))
        thread = threading.Thread(target=lambda: (core.shutdown(), shutdown_done.set()))
        thread.start(); self.assertFalse(shutdown_done.wait(0.05))
        release.set(); self.assertTrue(shutdown_done.wait(0.5)); thread.join()

    def test_terminal_send_failure_still_clears_active_worker(self):
        def send_line(line):
            if line == "DISC_ERROR":
                raise OSError("serial disconnected")

        core = BridgeCore(send_line, lambda **_kwargs: 7)
        core.handle("DISC_START")
        self.assertTrue(core.wait_for_idle(1.0))
        self.assertFalse(core.disc_active)

    def test_success_requires_fifth_rfid_then_sends_done(self):
        tx = []
        ready = [threading.Event() for _ in range(5)]

        def run_disc(*, rfid_gate, on_action_complete):
            for index in range(1, 6):
                self.assertTrue(rfid_gate.on_action_complete(index)); on_action_complete(index)
                ready[index - 1].set(); deadline = time.monotonic() + 1.0
                while not (rfid_gate.is_complete() if index == 5 else rfid_gate.can_execute_action()):
                    if time.monotonic() >= deadline: return 9
                    time.sleep(0.001)
            return 0

        core = BridgeCore(tx.append, run_disc); core.handle("DISC_START")
        for index, event in enumerate(ready, 1):
            self.assertTrue(event.wait(0.5)); self.assertNotIn("DISC_DONE", tx)
            core.handle(f"DISC_RFID_OK {index}")
        self.assertTrue(core.wait_for_idle(1.0)); self.assertEqual(tx[-1], "DISC_DONE")
        self.assertEqual([x for x in tx if x.startswith("DISC_ACTION_DONE")],
                         [f"DISC_ACTION_DONE {i}" for i in range(1, 6)])


class SlowSerial:
    def __init__(self): self.output = bytearray()
    def write(self, payload):
        for byte in payload:
            self.output.append(byte); time.sleep(0.0001)
        return len(payload)
    def flush(self): pass


class SerialWriterTests(unittest.TestCase):
    def test_concurrent_lines_never_interleave_bytes(self):
        serial = SlowSerial(); send_line = SerialLineWriter(serial, log=lambda _text: None)
        threads = [threading.Thread(target=send_line, args=(line,))
                   for line in ("PONG", "DISC_ACTION_DONE 1")]
        for thread in threads: thread.start()
        for thread in threads: thread.join()
        self.assertCountEqual(serial.output.decode("ascii").splitlines(),
                              ["PONG", "DISC_ACTION_DONE 1"])

    def test_write_failure_is_visible_to_serial_service_loop(self):
        class FailingSerial:
            def write(self, _payload): raise OSError("disconnected")
            def flush(self): pass

        send_line = SerialLineWriter(FailingSerial(), log=lambda _text: None)
        with self.assertRaises(OSError): send_line("DISC_ERROR")
        with self.assertRaisesRegex(RuntimeError, "serial TX failed"):
            send_line.raise_if_failed()


class SystemdPackagingTests(unittest.TestCase):
    def test_service_template_autostarts_bridge_with_expected_ports(self):
        text = (ROOT / "systemd" / "rdk-disc.service.in").read_text(encoding="utf-8")
        self.assertIn("WantedBy=multi-user.target", text); self.assertIn("Restart=always", text)
        self.assertIn("--stm32-port /dev/ttyUSB0", text); self.assertIn("--stm32-baud 115200", text)
        self.assertIn("SupplementaryGroups=dialout video", text)

    def test_install_script_enables_service(self):
        text = (ROOT / "install_service.sh").read_text(encoding="utf-8")
        self.assertIn("systemctl enable --now rdk-disc.service", text)
        self.assertIn("systemctl daemon-reload", text)


if __name__ == "__main__": unittest.main()
