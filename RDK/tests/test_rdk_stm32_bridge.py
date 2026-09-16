import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / 'tools'
for p in (ROOT, TOOLS):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from rdk_stm32_bridge import build_disc_command, normalize_command, BridgeCore


class BridgeCommandTests(unittest.TestCase):
    def test_normalize_command_accepts_crlf_and_spaces(self):
        self.assertEqual(normalize_command(b'  disc_start\r\n'), 'DISC_START')
        self.assertEqual(normalize_command(b'PING\n'), 'PING')

    def test_disc_command_preserves_validated_disc_defaults(self):
        cmd = build_disc_command(Path('/home/sunrise/licang_vision'))
        joined = ' '.join(cmd)
        self.assertEqual(Path(cmd[cmd.index('--config') + 1]), Path('/home/sunrise/licang_vision/rdk_vision/config.yaml'))
        self.assertIn('--servo-port /dev/ttyS1', joined)
        self.assertIn('--servo-baud 9600', joined)
        self.assertIn('--prep-group 101', joined)
        self.assertIn('--trigger-group 102', joined)
        self.assertIn('--max-actions 5', joined)
        self.assertIn('--trigger-x 380', joined)
        self.assertIn('--servo-timeout 30', joined)


class BridgeCoreTests(unittest.TestCase):
    def test_actual_serial_bytearray_lines(self):
        tx = []
        core = BridgeCore(send_line=tx.append, run_disc=lambda: 0)
        for line in (b"PING\r\n", b"DISC_START\r\n"):
            raw, _, _ = bytearray(line).partition(b"\n")
            core.handle(normalize_command(raw.rstrip(b"\r")))
        self.assertEqual(tx, ["PONG", "DISC_ACK", "DISC_DONE"])

    def test_ping_replies_pong(self):
        tx = []
        core = BridgeCore(send_line=tx.append, run_disc=lambda: 0)
        core.handle('PING')
        self.assertEqual(tx, ['PONG'])

    def test_disc_success_replies_ack_then_done(self):
        tx = []
        core = BridgeCore(send_line=tx.append, run_disc=lambda: 0)
        core.handle('DISC_START')
        self.assertEqual(tx, ['DISC_ACK', 'DISC_DONE'])

    def test_disc_failure_replies_ack_then_error(self):
        tx = []
        core = BridgeCore(send_line=tx.append, run_disc=lambda: 7)
        core.handle('DISC_START')
        self.assertEqual(tx, ['DISC_ACK', 'DISC_ERROR'])

    def test_unknown_command_returns_error(self):
        tx = []
        core = BridgeCore(send_line=tx.append, run_disc=lambda: 0)
        core.handle('NOPE')
        self.assertEqual(tx, ['ERR_UNKNOWN'])


if __name__ == '__main__':
    unittest.main()

class SystemdPackagingTests(unittest.TestCase):
    def test_service_template_autostarts_bridge_with_expected_ports(self):
        text = (ROOT / 'systemd' / 'rdk-disc.service.in').read_text(encoding='utf-8')
        self.assertIn('WantedBy=multi-user.target', text)
        self.assertIn('Restart=always', text)
        self.assertIn('--stm32-port /dev/ttyUSB0', text)
        self.assertIn('--stm32-baud 115200', text)
        self.assertIn('SupplementaryGroups=dialout video', text)

    def test_install_script_enables_service(self):
        text = (ROOT / 'install_service.sh').read_text(encoding='utf-8')
        self.assertIn('systemctl enable --now rdk-disc.service', text)
        self.assertIn('systemctl daemon-reload', text)
