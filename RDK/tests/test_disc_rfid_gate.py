import sys
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from disc_rfid_gate import DiscRfidGate


class DiscRfidGateTests(unittest.TestCase):
    def test_initial_action_is_allowed(self):
        gate = DiscRfidGate(max_actions=5)
        self.assertTrue(gate.can_execute_action())
        self.assertEqual(gate.snapshot(), (True, 0, 0, 0, False))

    def test_action_completion_closes_until_matching_rfid(self):
        gate = DiscRfidGate(max_actions=5)
        self.assertTrue(gate.on_action_complete(1))
        self.assertEqual(gate.snapshot(), (False, 1, 0, 1, False))
        self.assertFalse(gate.on_rfid_confirmed(0))
        self.assertFalse(gate.on_rfid_confirmed(2))
        self.assertFalse(gate.can_execute_action())
        self.assertTrue(gate.on_rfid_confirmed(1))
        self.assertEqual(gate.snapshot(), (True, 1, 1, 0, False))

    def test_duplicate_and_out_of_order_confirmations_never_reopen_gate(self):
        gate = DiscRfidGate(max_actions=5)
        gate.on_action_complete(1)
        gate.on_rfid_confirmed(1)
        gate.on_action_complete(2)
        self.assertFalse(gate.on_rfid_confirmed(1))
        self.assertFalse(gate.on_rfid_confirmed(3))
        self.assertFalse(gate.can_execute_action())
        self.assertTrue(gate.on_rfid_confirmed(2))
        self.assertFalse(gate.on_rfid_confirmed(2))
        self.assertTrue(gate.can_execute_action())

    def test_fifth_rfid_completes_without_allowing_sixth_action(self):
        gate = DiscRfidGate(max_actions=5)
        for index in range(1, 6):
            self.assertTrue(gate.on_action_complete(index))
            self.assertFalse(gate.can_execute_action())
            self.assertTrue(gate.on_rfid_confirmed(index))
        self.assertTrue(gate.is_complete())
        self.assertFalse(gate.can_execute_action())
        self.assertFalse(gate.on_action_complete(6))


if __name__ == "__main__":
    unittest.main()
