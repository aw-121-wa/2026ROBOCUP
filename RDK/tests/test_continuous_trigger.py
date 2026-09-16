import sys
from pathlib import Path
import unittest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))

from continuous_trigger import ContinuousTriggerGate


class ContinuousTriggerGateTests(unittest.TestCase):
    def test_two_hits_in_three_triggers_once_then_waits_for_clear(self):
        gate = ContinuousTriggerGate(window_frames=3, required_hits=2, clear_frames=2)

        self.assertIsNone(gate.update(True))
        self.assertIsNone(gate.update(False))
        self.assertEqual(gate.update(True), 'TRIGGER')
        self.assertEqual(gate.state, 'WAIT_CLEAR')

        # Keeping the target present must never retrigger.
        self.assertIsNone(gate.update(True))
        self.assertIsNone(gate.update(True))
        self.assertEqual(gate.state, 'WAIT_CLEAR')

    def test_requires_two_consecutive_invalid_frames_before_rearm(self):
        gate = ContinuousTriggerGate(window_frames=3, required_hits=2, clear_frames=2)
        gate.update(True)
        self.assertEqual(gate.update(True), 'TRIGGER')

        self.assertIsNone(gate.update(False))
        self.assertEqual(gate.state, 'WAIT_CLEAR')

        # A valid frame resets the consecutive-clear count.
        self.assertIsNone(gate.update(True))
        self.assertIsNone(gate.update(False))
        self.assertEqual(gate.update(False), 'REARMED')
        self.assertEqual(gate.state, 'ARMED')

    def test_after_rearm_a_new_target_can_trigger_again(self):
        gate = ContinuousTriggerGate(window_frames=3, required_hits=2, clear_frames=2)
        gate.update(True)
        self.assertEqual(gate.update(True), 'TRIGGER')
        gate.update(False)
        self.assertEqual(gate.update(False), 'REARMED')

        self.assertIsNone(gate.update(True))
        self.assertEqual(gate.update(True), 'TRIGGER')

    def test_invalid_frames_while_armed_do_not_accumulate_hits(self):
        gate = ContinuousTriggerGate(window_frames=3, required_hits=2, clear_frames=2)
        for _ in range(5):
            self.assertIsNone(gate.update(False))
        self.assertEqual(gate.state, 'ARMED')


if __name__ == '__main__':
    unittest.main()
