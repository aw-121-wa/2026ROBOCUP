import sys
from pathlib import Path
import unittest
from types import SimpleNamespace

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))

from disc_rtl_trigger import RightToLeftDiscTrigger, derive_trigger_x


def result(*, valid=False, reason='no_candidate', bbox=None, aspect=0.0, fill=0.0):
    return SimpleNamespace(
        valid=valid,
        reason=reason,
        bbox=bbox,
        aspect_ratio=aspect,
        fill_ratio=fill,
    )


class RightToLeftDiscTriggerTests(unittest.TestCase):
    def test_derive_trigger_x_uses_visible_width_ratio(self):
        self.assertEqual(derive_trigger_x(435, 152, 0.70), 329)

    def make_gate(self):
        return RightToLeftDiscTrigger(
            roi_right=435,
            trigger_x=330,
            reference_width=152,
            reference_height=150,
            clear_frames=2,  # accepted for CLI compatibility, ignored by new gate
            edge_tolerance_px=5,
            min_visible_width_scale=0.60,
            min_height_scale=0.65,
            aspect_min=0.55,
            aspect_max=1.35,
            fill_min=0.45,
            fill_max=0.95,
        )

    def test_partial_ball_entering_from_right_triggers_before_full_valid(self):
        gate = self.make_gate()
        partial = result(
            valid=False,
            reason='edge_margin',
            bbox=(328, 150, 107, 145),
            aspect=107/145,
            fill=0.66,
        )
        self.assertEqual(gate.update(partial), 'TRIGGER_EARLY')
        self.assertEqual(gate.state, 'ARMED')

    def test_bottom_edge_nearly_complete_ball_triggers(self):
        gate = self.make_gate()
        bottom_edge = result(
            valid=False,
            reason='edge_margin',
            bbox=(212, 161, 150, 134),
            aspect=1.119,
            fill=0.622,
        )
        self.assertEqual(gate.update(bottom_edge), 'TRIGGER_EDGE')
        self.assertEqual(gate.state, 'ARMED')

    def test_left_edge_small_candidate_does_not_trigger(self):
        gate = self.make_gate()
        left_edge = result(
            valid=False,
            reason='edge_margin',
            bbox=(195, 150, 110, 145),
            aspect=110/145,
            fill=0.66,
        )
        self.assertIsNone(gate.update(left_edge))
        self.assertEqual(gate.state, 'ARMED')

    def test_normal_valid_target_is_fallback_trigger(self):
        gate = self.make_gate()
        full = result(
            valid=True,
            reason='ok',
            bbox=(275, 150, 152, 150),
            aspect=1.01,
            fill=0.68,
        )
        self.assertEqual(gate.update(full), 'TRIGGER_VALID')
        self.assertEqual(gate.state, 'ARMED')

    def test_trigger_rearms_immediately_without_clear_frames(self):
        gate = self.make_gate()
        full = result(
            valid=True,
            reason='ok',
            bbox=(275, 150, 152, 150),
            aspect=1.0,
            fill=0.68,
        )
        self.assertEqual(gate.update(full), 'TRIGGER_VALID')
        # No WAIT_CLEAR state anymore: after G102 completion the caller can
        # immediately feed the next fresh frame and trigger again.
        self.assertEqual(gate.state, 'ARMED')
        self.assertEqual(gate.update(full), 'TRIGGER_VALID')

    def test_too_small_right_edge_sliver_does_not_trigger(self):
        gate = self.make_gate()
        sliver = result(
            valid=False,
            reason='edge_margin',
            bbox=(390, 160, 45, 125),
            aspect=45/125,
            fill=0.65,
        )
        self.assertIsNone(gate.update(sliver))

    def test_large_non_ball_edge_candidate_is_rejected(self):
        gate = self.make_gate()
        machine_part = result(
            valid=False,
            reason='edge_margin',
            bbox=(200, 130, 230, 175),
            aspect=1.314,
            fill=0.70,
        )
        self.assertIsNone(gate.update(machine_part))


if __name__ == '__main__':
    unittest.main()
