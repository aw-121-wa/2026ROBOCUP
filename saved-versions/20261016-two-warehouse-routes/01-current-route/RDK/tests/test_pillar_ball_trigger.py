import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from pillar_ball_trigger import PillarBallTrigger


class PillarBallTests(unittest.TestCase):
    def test_valid_ball_triggers_without_position_or_direction_window(self):
        trigger = PillarBallTrigger()
        for x in (100, 200, 280):
            result = SimpleNamespace(valid=True, bbox=(x,100,148,150), reason='ok')
            self.assertEqual(trigger.update(result), 'TRIGGER_VALID')

    def test_rejected_candidates_never_use_early_fallback(self):
        trigger = PillarBallTrigger()
        for reason in ('edge_margin','size','aspect_ratio','fill_ratio','no_candidate'):
            result = SimpleNamespace(valid=False, bbox=(92,100,100,150), reason=reason)
            self.assertIsNone(trigger.update(result))
        self.assertIsNone(trigger.update(SimpleNamespace(valid=True, bbox=None)))

if __name__ == '__main__': unittest.main()
