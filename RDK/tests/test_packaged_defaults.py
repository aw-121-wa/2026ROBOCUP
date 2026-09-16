from pathlib import Path
import unittest


class PackagedDefaultsTests(unittest.TestCase):
    def test_disc_trigger_x_default_is_380(self):
        path = Path(__file__).resolve().parents[1] / "tools" / "vision_servo_direct_test.py"
        text = path.read_text(encoding="utf-8")
        self.assertIn("DEFAULT_TRIGGER_X = 380", text)
        self.assertIn("default=DEFAULT_TRIGGER_X", text)


if __name__ == "__main__":
    unittest.main()
