"""The previous deployment entry point must delegate to the unchanged ZHY bridge."""
from pathlib import Path
import subprocess
import sys
import unittest

class LauncherTests(unittest.TestCase):
    def test_help_uses_zhy_bridge_options(self):
        script = Path(__file__).resolve().parents[1] / "tools/rdk/path_bridge.py"
        result = subprocess.run([sys.executable, str(script), "--help"],
                                capture_output=True, text=True, check=True)
        self.assertIn("--stm32-port", result.stdout)
        self.assertIn("--stm32-baud", result.stdout)
        self.assertIn("--project-root", result.stdout)

if __name__ == "__main__":
    unittest.main()
