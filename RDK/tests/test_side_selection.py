import sys
import unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tools')]
from rdk_stm32_bridge import BridgeCore
class SideTests(unittest.TestCase):
    def core(self, select):
        self.lines=[]
        return BridgeCore(self.lines.append,lambda **kw:0,select_color=select)
    def test_select_both(self):
        selected=[];core=self.core(selected.append)
        core.handle('COLOR BLUE');core.handle('COLOR RED')
        self.assertEqual(selected,['blue','red'])
        self.assertEqual(self.lines,['COLOR_OK BLUE','COLOR_OK RED'])
    def test_busy_does_not_change(self):
        selected=[];core=self.core(selected.append);core._worker=object()
        core.handle('COLOR BLUE')
        self.assertEqual(selected,[]);self.assertEqual(self.lines,['COLOR_ERROR'])
    def test_missing_config_does_not_ack(self):
        def missing(color): raise FileNotFoundError(color)
        core=self.core(missing);core.handle('COLOR BLUE')
        self.assertEqual(self.lines,['COLOR_ERROR'])
    def test_unconfigured_does_not_ack(self):
        core=self.core(None);core.handle('COLOR RED')
        self.assertEqual(self.lines,['COLOR_ERROR'])
if __name__=='__main__': unittest.main()
