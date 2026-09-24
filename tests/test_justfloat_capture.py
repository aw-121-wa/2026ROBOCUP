import importlib.util,struct,unittest
from pathlib import Path
spec=importlib.util.spec_from_file_location('capture',Path(__file__).parents[1]/'tools/capture_justfloat.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class TestDecoder(unittest.TestCase):
    def test_chunks_and_embedded_tail(self):
        frame=bytearray(struct.pack('<38f',*range(38))+m.TAIL);frame[21:25]=m.TAIL
        data=b'garbage'+frame*4; d=m.Decoder(38); rows=[]
        for i in range(0,len(data),17): rows+=d.feed(data[i:i+17])
        self.assertEqual(len(rows),4); self.assertEqual(d.skipped,7); self.assertFalse(d.buf)
    def test_new_channels_and_resync(self):
        frame=struct.pack('<49f',*range(49))+m.TAIL; d=m.Decoder(49)
        rows=d.feed(frame*2+b'x'+frame*2)
        self.assertEqual(len(rows),4); self.assertEqual(rows[-1][48],48);self.assertEqual(d.skipped,1)
if __name__=='__main__': unittest.main()
