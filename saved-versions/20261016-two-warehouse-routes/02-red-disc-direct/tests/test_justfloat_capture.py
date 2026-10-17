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
    def test_runtime_frame_validation(self):
        f=[0.0]*49; f[12]=2; f[13]=.005; f[19]=1
        self.assertTrue(m.valid_runtime_frame(f))
        f[15]=-1.05928
        self.assertFalse(m.valid_runtime_frame(f))
        f[15]=2
        self.assertTrue(m.valid_runtime_frame(f)) # Genuine faults must reach STOP handling.
        f[13]=180
        self.assertFalse(m.valid_runtime_frame(f))
        f[13]=.005; f[7]=float('inf')
        self.assertFalse(m.valid_runtime_frame(f))
if __name__=='__main__': unittest.main()
