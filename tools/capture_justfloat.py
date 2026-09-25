"""Lossless JustFloat capture; --run-path explicitly permits ARM/PATH and timeout STOP."""
import argparse, csv, json, math, struct, time
from pathlib import Path
TAIL = bytes.fromhex('0000807f')
class Decoder:
    def __init__(self, channels):
        self.channels=channels; self.size=4*(channels+1); self.buf=bytearray(); self.locked=False; self.skipped=0
    def feed(self, data):
        self.buf.extend(data); rows=[]
        while True:
            if not self.locked:
                if len(self.buf)<2*self.size: break
                found=-1
                for i in range(len(self.buf)-2*self.size+1):
                    if self.buf[i+self.size-4:i+self.size]==TAIL and self.buf[i+2*self.size-4:i+2*self.size]==TAIL:
                        found=i; break
                if found<0:
                    n=len(self.buf)-2*self.size+1; self.skipped+=n; del self.buf[:n]; break
                self.skipped+=found; del self.buf[:found]; self.locked=True
            if len(self.buf)<self.size: break
            if self.buf[self.size-4:self.size]!=TAIL:
                self.locked=False; self.skipped+=1; del self.buf[:1]; continue
            rows.append(struct.unpack('<%df'%self.channels,self.buf[:self.size-4])); del self.buf[:self.size]
        return rows

def valid_runtime_frame(f):
    """Reject shifted/nonfinite telemetry before using fields to drive commands.

    JustFloat has no checksum; raw bytes remain saved even when a row is rejected.
    This is a plausibility check, not proof that every byte is intact.
    """
    if len(f) not in (38,49) or not all(math.isfinite(v) for v in f): return False
    limits={12:(0,2),14:(0,1),15:(0,65535),16:(-65535,65535),
            17:(0,16777216),18:(0,1),19:(0,1),27:(0,5),28:(0,13),
            30:(0,65535),33:(0,15),34:(0,99),36:(0,1),37:(0,1)}
    if len(f)==49: limits.update({38:(0,16777215),39:(0,16777215),
                                  41:(0,16777215),42:(0,6),46:(0,8),47:(0,16777215)})
    return 0 < f[13] <= .1 and all(lo<=f[i]<=hi and f[i]==int(f[i])
                                  for i,(lo,hi) in limits.items())

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--port',default='COM3'); ap.add_argument('--baud',type=int,default=115200)
    ap.add_argument('--channels',type=int,choices=[38,49],default=49); ap.add_argument('--duration',type=float,default=120)
    ap.add_argument('--out',required=True); ap.add_argument('--run-path',action='store_true'); a=ap.parse_args()
    import serial
    base=Path(a.out); base.parent.mkdir(parents=True,exist_ok=True)
    d=Decoder(a.channels); frames=0; stage='observe'; seq=None; deadline=0; last=None; result='duration'; gaps=0; prevseq=None
    s=serial.Serial(); s.port=a.port; s.baudrate=a.baud; s.timeout=.1; s.dtr=False; s.rts=False
    s.open(); s.reset_input_buffer(); started=time.monotonic(); armed_sent=path_sent=False; invalid_frames=0; last_valid=started
    def command(cmd):
        s.write((cmd+'\r\n').encode('ascii')); s.flush(); print('SEND',cmd,flush=True)
    try:
        with base.with_suffix('.bin').open('wb') as raw, base.with_suffix('.csv').open('w',newline='') as out:
            w=csv.writer(out); w.writerow(['host_seconds']+['CH%d'%i for i in range(a.channels)])
            while time.monotonic()-started<a.duration:
                b=s.read(s.in_waiting or 1); raw.write(b)
                for f in d.feed(b):
                    if not valid_runtime_frame(f):
                        invalid_frames+=1
                        continue
                    last_valid=time.monotonic()
                    frames+=1; w.writerow([round(time.monotonic()-started,6),*f])
                    if a.channels==49:
                        cur=int(f[39]); gaps+=0 if prevseq is None else max(0,((cur-prevseq)&0xffffff)-1); prevseq=cur
                    key=(int(f[28]),int(f[34]),int(f[27]))
                    if key!=last:
                        print('STATE step/phase/result',key,'CH20',round(f[20],4),'fault',f[15],flush=True); last=key
                    if a.run_path and stage=='observe':
                        if f[15]: raise RuntimeError('Refusing start: chassis fault')
                        if time.monotonic()-started < 2 or f[18] or f[12]!=2 or not f[19]: continue
                        seq=f[17]; command('ARM'); armed_sent=True; stage='arm'; deadline=time.monotonic()+8
                    elif stage=='arm' and f[17]!=seq:
                        if f[16]!=1 or not f[14]: raise RuntimeError('ARM rejected: host_result=%s'%f[16])
                        seq=f[17]; command('PATH'); path_sent=True; stage='path'; deadline=time.monotonic()+8
                    elif stage=='path' and f[17]!=seq:
                        if f[16]!=1: raise RuntimeError('PATH rejected: host_result=%s'%f[16])
                        stage='running'
                    if stage in ('arm','path') and time.monotonic()>deadline: raise RuntimeError('Command acknowledgement timeout')
                    if stage=='running' and (f[27]>=2 or f[15]):
                        result='mission_result_%d_fault_%d'%(f[27],f[15]); stage='tail'; deadline=time.monotonic()+2
                        if f[27]>=3 or f[15]: command('STOP')
                if stage in ('arm','path') and time.monotonic()>deadline: raise RuntimeError('Command acknowledgement timeout')
                if armed_sent and stage!='tail' and time.monotonic()-last_valid>3:
                    command('STOP'); result='telemetry_lost_or_power_off'; break
                if stage=='tail' and time.monotonic()>=deadline: break
                if frames and frames%20==0: raw.flush(); out.flush()
            if a.run_path and stage!='tail' and armed_sent:
                command('STOP')
                if result=='duration': result='timeout_stopped'
    except BaseException:
        if armed_sent:
            try: command('STOP')
            except Exception: pass
        raise
    finally:
        s.close()
        meta=dict(port=a.port,channels=a.channels,frames=frames,seconds=time.monotonic()-started,result=result,
                  invalid_frames=invalid_frames,leading_or_corrupt_bytes=d.skipped,trailing_bytes=len(d.buf),sequence_gaps=gaps if a.channels==49 else None)
        base.with_suffix('.json').write_text(json.dumps(meta,indent=2),encoding='utf-8'); print(json.dumps(meta),flush=True)
if __name__=='__main__': main()
