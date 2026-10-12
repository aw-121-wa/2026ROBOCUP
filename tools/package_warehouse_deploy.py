import os
from pathlib import Path
os.chdir(Path(__file__).resolve().parents[1])
Path("build").mkdir(exist_ok=True)
from pathlib import Path
import tarfile,json,hashlib
root=Path('RDK');files=[root/'tools/rdk_stm32_bridge.py',root/'rdk_vision/warehouse_digits.py',root/'rdk_vision/warehouse_number.yaml',root/'rdk_vision/warehouse_stream.py',root/'rdk_vision/block_digits.py',root/'rdk_vision/block_digits.yaml',root/'rdk_vision/stair_low.yaml',root/'rdk_vision/pillar_runtime.yaml']+list((root/'rdk_vision/block_samples').rglob('*.png'))
with tarfile.open('build/warehouse-continuous-deploy.tar.gz','w:gz') as archive:
 for p in files: archive.add(p,arcname=p.relative_to(root).as_posix())
Path('build/warehouse-deploy-manifest.json').write_text(json.dumps({p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}))
