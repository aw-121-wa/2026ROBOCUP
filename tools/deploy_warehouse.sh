set -eu
cd /home/sunrise/licang_vision
backup="backups/warehouse-continuous-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$backup"
cp tools/rdk_stm32_bridge.py "$backup/rdk_stm32_bridge.py"
cp rdk_vision/warehouse_digits.py "$backup/warehouse_digits.py"
if [ -f rdk_vision/warehouse_stream.py ]; then cp rdk_vision/warehouse_stream.py "$backup/warehouse_stream.py"; fi
if [ -f rdk_vision/block_digits.py ]; then cp rdk_vision/block_digits.py "$backup/block_digits.py"; fi
if [ -f rdk_vision/block_digits.yaml ]; then cp rdk_vision/block_digits.yaml "$backup/block_digits.yaml"; fi
cp -a rdk_vision/block_samples "$backup/block_samples"
systemctl stop rdk-disc.service
tar -xzf /tmp/warehouse-continuous-deploy.tar.gz
chown -R sunrise:sunrise tools/rdk_stm32_bridge.py rdk_vision/warehouse_digits.py rdk_vision/warehouse_stream.py rdk_vision/block_digits.py rdk_vision/block_digits.yaml rdk_vision/block_samples
python3 - <<'PY'
import json,hashlib,py_compile
from pathlib import Path
manifest=json.loads(Path('/tmp/warehouse-deploy-manifest.json').read_text())
for name,digest in manifest.items():
    assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name
for name in manifest:
    if name.endswith('.py'): py_compile.compile(name,doraise=True)
print('Verified deployment files:',len(manifest))
PY
PYTHONPATH=.:tools python3 - <<'PY'
from pathlib import Path
import yaml
from rdk_vision.block_digits import BlockDetector
from rdk_vision.warehouse_stream import StableLocation
root=Path('rdk_vision');settings=yaml.safe_load((root/'block_digits.yaml').read_text())
for side in ('red','blue'):
    for row in (1,2,3): assert BlockDetector(root,settings[side][row]).ready
print('Six row profiles and templates ready')
PY
systemctl start rdk-disc.service
systemctl is-active rdk-disc.service
printf 'Backup: %s\n' "$backup"
