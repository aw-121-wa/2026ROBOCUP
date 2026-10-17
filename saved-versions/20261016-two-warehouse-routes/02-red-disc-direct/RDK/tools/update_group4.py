#!/usr/bin/env python3
"""Enable GROUP 4 without replacing on-device vision customizations."""
import ast
import hashlib
import shutil
from datetime import datetime
from pathlib import Path


def main():
    root = Path('/home/sunrise/licang_vision')
    target = root / 'tools/rdk_stm32_bridge.py'
    original = target.read_bytes()
    old = b"r'GROUP (0|1|2|3|100|105|109|110|111)'"
    new = b"r'GROUP (0|1|2|3|4|100|105|109|110|111)'"
    if original.count(new) == 1 and old not in original:
        print('GROUP 4 already enabled; no changes.')
        return
    if original.count(old) != 1 or new in original:
        raise SystemExit('Unexpected GROUP parser; no files changed. Inspect bridge manually.')
    protected = list((root / 'rdk_vision').glob('*.yaml'))
    protected.append(root / 'tools/stair_task.py')
    hashes = {p: hashlib.sha256(p.read_bytes()).digest() for p in protected}
    updated = original.replace(old, new, 1)
    ast.parse(updated.decode('utf-8-sig'))
    backup = target.with_name(target.name + '.before-group4-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    shutil.copy2(target, backup)
    target.write_bytes(updated)
    assert all(hashlib.sha256(p.read_bytes()).digest() == h for p, h in hashes.items())
    print('Updated:', target)
    print('Backup:', backup)
    print('Only GROUP 4 allowance changed. YAML files and stair_task.py unchanged.')


if __name__ == '__main__':
    main()
