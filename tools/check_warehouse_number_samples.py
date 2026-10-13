"""Check labeled warehouse camera captures with the production detector.

Run: PYTHONPATH=RDK python3 tools/check_warehouse_number_samples.py SAMPLE_ROOT
Labels are 1/2/3/empty, optionally followed by timestamp directories.
"""
import argparse
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'RDK'))
import cv2
from rdk_vision.warehouse_digits import DigitDetector


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('samples', type=Path)
    args = parser.parse_args()
    detector = DigitDetector()
    counts = Counter()
    failures = []
    for path in sorted(args.samples.rglob('*.png')):
        labels = [part for part in path.relative_to(args.samples).parts[:-1]
                  if part in ('1', '2', '3', 'empty')]
        if len(labels) != 1:
            continue
        expected = 0 if labels[0] == 'empty' else int(labels[0])
        frame = cv2.imread(str(path))
        if frame is None:
            failures.append((str(path), 'unreadable'))
            continue
        actual, score = detector.detect(frame)
        counts['total'] += 1
        counts['correct'] += actual == expected
        if actual != expected:
            failures.append((str(path), f'expected={expected} actual={actual} score={score:.3f}'))
    print(f"Correct: {counts['correct']}/{counts['total']}")
    for path, reason in failures:
        print(f'{path}: {reason}')
    return 1 if failures or not counts['total'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
