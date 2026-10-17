#!/usr/bin/env python3
"""Read-only pillar preview. Uses the task's actual detector and position trigger.

Stop rdk-disc.service before opening this tool. No serial ports or actions used.
"""
import argparse
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2
from pillar_profile import load_pillar_profile
from rdk_vision.config import load_config
from rdk_vision.ball_detector import BallDetector
from rdk_vision.camera import LatestFrameCamera


def draw_preview(frame, config, trigger, result, event, stale):
    display = frame.copy()
    roi = config.roi
    cv2.rectangle(display, (roi.x, roi.y), (roi.x + roi.width, roi.y + roi.height), (0,255,255), 2)
    cx, cy = trigger.center_x, trigger.center_y
    tx, ty = trigger.tolerance_x, trigger.tolerance_y
    cv2.rectangle(display, (round(cx-tx), round(cy-ty)), (round(cx+tx), round(cy+ty)), (255,255,0), 2)
    cv2.drawMarker(display, (round(cx), round(cy)), (255,255,0), cv2.MARKER_CROSS, 16, 1)
    if result.bbox is not None:
        x,y,w,h = result.bbox
        color = (0,255,0) if result.valid else (0,0,255)
        cv2.rectangle(display, (x,y), (x+w,y+h), color, 2)
        cv2.circle(display, (round(x+w/2),round(y+h/2)), 4, color, -1)
    state = 'STALE - NO TRIGGER' if stale else 'STOP POSITION (preview only)' if event else 'WAIT POSITION'
    for i, line in enumerate((state, f'reason={result.reason} bbox={result.bbox}',
                             'yellow=ROI cyan=grab window; q/ESC=quit')):
        cv2.putText(display, line, (10,25+i*24), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
    return display


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'rdk_vision/config.yaml'))
    parser.add_argument('--profile', default=str(ROOT / 'rdk_vision/pillar.yaml'))
    args = parser.parse_args()
    config, trigger = load_pillar_profile(load_config(args.config), args.profile)
    detector = BallDetector(config)
    camera = LatestFrameCamera(config.camera)
    try:
        camera.start()
        if not camera.wait_until_ready(config.camera.startup_timeout_ms):
            raise RuntimeError('camera not ready')
        while True:
            snapshot = camera.get_latest()
            if snapshot is not None:
                stale = (time.monotonic()-snapshot.timestamp)*1000 > config.camera.stale_ms
                result = detector.detect(snapshot.frame, 'red')
                event = None if stale else trigger.update(result)
                cv2.imshow('Pillar preview - NO ACTION', draw_preview(snapshot.frame, config, trigger, result, event, stale))
            if cv2.waitKey(10) & 0xff in (27, ord('q')):
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
