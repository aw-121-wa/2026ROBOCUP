import argparse
import logging

import cv2

from rdk_vision.ball_detector import BallDetector
from rdk_vision.camera import LatestFrameCamera
from rdk_vision.config import RoiConfig, load_config, save_config, with_calibration

WINDOW = "RDK Calibration"
MASK_WINDOW = "RDK Mask"


def _noop(_value):
    pass


def _create_trackbars(config, color):
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.namedWindow(MASK_WINDOW, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("ROI_X", WINDOW, config.roi.x, config.camera.width - 1, _noop)
    cv2.createTrackbar("ROI_Y", WINDOW, config.roi.y, config.camera.height - 1, _noop)
    cv2.createTrackbar("ROI_W", WINDOW, config.roi.width, config.camera.width, _noop)
    cv2.createTrackbar("ROI_H", WINDOW, config.roi.height, config.camera.height, _noop)
    for name, maximum in (
        ("H1_MIN", 179), ("H1_MAX", 179),
        ("S1_MIN", 255), ("S1_MAX", 255),
        ("V1_MIN", 255), ("V1_MAX", 255),
        ("H2_MIN", 179), ("H2_MAX", 179),
        ("S2_MIN", 255), ("S2_MAX", 255),
        ("V2_MIN", 255), ("V2_MAX", 255),
    ):
        cv2.createTrackbar(name, WINDOW, 0, maximum, _noop)
    _set_color_trackbars(config, color)


def _set_trackbar(name, value):
    cv2.setTrackbarPos(name, WINDOW, int(value))


def _set_color_trackbars(config, color):
    ranges = config.colors[color]
    first = ranges[0]
    second = ranges[1] if len(ranges) > 1 else (0, 0, 0, 0, 0, 0)
    for prefix, values in (("1", first), ("2", second)):
        low_h, low_s, low_v, high_h, high_s, high_v = values
        _set_trackbar(f"H{prefix}_MIN", low_h)
        _set_trackbar(f"H{prefix}_MAX", high_h)
        _set_trackbar(f"S{prefix}_MIN", low_s)
        _set_trackbar(f"S{prefix}_MAX", high_s)
        _set_trackbar(f"V{prefix}_MIN", low_v)
        _set_trackbar(f"V{prefix}_MAX", high_v)


def _range_from_trackbars(prefix):
    h0 = cv2.getTrackbarPos(f"H{prefix}_MIN", WINDOW)
    h1 = cv2.getTrackbarPos(f"H{prefix}_MAX", WINDOW)
    s0 = cv2.getTrackbarPos(f"S{prefix}_MIN", WINDOW)
    s1 = cv2.getTrackbarPos(f"S{prefix}_MAX", WINDOW)
    v0 = cv2.getTrackbarPos(f"V{prefix}_MIN", WINDOW)
    v1 = cv2.getTrackbarPos(f"V{prefix}_MAX", WINDOW)
    low_h, high_h = sorted((h0, h1))
    low_s, high_s = sorted((s0, s1))
    low_v, high_v = sorted((v0, v1))
    return (low_h, low_s, low_v, high_h, high_s, high_v)


def _working_config(base, color):
    min_roi_size = base.roi.edge_margin * 2 + 1
    x = min(cv2.getTrackbarPos("ROI_X", WINDOW), base.camera.width - min_roi_size)
    y = min(cv2.getTrackbarPos("ROI_Y", WINDOW), base.camera.height - min_roi_size)
    width = max(min_roi_size, cv2.getTrackbarPos("ROI_W", WINDOW))
    height = max(min_roi_size, cv2.getTrackbarPos("ROI_H", WINDOW))
    width = min(width, base.camera.width - x)
    height = min(height, base.camera.height - y)
    roi = RoiConfig(x, y, width, height, base.roi.edge_margin)

    colors = {name: list(ranges) for name, ranges in base.colors.items()}
    if color == "red":
        colors[color] = [_range_from_trackbars("1"), _range_from_trackbars("2")]
    else:
        colors[color] = [_range_from_trackbars("1")]
    return with_calibration(base, roi=roi, colors=colors)


def _draw_result(frame, config, color, result, frame_id):
    display = frame.copy()
    roi = config.roi
    cv2.rectangle(
        display,
        (roi.x, roi.y),
        (roi.x + roi.width, roi.y + roi.height),
        (0, 255, 255),
        2,
    )
    if result.bbox is not None:
        x, y, width, height = result.bbox
        box_color = (0, 255, 0) if result.valid else (0, 0, 255)
        cv2.rectangle(display, (x, y), (x + width, y + height), box_color, 2)

    lines = [
        f"color={color} frame={frame_id} valid={result.valid} reason={result.reason}",
        f"bbox={result.bbox} area={result.contour_area:.1f} pixels={result.mask_pixels}",
        f"aspect={result.aspect_ratio:.3f} fill={result.fill_ratio:.3f}",
        "keys: r=red b=blue s=save q/ESC=quit",
    ]
    for index, text in enumerate(lines):
        cv2.putText(
            display,
            text,
            (10, 25 + index * 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return display


def main(argv=None):
    parser = argparse.ArgumentParser(description="RDK USB-camera ball calibration")
    parser.add_argument("--config", default="rdk_vision/config.yaml")
    parser.add_argument("--color", choices=("red", "blue"), default="red")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
    logger = logging.getLogger("rdk_vision.calibrate")
    config = load_config(args.config)
    active_color = args.color
    camera = LatestFrameCamera(config.camera)

    try:
        camera.start()
        if not camera.wait_until_ready(config.camera.startup_timeout_ms):
            raise RuntimeError("camera produced no frame before calibration timeout")
        _create_trackbars(config, active_color)

        while True:
            snapshot = camera.get_latest()
            if snapshot is None:
                key = cv2.waitKey(10) & 0xFF
                if key in (27, ord("q")):
                    break
                continue

            working = _working_config(config, active_color)
            detector = BallDetector(working)
            result = detector.detect(snapshot.frame, active_color)
            mask = detector.build_mask(snapshot.frame, active_color)
            display = _draw_result(snapshot.frame, working, active_color, result, snapshot.frame_id)
            cv2.imshow(WINDOW, display)
            cv2.imshow(MASK_WINDOW, mask)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("r") and active_color != "red":
                active_color = "red"
                _set_color_trackbars(config, active_color)
                continue
            if key == ord("b") and active_color != "blue":
                active_color = "blue"
                _set_color_trackbars(config, active_color)
                continue
            if key == ord("s"):
                saved = working
                if result.valid and result.bbox is not None:
                    _, _, width, height = result.bbox
                    saved = with_calibration(
                        saved,
                        reference_width=width,
                        reference_height=height,
                    )
                    logger.info("saved reference ball size %dx%d", width, height)
                else:
                    logger.info("no valid ball; keeping reference size unchanged")
                save_config(saved, args.config)
                config = saved
                logger.info("saved calibration to %s", args.config)
    finally:
        camera.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
