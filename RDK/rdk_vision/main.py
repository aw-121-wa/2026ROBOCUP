import argparse
from collections import deque
import logging
import time

from rdk_vision.ball_detector import BallDetector
from rdk_vision.camera import LatestFrameCamera
from rdk_vision.config import load_config
from rdk_vision.serial_link import SerialLink, SerialLinkError


class RecognitionSession:
    def __init__(self, window_frames: int, required_hits: int):
        self.window_frames = window_frames
        self.required_hits = required_hits
        self.reset()

    def reset(self):
        self.armed = False
        self.color = None
        self.request_frame_id = -1
        self.last_processed_frame_id = -1
        self._hits = deque(maxlen=self.window_frames)

    def arm(self, color: str, request_frame_id: int):
        self.armed = True
        self.color = color
        self.request_frame_id = request_frame_id
        self.last_processed_frame_id = request_frame_id
        self._hits.clear()

    def accept_frame(self, frame_id: int, hit: bool) -> bool:
        if not self.armed:
            return False
        if frame_id <= self.request_frame_id or frame_id <= self.last_processed_frame_id:
            return False
        self.last_processed_frame_id = frame_id
        self._hits.append(bool(hit))
        return sum(self._hits) >= self.required_hits


class VisionService:
    def __init__(
        self,
        camera,
        serial_link,
        detector,
        config,
        *,
        clock=None,
        sleep_fn=None,
        logger=None,
    ):
        self.camera = camera
        self.serial_link = serial_link
        self.detector = detector
        self.config = config
        self.clock = clock or time.monotonic
        self.sleep_fn = sleep_fn or time.sleep
        self.logger = logger or logging.getLogger("rdk_vision")
        self.session = RecognitionSession(
            config.temporal.window_frames,
            config.temporal.required_hits,
        )
        self.request_sequence = 0
        self._camera_fault = None

    def step(self):
        now = self.clock()

        requests = self.serial_link.poll_requests()
        for color in requests:
            current = self.camera.get_latest()
            request_frame_id = current.frame_id if current is not None else -1
            self.request_sequence += 1
            self.session.arm(color, request_frame_id)
            self.logger.info(
                "request seq=%d color=%s request_frame_id=%d",
                self.request_sequence,
                color,
                request_frame_id,
            )

        if not self.session.armed:
            return

        snapshot = self.camera.get_latest()
        if snapshot is None:
            return

        age_ms = (now - snapshot.timestamp) * 1000.0
        if age_ms > self.config.camera.stale_ms:
            self._set_camera_fault("stale", age_ms)
            return
        self._clear_camera_fault_if_needed()

        if snapshot.frame_id <= self.session.last_processed_frame_id:
            return

        result = self.detector.detect(snapshot.frame, self.session.color)
        confirmed = self.session.accept_frame(snapshot.frame_id, result.valid)
        self._log_detection(snapshot, result, age_ms)
        if not confirmed:
            return

        color = self.session.color
        self.serial_link.send_found()
        self.logger.info(
            "FOUND seq=%d color=%s frame=%d software_latency_ms=%.1f",
            self.request_sequence,
            color,
            snapshot.frame_id,
            age_ms,
        )
        self.session.reset()

    def _set_camera_fault(self, fault: str, age_ms: float) -> None:
        if self._camera_fault != fault:
            self.logger.warning("camera fault=%s frame_age_ms=%.1f", fault, age_ms)
        self._camera_fault = fault

    def _clear_camera_fault_if_needed(self) -> None:
        if self._camera_fault is not None:
            self.logger.info("camera recovered from fault=%s", self._camera_fault)
            self._camera_fault = None

    def _log_detection(self, snapshot, result, age_ms: float) -> None:
        self.logger.debug(
            "detect frame=%d color=%s valid=%s reason=%s bbox=%s "
            "area=%.1f pixels=%d aspect=%.3f fill=%.3f age_ms=%.1f",
            snapshot.frame_id,
            self.session.color,
            result.valid,
            result.reason,
            result.bbox,
            result.contour_area,
            result.mask_pixels,
            result.aspect_ratio,
            result.fill_ratio,
            age_ms,
        )

    def _recover_uart(self, exc: SerialLinkError) -> None:
        self.logger.error("UART fault: %s", exc)
        self.session.reset()
        if not self.serial_link.reconnect():
            raise RuntimeError("UART reconnect failed") from exc
        self.logger.info("UART reconnected; recognition state is IDLE")

    def run(self, max_steps=None):
        steps = 0
        while max_steps is None or steps < max_steps:
            try:
                self.step()
            except SerialLinkError as exc:
                self._recover_uart(exc)
            steps += 1
            if max_steps is None or steps < max_steps:
                self.sleep_fn(0.001)


def _configure_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="RDK X5 red/blue ball vision service")
    parser.add_argument(
        "--config",
        default="rdk_vision/config.yaml",
        help="path to RDK vision YAML configuration",
    )
    args = parser.parse_args(argv)

    config = load_config(args.config)
    _configure_logging()
    logger = logging.getLogger("rdk_vision")
    camera = LatestFrameCamera(config.camera)
    serial_link = SerialLink(config.serial)

    try:
        camera.start()
        if not camera.wait_until_ready(config.camera.startup_timeout_ms):
            raise RuntimeError(
                f"camera {config.camera.device} produced no frame within "
                f"{config.camera.startup_timeout_ms} ms"
            )
        serial_link.open()
        detector = BallDetector(config)
        service = VisionService(camera, serial_link, detector, config, logger=logger)
        logger.info(
            "vision service ready camera=%s serial=%s baud=%d",
            config.camera.device,
            config.serial.device,
            config.serial.baudrate,
        )
        service.run()
    except KeyboardInterrupt:
        logger.info("vision service interrupted")
    finally:
        serial_link.close()
        camera.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
