#!/usr/bin/env python3
"""RDK X5 clockwise-disc test: G101 once, then early G102 for 5 red balls.

Validated hardware flow retained from the existing test program:

1. RDK opens the Hiwonder UART and executes G101 once.
2. USB camera startup/AWB warm-up runs in parallel with G101.
3. Recognition begins only after both G101 and camera/AWB startup are complete.
4. The disc rotates clockwise; in the current camera view the balls move from
   RIGHT to LEFT.
5. Do not wait until the whole ball is inside the old trigger ROI.  When a red
   candidate enters from the ROI's RIGHT edge and reaches an adjustable advance
   line, immediately transmit G102 on that same frame.
6. G102 is counted only after the Hiwonder 0x08 completion frame is received.
7. Standalone mode immediately rearms after each G102 completion. Bridge mode
   keeps vision active but suppresses G102 until the matching RFID confirmation.
8. Bridge mode stops only after five completed G102 actions and RFID #5.

The existing rdk_vision HSV, ROI and reference-size calibration are reused.  No
changes are made to the detector/config files.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = Path(__file__).resolve().parent
for path in (ROOT, TOOLS_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from disc_rtl_trigger import RightToLeftDiscTrigger, derive_trigger_x
from disc_rfid_gate import DiscRfidGate
from hiwonder_action import HiwonderActionBoard
from rdk_vision.ball_detector import BallDetector
from rdk_vision.camera import LatestFrameCamera
from rdk_vision.config import load_config


DEFAULT_MAX_ACTIONS = 5
DEFAULT_TRIGGER_X = 380


def run_preparation_with_camera_warmup(
    *,
    board,
    camera,
    prep_group: int,
    repeat_count: int,
    servo_timeout_s: float,
    camera_ready_timeout_ms: int,
):
    """Run camera/AWB startup concurrently with the G101 preparation action.

    Camera startup currently performs the AWB warm-up synchronously before its
    capture thread begins publishing frames.  Starting it in a helper thread lets
    those warm-up frames be consumed while the servo board is executing G101.
    Recognition is not allowed to continue until both operations have completed
    and a post-lock camera frame is available.
    """

    camera_error = []

    def start_camera():
        try:
            camera.start()
        except BaseException as exc:  # propagate startup failure to main task
            camera_error.append(exc)

    camera_thread = threading.Thread(
        target=start_camera,
        name="disc-camera-startup",
        daemon=True,
    )
    camera_thread.start()

    prep = board.run_group(
        prep_group,
        repeat_count=repeat_count,
        timeout_s=servo_timeout_s,
    )

    # Most of this wait is normally hidden by G101.  The timeout only covers
    # the remaining camera startup time after G101 has completed.
    camera_thread.join(timeout=max(0.1, camera_ready_timeout_ms / 1000.0))
    if camera_thread.is_alive():
        raise RuntimeError(
            "camera/AWB startup did not finish before startup timeout"
        )
    if camera_error:
        raise camera_error[0]

    if not camera.wait_until_ready(camera_ready_timeout_ms):
        raise RuntimeError("camera produced no frame before startup timeout")

    return prep


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "RDK X5 clockwise-disc test: run G101 once, then trigger G102 "
            "early as red balls enter the ROI from right to left"
        )
    )
    parser.add_argument("--config", default="rdk_vision/config.yaml")
    parser.add_argument("--color", choices=["red", "blue"], default="red")
    parser.add_argument("--servo-port", default="/dev/ttyS1")
    parser.add_argument("--servo-baud", type=int, default=9600)
    parser.add_argument("--prep-group", type=int, default=101, choices=range(0, 256))
    parser.add_argument(
        "--trigger-group", type=int, default=102, choices=range(0, 256)
    )
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--servo-timeout", type=float, default=30.0)
    parser.add_argument(
        "--clear-frames",
        type=int,
        default=2,
        help="deprecated compatibility option; immediate rearm is always used",
    )
    parser.add_argument(
        "--max-actions",
        type=int,
        default=DEFAULT_MAX_ACTIONS,
        help="completed G102 actions before normal exit (default: 5; 0=infinite)",
    )
    parser.add_argument(
        "--trigger-x",
        type=int,
        default=DEFAULT_TRIGGER_X,
        help=(
            "early trigger line using candidate bbox LEFT edge. For right-to-left "
            "motion, a larger X triggers earlier. If omitted it is derived from "
            "the calibrated reference width."
        ),
    )
    parser.add_argument(
        "--trigger-visible-scale",
        type=float,
        default=0.70,
        help=(
            "when --trigger-x is omitted, fraction of calibrated ball width that "
            "must be visible inside the ROI before G102 (smaller=earlier, default=0.70)"
        ),
    )
    parser.add_argument(
        "--edge-tolerance-px",
        type=int,
        default=5,
        help="tolerance for deciding that the partial candidate touches ROI right edge",
    )
    parser.add_argument(
        "--early-min-width-scale",
        type=float,
        default=0.60,
        help="minimum visible width / reference_width for early trigger",
    )
    parser.add_argument(
        "--early-min-height-scale",
        type=float,
        default=0.65,
        help="minimum visible height / reference_height for early trigger",
    )
    return parser


def run_disc_task(
    args,
    *,
    rfid_gate: DiscRfidGate | None = None,
    on_action_complete=None,
    config=None,
    detector=None,
    trigger=None,
    camera=None,
    board=None,
    clock=time.monotonic,
    sleep_fn=time.sleep,
    max_idle_iterations=None,
) -> int:
    """Run the validated disc loop, optionally gating actions on RFID.

    The RFID gate is queried only for the current eligible, non-stale frame.
    It never pauses camera capture, detection, or trigger calculation.
    """
    config = config or load_config(args.config)
    detector = detector or BallDetector(config)

    roi_right = config.roi.x + config.roi.width
    trigger_x = args.trigger_x
    if trigger_x is None:
        trigger_x = derive_trigger_x(
            roi_right,
            config.ball.reference_width,
            args.trigger_visible_scale,
        )

    if not config.roi.x < trigger_x < roi_right:
        raise ValueError(
            f"trigger_x must be inside ROI: {config.roi.x} < X < {roi_right}; "
            f"got {trigger_x}"
        )

    trigger = trigger or RightToLeftDiscTrigger(
        roi_right=roi_right,
        trigger_x=trigger_x,
        reference_width=config.ball.reference_width,
        reference_height=config.ball.reference_height,
        clear_frames=args.clear_frames,
        edge_tolerance_px=args.edge_tolerance_px,
        min_visible_width_scale=args.early_min_width_scale,
        min_height_scale=args.early_min_height_scale,
        # Early partial balls are horizontally clipped, so allow a somewhat
        # wider aspect range than the normal full-ball validator.
        aspect_min=0.55,
        aspect_max=max(1.35, config.ball.aspect_ratio_max),
        fill_min=max(0.45, config.ball.fill_ratio_min - 0.05),
        fill_max=min(0.95, config.ball.fill_ratio_max + 0.05),
    )

    camera = camera or LatestFrameCamera(config.camera)
    board = board or HiwonderActionBoard(args.servo_port, args.servo_baud)

    action_count = 0
    try:
        board.open()
        print(
            f"Servo ready: {args.servo_port} @ {args.servo_baud}; "
            f"prep=G{args.prep_group}, trigger=G{args.trigger_group}"
        )
        print(
            f"Starting camera/AWB warm-up in parallel with preparation "
            f"action G{args.prep_group}..."
        )
        prep = run_preparation_with_camera_warmup(
            board=board,
            camera=camera,
            prep_group=args.prep_group,
            repeat_count=args.repeat,
            servo_timeout_s=args.servo_timeout,
            camera_ready_timeout_ms=config.camera.startup_timeout_ms,
        )
        print(
            f"PREP + CAMERA READY G{args.prep_group}; "
            f"rx={prep.raw.hex(' ')}"
        )

        snapshot = camera.get_latest()
        if snapshot is None:
            raise RuntimeError("camera ready but no latest frame is available")

        last_frame_id = snapshot.frame_id
        last_reason = None
        last_suppressed_event = None
        last_suppressed_at = 0.0
        idle_iterations = 0

        print(
            f"Camera ready: frame={snapshot.frame_id}, shape={snapshot.frame.shape}, "
            f"color={args.color}"
        )
        print(
            "CLOCKWISE DISC / CAMERA MOTION: RIGHT -> LEFT\n"
            f"ROI x=[{config.roi.x}, {roi_right}), reference="
            f"{config.ball.reference_width}x{config.ball.reference_height}\n"
            f"EARLY TRIGGER: bbox.left <= {trigger_x} while bbox.right touches "
            f"ROI right edge (tolerance={args.edge_tolerance_px}px).\n"
            "For right-to-left motion: increase --trigger-x to trigger EARLIER; "
            "decrease it to trigger LATER."
        )
        print(
            f"Task limit: {args.max_actions if args.max_actions else 'infinite'} "
            f"completed G{args.trigger_group} actions; immediate_rearm=True."
        )

        while True:
            if rfid_gate is not None:
                if rfid_gate.is_complete():
                    print(
                        f"DISC TASK COMPLETE: {action_count}/{args.max_actions} "
                        f"completed G{args.trigger_group} actions and RFID confirmed"
                    )
                    return 0
                if rfid_gate.is_cancelled():
                    return 1
            snapshot = camera.get_latest()
            if snapshot is None or snapshot.frame_id <= last_frame_id:
                idle_iterations += 1
                if max_idle_iterations is not None and idle_iterations >= max_idle_iterations:
                    return 1
                sleep_fn(0.001)
                continue
            idle_iterations = 0
            last_frame_id = snapshot.frame_id

            age_ms = (clock() - snapshot.timestamp) * 1000.0
            if age_ms > config.camera.stale_ms:
                if last_reason != "stale":
                    print(f"frame={snapshot.frame_id} STALE age_ms={age_ms:.1f}")
                    last_reason = "stale"
                # Stale data must never trigger or prove that a target left.
                continue

            result = detector.detect(snapshot.frame, args.color)
            event = trigger.update(result)

            bbox_left = None
            bbox_right = None
            bbox_cx = None
            if result.bbox is not None:
                bx, _by, bw, _bh = result.bbox
                bbox_left = bx
                bbox_right = bx + bw
                bbox_cx = bx + bw / 2.0

            if result.reason != last_reason or result.valid or event is not None:
                print(
                    f"frame={snapshot.frame_id} state={trigger.state} "
                    f"valid={result.valid} reason={result.reason} bbox={result.bbox} "
                    f"left={bbox_left} cx={bbox_cx} right={bbox_right} "
                    f"trigger_x={trigger_x} aspect={result.aspect_ratio:.3f} "
                    f"fill={result.fill_ratio:.3f} age_ms={age_ms:.1f}"
                )
                last_reason = result.reason

            if event not in ("TRIGGER_EARLY", "TRIGGER_EDGE", "TRIGGER_VALID"):
                continue

            if rfid_gate is not None and not rfid_gate.can_execute_action(snapshot.timestamp):
                waiting = rfid_gate.waiting_rfid_index
                now_s = clock()
                if event != last_suppressed_event or now_s - last_suppressed_at >= 0.5:
                    print(
                        f"frame={snapshot.frame_id} event={event} "
                        f"ACTION SUPPRESSED: WAITING RFID #{waiting}"
                    )
                    last_suppressed_event = event
                    last_suppressed_at = now_s
                continue

            next_action = action_count + 1
            detected_ns = time.monotonic_ns()
            print(
                f"{event} #{next_action} frame={snapshot.frame_id}; "
                f"sending G{args.trigger_group} immediately"
            )

            # Measure the actual detection -> UART write/flush interval.  Do not
            # hide it inside run_group(), because that also waits for mechanical
            # completion and would mix software latency with servo motion time.
            tx = board.start_group(args.trigger_group, repeat_count=args.repeat)
            tx_done_ns = time.monotonic_ns()
            detect_to_tx_ms = (tx_done_ns - detected_ns) / 1_000_000.0
            print(
                f"TX frame: {tx.hex(' ')}; "
                f"DETECT_TO_TX_MS={detect_to_tx_ms:.3f}"
            )

            completed = board.wait_group_complete(
                args.trigger_group,
                timeout_s=args.servo_timeout,
            )
            finished_ns = time.monotonic_ns()
            action_count = next_action
            print(
                f"ACTION COMPLETE #{action_count} G{args.trigger_group}; "
                f"detect_to_complete_ms={(finished_ns - detected_ns) / 1_000_000.0:.1f}; "
                f"rx={completed.raw.hex(' ')}"
            )

            if rfid_gate is not None:
                if not rfid_gate.on_action_complete(action_count):
                    raise RuntimeError(
                        f"disc RFID gate rejected action completion {action_count}"
                    )
            if on_action_complete is not None:
                on_action_complete(action_count)
            if rfid_gate is not None:
                print(f"ACTION GATE CLOSED: WAITING RFID #{action_count}")

            # Ignore frames captured while G102 was executing.  Immediate rearm
            # begins from the first fresh frame after action completion.
            latest_after_action = camera.get_latest()
            if latest_after_action is not None:
                last_frame_id = latest_after_action.frame_id
            last_reason = None

            if rfid_gate is None and args.max_actions and action_count >= args.max_actions:
                print(
                    f"DISC TASK COMPLETE: {action_count}/{args.max_actions} "
                    f"completed G{args.trigger_group} actions"
                )
                return 0

            print("REARMED IMMEDIATELY: continuing recognition on next fresh frame")

    except KeyboardInterrupt:
        print("\nABORTED by user; no further servo commands will be sent")
        return 130
    finally:
        camera.stop()
        board.close()


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.clear_frames < 1:
        parser.error("--clear-frames must be >= 1")
    if args.max_actions < 0:
        parser.error("--max-actions must be >= 0")
    if not 0.0 < args.trigger_visible_scale <= 1.0:
        parser.error("--trigger-visible-scale must be in (0, 1]")
    if not 0.0 < args.early_min_width_scale <= 1.0:
        parser.error("--early-min-width-scale must be in (0, 1]")
    if not 0.0 < args.early_min_height_scale <= 1.0:
        parser.error("--early-min-height-scale must be in (0, 1]")

    return run_disc_task(args)


if __name__ == "__main__":
    raise SystemExit(main())
