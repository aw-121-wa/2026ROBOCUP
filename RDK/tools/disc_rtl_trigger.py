#!/usr/bin/env python3
"""Right-to-left trigger logic for the rotating-disc red-ball task.

The disc rotates clockwise and, in the current camera view, balls move from
RIGHT to LEFT.  This gate supports three trigger paths:

1. ``TRIGGER_EARLY``: a partial red ball enters from the ROI right edge and its
   visible left edge crosses ``trigger_x``.
2. ``TRIGGER_EDGE``: the detector reports ``edge_margin`` but the candidate is
   already close to the calibrated full-ball size/shape (for example the ball
   touches the bottom ROI edge as observed on hardware).
3. ``TRIGGER_VALID``: the detector's normal full-ball validation succeeds.

There is intentionally NO WAIT_CLEAR/rearm state.  After a G102 action finishes,
the caller immediately resumes feeding fresh frames, so the next eligible red
candidate can trigger at once.  ``clear_frames`` is accepted only for backward
CLI compatibility and is ignored.
"""
from __future__ import annotations


def derive_trigger_x(
    roi_right: int,
    reference_width: int,
    visible_scale: float = 0.70,
) -> int:
    """Return the default left-edge trigger line for a right-to-left ball."""
    if reference_width <= 0:
        raise ValueError("reference_width must be positive")
    if not 0.0 < visible_scale <= 1.0:
        raise ValueError("visible_scale must be in (0, 1]")
    return int(roi_right - round(reference_width * visible_scale))


class RightToLeftDiscTrigger:
    ARMED = "ARMED"

    def __init__(
        self,
        *,
        roi_right: int,
        trigger_x: int,
        reference_width: int,
        reference_height: int,
        clear_frames: int = 2,
        edge_tolerance_px: int = 5,
        min_visible_width_scale: float = 0.60,
        min_height_scale: float = 0.65,
        aspect_min: float = 0.55,
        aspect_max: float = 1.35,
        fill_min: float = 0.45,
        fill_max: float = 0.95,
        safe_edge_min_width_scale: float = 0.85,
        safe_edge_min_height_scale: float = 0.80,
        safe_edge_max_width_scale: float = 1.25,
        safe_edge_max_height_scale: float = 1.25,
    ) -> None:
        if clear_frames < 1:
            raise ValueError("clear_frames must be >= 1")
        if reference_width <= 0 or reference_height <= 0:
            raise ValueError("reference size must be positive")
        if not 0.0 < min_visible_width_scale <= 1.0:
            raise ValueError("min_visible_width_scale must be in (0, 1]")
        if not 0.0 < min_height_scale <= 1.0:
            raise ValueError("min_height_scale must be in (0, 1]")
        if trigger_x >= roi_right:
            raise ValueError(
                "trigger_x must be left of roi_right for right-to-left motion"
            )

        self.roi_right = int(roi_right)
        self.trigger_x = int(trigger_x)
        self.reference_width = int(reference_width)
        self.reference_height = int(reference_height)
        # Retained only so existing code/CLI can still pass the value.
        self.clear_frames = int(clear_frames)
        self.edge_tolerance_px = int(edge_tolerance_px)
        self.min_visible_width_scale = float(min_visible_width_scale)
        self.min_height_scale = float(min_height_scale)
        self.aspect_min = float(aspect_min)
        self.aspect_max = float(aspect_max)
        self.fill_min = float(fill_min)
        self.fill_max = float(fill_max)
        self.safe_edge_min_width_scale = float(safe_edge_min_width_scale)
        self.safe_edge_min_height_scale = float(safe_edge_min_height_scale)
        self.safe_edge_max_width_scale = float(safe_edge_max_width_scale)
        self.safe_edge_max_height_scale = float(safe_edge_max_height_scale)

        self.state = self.ARMED

    def reset(self) -> None:
        self.state = self.ARMED

    def _shape_ok(self, result) -> bool:
        aspect = float(getattr(result, "aspect_ratio", 0.0))
        fill = float(getattr(result, "fill_ratio", 0.0))
        return (
            self.aspect_min <= aspect <= self.aspect_max
            and self.fill_min <= fill <= self.fill_max
        )

    def _is_early_right_edge_candidate(self, result) -> bool:
        bbox = getattr(result, "bbox", None)
        if bbox is None or getattr(result, "reason", None) != "edge_margin":
            return False

        x, _y, width, height = bbox
        right = x + width

        # Right-to-left early trigger: candidate is still clipped by the ROI's
        # right edge and its visible left edge has crossed the advance line.
        if right < self.roi_right - self.edge_tolerance_px:
            return False
        if x > self.trigger_x:
            return False

        if width < self.reference_width * self.min_visible_width_scale:
            return False
        if height < self.reference_height * self.min_height_scale:
            return False

        # Reject obvious large red mechanical structures even if they touch the
        # right edge and happen to have a plausible fill/aspect ratio.
        if width > self.reference_width * self.safe_edge_max_width_scale:
            return False
        if height > self.reference_height * self.safe_edge_max_height_scale:
            return False

        return self._shape_ok(result)

    def _is_safe_edge_candidate(self, result) -> bool:
        """Accept a nearly complete ball that only fails ``edge_margin``.

        This covers the real hardware case where a 152x150 calibrated ball is
        detected as roughly 150x134 because it touches the bottom ROI edge.
        The candidate must already have crossed the right-to-left trigger line
        and remain close to the calibrated ball size/shape.
        """
        bbox = getattr(result, "bbox", None)
        if bbox is None or getattr(result, "reason", None) != "edge_margin":
            return False

        x, _y, width, height = bbox
        if x > self.trigger_x:
            return False

        if width < self.reference_width * self.safe_edge_min_width_scale:
            return False
        if height < self.reference_height * self.safe_edge_min_height_scale:
            return False
        if width > self.reference_width * self.safe_edge_max_width_scale:
            return False
        if height > self.reference_height * self.safe_edge_max_height_scale:
            return False

        return self._shape_ok(result)

    def update(self, result) -> str | None:
        # Always ARMED: after G102 completes the main program immediately
        # resumes on the next fresh frame. No target-clear/rearm delay exists.
        self.state = self.ARMED

        if self._is_early_right_edge_candidate(result):
            return "TRIGGER_EARLY"

        if self._is_safe_edge_candidate(result):
            return "TRIGGER_EDGE"

        if bool(getattr(result, "valid", False)):
            return "TRIGGER_VALID"

        return None
