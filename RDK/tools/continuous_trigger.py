#!/usr/bin/env python3
"""Pure trigger/rearm state machine for continuous visual action tests."""
from __future__ import annotations

from collections import deque


class ContinuousTriggerGate:
    """Confirm a target, then require it to leave before rearming.

    ARMED:
      - Apply the configured N-of-M temporal confirmation.
      - On confirmation, emit ``TRIGGER`` exactly once and enter WAIT_CLEAR.

    WAIT_CLEAR:
      - Ignore all valid frames.
      - Require ``clear_frames`` consecutive invalid frames.
      - Then emit ``REARMED`` and return to ARMED.
    """

    ARMED = "ARMED"
    WAIT_CLEAR = "WAIT_CLEAR"

    def __init__(
        self,
        window_frames: int,
        required_hits: int,
        clear_frames: int = 2,
    ) -> None:
        if window_frames < 1:
            raise ValueError("window_frames must be >= 1")
        if not 1 <= required_hits <= window_frames:
            raise ValueError("required_hits must be in 1..window_frames")
        if clear_frames < 1:
            raise ValueError("clear_frames must be >= 1")

        self.window_frames = int(window_frames)
        self.required_hits = int(required_hits)
        self.clear_frames = int(clear_frames)
        self._hits = deque(maxlen=self.window_frames)
        self._clear_count = 0
        self.state = self.ARMED

    def reset(self) -> None:
        self._hits.clear()
        self._clear_count = 0
        self.state = self.ARMED

    def update(self, valid: bool) -> str | None:
        valid = bool(valid)

        if self.state == self.ARMED:
            self._hits.append(valid)
            if sum(self._hits) >= self.required_hits:
                self.state = self.WAIT_CLEAR
                self._hits.clear()
                self._clear_count = 0
                return "TRIGGER"
            return None

        # WAIT_CLEAR: only consecutive invalid frames prove the previous target
        # has left the trigger region. A valid frame restarts the clear count.
        if valid:
            self._clear_count = 0
            return None

        self._clear_count += 1
        if self._clear_count >= self.clear_frames:
            self.state = self.ARMED
            self._clear_count = 0
            self._hits.clear()
            return "REARMED"
        return None
