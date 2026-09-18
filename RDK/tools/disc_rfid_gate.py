#!/usr/bin/env python3
"""Thread-safe action permission state for the disc RFID handshake."""
from __future__ import annotations

import threading
import time


class DiscRfidGate:
    def __init__(self, max_actions: int = 5) -> None:
        if max_actions < 1:
            raise ValueError("max_actions must be positive")
        self.max_actions = int(max_actions)
        self._lock = threading.Lock()
        self.action_allowed = True
        self.action_done_index = 0
        self.rfid_confirmed_index = 0
        self.waiting_rfid_index = 0
        self._complete = False
        self._cancelled = False
        self._action_allowed_since = None

    def snapshot(self):
        with self._lock:
            return (
                self.action_allowed,
                self.action_done_index,
                self.rfid_confirmed_index,
                self.waiting_rfid_index,
                self._complete,
            )

    def can_execute_action(self, frame_timestamp=None) -> bool:
        with self._lock:
            if not self.action_allowed or self._complete or self._cancelled:
                return False
            return (
                frame_timestamp is None
                or self._action_allowed_since is None
                or frame_timestamp >= self._action_allowed_since
            )

    def on_action_complete(self, index: int) -> bool:
        with self._lock:
            if (
                self._cancelled
                or self._complete
                or not self.action_allowed
                or index != self.action_done_index + 1
                or index > self.max_actions
            ):
                return False
            self.action_done_index = index
            self.waiting_rfid_index = index
            self.action_allowed = False
            return True

    def on_rfid_confirmed(self, index: int, *, confirmed_at=None) -> bool:
        with self._lock:
            if (
                self._cancelled
                or self._complete
                or index != self.waiting_rfid_index
                or index != self.rfid_confirmed_index + 1
            ):
                return False
            self.rfid_confirmed_index = index
            self.waiting_rfid_index = 0
            if index == self.max_actions:
                self._complete = True
                self.action_allowed = False
            else:
                self.action_allowed = True
                self._action_allowed_since = (
                    time.monotonic() if confirmed_at is None else float(confirmed_at)
                )
            return True

    def is_complete(self) -> bool:
        with self._lock:
            return self._complete

    def is_cancelled(self) -> bool:
        with self._lock:
            return self._cancelled

    def cancel(self) -> None:
        with self._lock:
            self._cancelled = True
            self.action_allowed = False
