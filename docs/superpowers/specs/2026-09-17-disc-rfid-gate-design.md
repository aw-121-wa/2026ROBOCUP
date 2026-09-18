# Disc Vision RFID Gate Design

## Scope

Only the rotating-disc task changes. Obstacle, stair, chassis route, action-group numbers, camera calibration, detector thresholds, trigger mathematics, serial devices, baud rates, and the 60-second overall disc timeout remain unchanged.

## RDK architecture

`DiscRfidGate` is a lock-protected pure state object. It starts with action 1 permitted, closes after each confirmed G102 completion, accepts only the matching `DISC_RFID_OK N`, reopens for actions 2 through 5, and becomes complete without reopening after RFID 5.

The validated disc loop becomes `run_disc_task(...)`. It always fetches fresh frames, applies the existing stale check, calls `BallDetector.detect`, and calls `RightToLeftDiscTrigger.update`. An eligible event starts G102 only when the optional gate permits it. A blocked eligible event is logged with rate limiting and is never latched. G102 completion, not transmission, advances the action count and invokes the completion callback. With no gate, the standalone CLI preserves immediate rearm and exits after five completions.

`BridgeCore` starts exactly one daemon worker for a disc transaction. The serial caller returns immediately after `DISC_ACK`, so it can continue handling `PING` and indexed RFID confirmations. The worker calls `run_disc_task`, sends indexed action-complete messages through a callback, and finally emits `DISC_DONE` or `DISC_ERROR`. Serial writes are protected by one lock.

## STM32 architecture

`RdkLink` stage 4 accepts strictly increasing `DISC_ACTION_DONE 1..5`, exposes each accepted index as a one-shot event, and keeps the original transaction active with `PATH_WAIT`. `Rdk_SendDiscRfidOk` queues one auxiliary line without changing `started`, `timeout`, `stage`, `active`, or the main request sequence. Invalid or duplicate action indices follow the existing protocol-failure lock behavior.

`PathPorts` clears the mission RFID list and closes capture when a disc starts. On an action-done event it opens exactly the matching RFID gate, clears ring and partial-parser state, and stores the current distinct-ID count. UART7 receive interrupts remain armed continuously; capture only controls whether received bytes enter the gate ring. Parsing stops as soon as one new distinct UID grows the count, closes capture, clears the waiting index, and queues `DISC_RFID_OK N`. Bytes while closed and a second frame already buffered for the same gate cannot pre-authorize the next action.

`PathMission` accepts `PATH_OK` for the disc phase only when at least `DISC_REQUIRED_RFID_COUNT` distinct IDs exist; otherwise it fails. The required count is centralized as 5 in `disc_task_config.h`.

## Safety and timing

No intermediate message changes the overall RDK or mission deadline. Cancellation, link/protocol failure, and timeout close both gates. RDK never stores an eligible event across RFID wait; after reopening, only a newly processed non-stale frame can start G102. The fifth action remains blocked while vision continues until RFID 5; only then can the worker return success and the bridge send `DISC_DONE`.

## Verification

Tests cover the pure gate, the real vision-loop decision path, asynchronous bridge behavior and serialized writes, indexed RDK link events and auxiliary TX, capture opening/closing and parser consumption, mission RFID completion defense, standalone compatibility, both firmware configurations, and all existing suites.
