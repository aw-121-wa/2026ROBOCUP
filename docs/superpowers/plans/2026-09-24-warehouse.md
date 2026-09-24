# Warehouse unloading implementation plan

> Superseded RFID policy: user requested restoring original UID acceptance, immediate permission and count-driven storage turns. Block decoding is now passive metadata; unknown destinations stay occupied and prevent false warehouse success. Rollback verified with 53 host tests, Release-Path build and read-only review. Firmware has not been reflashed for this rollback.

> Execute inline with superpowers:executing-plans; implementation stays inline in the existing uncommitted hardware baseline; one read-only review is delegated as required by requesting-code-review.

**Goal:** Preserve collected balls' block-1 destination and 12-slot position, then unload by column after the final gray-line alignment.

**Architecture:** Pure ball inventory module owned by PathMission; UART adapter validates UID+block frames and owns single-slot motor transitions. A warehouse mission module waits for turn completion before GROUP 109/110/111. RDK retains the existing generic group worker.

**Tech Stack:** STM32 C11, host C tests, Python bridge unittest, OpenOCD (deployment only on request).

**Spec:** User-approved conversation: 12 initially empty slots; storage/unloading port identical; 16 identical packed column/row bytes in block 1; skip missing cells; 200 mm retreat before each of 3 columns; groups include take/place/return. Protocol: manual V1.0.5 section 4.4, frame 04 1C 04, UID bytes 7..10, block bytes 11..26.

## Constraints and decisions

- Keep the established chassis trajectory and calibrated vision parameters.
- UID-only frames cannot authorize storage or create warehouse records. Corrupt framing/checksum is discarded; valid frames with illegal ball data or conflicting destinations fault the mission.
- Slot 0 is the initially aligned physical pocket. Clockwise storage step advances the logical port index modulo 12; reverse unloading decrements it. Each completed step updates the index once.
- Release matching RFID permission only after the storage step's estimated completion. Existing motor driver does not provide arrival feedback (840 ms travel/settling, 2 s watchdog).
- Keep inventory across link reset. Abort/error preserves records but marks occupancy/position uncertain; a new task requires a cleared, known-empty system (power reset after manual clearing). A clean completed warehouse may restart.
- No firmware flash or remote copy in this change. Build and document required paired MCU/RDK deployment.

## Steps

- [x] Tests: block decoding, duplicate UID/destination, arbitrary storage order, 12-slot modulo, selection and unload, missing cells, cancel/error/timeouts.
- [x] Add App/ball_inventory.[ch], integrate with mission and CMake.
- [x] Change App/path_ports.c to admit only UID+block for collection, gate one storage step, delay RFID_OK, diagnose slots and faults, preserve reset state.
- [x] Add App/path_warehouse.[ch], enter step 13 after gray alignment, retreat 200 mm for each column, turn before row group, clear slot only on completion.
- [x] Enable groups 109/110/111 in STM32 and RDK; verify ACK/DONE/error and busy serialization.
- [x] Update adapter end-to-end tests and old route endpoints; run all host and RDK tests, compile Release-Path, inspect diff.
- [x] Document inventory assumptions, reader mode, diagnostics, deployment and hardware verification.

## Verification result

53 host tests and 71 RDK tests passed. Release-Path firmware builds successfully. Read-only code review found no blocking issues. Full adapter simulations cover eight balls with a missing cell and all nine balls across disc, pillar and stair. No hardware flashing or RDK upload performed. Motor arrival remains time-estimated and requires physical verification.
