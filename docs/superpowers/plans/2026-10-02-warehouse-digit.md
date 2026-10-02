# Warehouse digit recognition implementation plan

> **For agentic workers:** Execute inline using executing-plans; preserve existing route and control parameters.

**Goal:** Stop at recognized warehouse digits and place RFID-matched balls, selecting default 123 for the entire warehouse if the first digit is unavailable.
**Architecture:** A separate bounded digit request on the existing RDK serial link carries a unique token and excluded-column mask. RDK owns an independent USB camera and multi-frame digit detector. STM32 owns stopping, map heading/gray gates, immutable per-column mapping and default-mode latch.
**Tech Stack:** C, CMake host tests, Python unittest, existing OpenCV/NumPy/PyYAML RDK stack.

## Constraints
- Two USB cameras on RDK; one visible digit at a time, no pixel-to-mm correction.
- First-column failure locks default mode; later failures in digit mode stop without guessing.
- Preserve stair/orbit/home parameters, inventory acknowledgement, and groups 109/110/111.
- No flashing or vehicle start in this request.

## Task 1: STM32 query and warehouse state machine
- [x] Add tests/test_warehouse_digit.c: shuffled 312 destinations, first NONE latch, fresh result during motion issues HOLD, no GROUP until settled, later NONE timeout and preserved stock.
- [x] Run host build to observe missing interfaces.
- [x] Append PC_WAREHOUSE_DIGIT and input/state fields; add Rdk_WarehouseBegin(token, excluded mask, deadline) and dedicated reply parsing before the strict ball protocol, ignoring stale digit tokens.
- [x] Add warehouse phases 8 (first request) and 9 (stopping/waiting) before existing phase 4 gray/heading checks; map logical columns in PathWarehouse_Code and cached turn planning.
- [x] Run ctest --test-dir build/host-path --output-on-failure.

## Task 2: RDK digit detector and bridge
- [x] Add unittest cases for 1/2/3, blank/noise rejection, repeated/stale frame rejection, exclusion and cancellation; add bridge token echo/NONE tests.
- [x] Run tests to observe missing module.
- [x] Package raster templates, isolate ROI/score configuration, require three different fresh frames; implement bounded camera task without pixel positioning.
- [x] Add WAREHOUSE_CHECK routing to the existing single-worker bridge and release the independent number camera on every exit.
- [x] Run both RDK test suites and existing bridge tests.

## Task 3: Validation and deployment instructions
- [x] Build Release LTO, inspect flash usage and diff.
- [x] Document stable USB paths, ROI setup, first-column fallback and hardware validation limits.

## Results

70 STM32 host checks, 95 RDK task/bridge tests and 28 base vision tests pass. Release LTO firmware is 57184 bytes Flash and 43960 bytes RAM. Number-camera stable path is intentionally unconfigured; the existing RDK address 192.168.128.10 was not reachable over SSH during this implementation. Deployment and real-camera/vehicle verification are not performed.
