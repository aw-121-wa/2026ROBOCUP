# Warehouse Destack Implementation Plan

> Execute inline using executing-plans; preserve existing workspace changes and do not commit or deploy without a request.

**Goal:** Implement the approved warehouse prescan, 3-to-1-column destacking and ball unloading sequence.
**Architecture:** A separate STM32 sub-state machine hooks column admission and completion; the existing ball inventory remains authoritative. RDK receives token-scoped block checks on the ball camera and distinguishes EMPTY, digit, UNKNOWN and ERROR.
**Tech Stack:** C11/STM32/FreeRTOS/CMake; Python/OpenCV/unittest.

## Constraints
- 112/115/118 pose, 113/116/119 pick, 114/117/120 place; rows run bottom to top.
- Fourth column is 200 mm beyond third; wait for arm completion before all vehicle motion.
- Preserve pure-chassis mode, side headings and return origin at third column.
- 64 KiB physical Flash limit remains unchanged.

## Tasks
- [ ] Add token-scoped BLOCK_CHECK/BLOCK_RESULT protocol and action groups 112-120; test stale/error/empty responses and cancellation.
- [ ] Add App/path_destack.c/h for column position recording, reverse traversal, carry/place/return states and occupancy checks. Hook App/path_warehouse.c and test movements and failure gates.
- [ ] Add RDK block vision configuration and classifier with fresh-frame confirmation and calibrated empty reference; extend bridge worker and tests. Unknown results never mean empty.
- [ ] Build host tests and firmware. Measure size and remove redundant old code or improve compiler size settings if needed without weakening fault handling.
- [ ] Document camera calibration and protocol, report deployable status and remaining field validation.
