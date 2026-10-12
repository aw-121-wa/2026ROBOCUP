# Warehouse Continuous Destack Implementation Plan

**Goal:** Implement the approved continuous warehouse scan, transfer and ball unloading workflow.
**Architecture:** An isolated PathDestack controller owns warehouse travel and row scanning. Existing ball inventory, arm commands and return remain reusable. A token-scoped RDK stream reports fresh digit locations.
**Tech Stack:** STM32 C, CMake/CTest, Python/OpenCV, UART.

## Tasks
- [ ] Restore calibrated block templates from 6bdf72e; implement frame localization with image center offsets and freshness tests.
- [ ] Add WH_SCAN / WH_FRAME / WH_STOP / WH_DONE token protocol; reject stale tokens and block arm commands until scan stop acknowledged.
- [ ] Implement strict four-probe warehouse admission and recoverable lateral search; retain non-warehouse probe semantics.
- [ ] Implement column mapping and serpentine row scan, stopped center verification, G112–120 transfer and resume, then physical column 3→2→1 unloading.
- [ ] Test protocol and state transitions including stale camera, lost line, arm failure, duplicate targets and scan boundaries; run host regression and Python tests.
- [ ] Build Release/LTO, inspect flash fit, document runtime parameters and unverified camera calibration. Do not flash or deploy without request.

## Validation commands
`cmake --build build/host-path`
`ctest --test-dir build/host-path --output-on-failure`
`cmake --build build/Release-Size-Audit`
`python -m unittest discover -s RDK/tests -p test_warehouse_stream.py`
