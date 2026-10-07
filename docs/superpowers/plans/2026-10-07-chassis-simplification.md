# Chassis Simplification Implementation Plan

**Goal:** Preserve route behavior while making heading, line acquisition and motor diagnostics independently testable.
**Architecture:** Route policy is passed explicitly to the chassis; one selected heading mode owns yaw output. Line acquisition is a separate nonblocking module. Motor feedback codec is isolated from hardware binding.
**Tech Stack:** C11, STM32 HAL, CMake, host assertions, Release LTO.

## Constraints
- Preserve red/blue route distances, targets (180/183.25 and 0/5.3 degrees), tolerances, vision parallelism and arc boundaries.
- Keep 49-channel telemetry compatible; additional diagnostics are debugger-visible.
- Do not install reference HWT101 logic on the current JY60.
- User confirmed one shared motor RX/TX UART; use addressed sequential polling and preserve command odometry until feedback is validated.
- Execute in this working directory as requested; no flashing or driving in this request.

## Tasks
- [x] Add regression tests for mode selection, explicit route policy, safe STOP, wheel quantization and feedback codec resynchronization.
- [x] Centralize heading/line parameters; select a single heading controller mode per update. Pass route policy from path_ports rather than reading telemetry in the controller.
- [x] Extract PathHeading_Ready and line acquisition to path_line.c. Replace recovery numbers with names, remove unused line fields, preserve first-only gate and red/blue behavior.
- [x] Centralize motion pending registration and RPM conversion. Retain externally exposed manual diagnostic APIs.
- [x] Add validated 0x36 position request/stream decoding from the reference protocol with freshness tracking; bind the confirmed shared USART3 with sequential addressed queries.
- [x] Run host suite, vision and chassis-only LTO builds, inspect diffs and document limitations.

## Verification
Run `cmake --build build/host-path -j4` then `ctest --test-dir build/host-path --output-on-failure`.
Build `build/Release-Size-Audit` and `build/Red-Chassis-Test`. Check STOP/fault overrides, precision tolerance boundaries, finite input rejection, stale/malformed/unknown motor replies, and continuous handoffs.
