# Disc RFID Gate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Gate each next disc G102 action on a new distinct STM32 RFID confirmation while keeping the vision loop active.

**Architecture:** A thread-safe RDK gate separates vision activity from action permission, and a non-blocking bridge worker keeps the STM32 serial loop responsive. STM32 `RdkLink` transports indexed intermediate messages while `PathPorts` owns capture timing and `PathMission` enforces the final count.

**Tech Stack:** Python 3 `unittest`/threading, C11 native CTest, STM32 HAL, CMake/Ninja.

**Spec:** `docs/superpowers/specs/2026-09-17-disc-rfid-gate-design.md`

## Global Constraints

- Modify only the disc vision and RFID gate behavior.
- Preserve every validated vision, action, UART, chassis, trigger, calibration, and timeout parameter.
- Add tests before production code and observe the expected failure for every behavior batch.
- Do not deploy, flash, SCP, or access real RDK hardware.

---

### Task 1: RDK gate and continuous vision loop

**Files:**
- Create: `RDK/tools/disc_rfid_gate.py`
- Modify: `RDK/tools/vision_servo_direct_test.py`
- Create: `RDK/tests/test_disc_rfid_gate.py`
- Create: `RDK/tests/test_disc_task_loop.py`

**Interfaces:**
- Produces: `DiscRfidGate(max_actions=5)`, `can_execute_action()`, `on_action_complete(index)`, `on_rfid_confirmed(index)`, `is_complete()`, `cancel()`.
- Produces: callable `run_disc_task(args, *, rfid_gate=None, on_action_complete=None, dependencies=None)` used by CLI and bridge.

- [ ] Write gate and task-loop tests for initial/open/closed/final states; strict indices; 100 suppressed eligible frames; detector/trigger execution while waiting; fresh/stale behavior; fifth-RFID completion; no sixth action; and standalone immediate rearm.
- [ ] Run the new tests and confirm failure because the gate and callable task interface do not exist.
- [ ] Implement the minimal lock-protected gate and extract the current loop without changing its detector/trigger math or hardware defaults.
- [ ] Run the new and existing RDK suites and confirm green.

### Task 2: Concurrent bridge and serialized TX

**Files:**
- Modify: `RDK/tools/rdk_stm32_bridge.py`
- Modify: `RDK/tests/test_rdk_stm32_bridge.py`

**Interfaces:**
- Consumes: `run_disc_task` and `DiscRfidGate`.
- Produces: immediate `BridgeCore.handle`, one-worker busy guard, indexed RFID routing, thread-safe line sender.

- [ ] Add tests proving DISC_START returns before worker completion, PING and matching RFID are handled while active, duplicate starts do not spawn, mismatched/duplicate indices do not advance, action callbacks emit indexed lines, and concurrent sends remain whole lines.
- [ ] Run bridge tests and confirm failures expose the current synchronous `subprocess.run` behavior.
- [ ] Replace subprocess execution with one in-process worker and lock serial writes.
- [ ] Run the bridge and complete RDK suites and confirm green.

### Task 3: STM32 indexed link protocol

**Files:**
- Modify: `App/rdk_link.h`
- Modify: `App/rdk_link.c`
- Modify: `tests/test_rdk_link.c`

**Interfaces:**
- Produces: `Rdk_TakeDiscActionDone(RdkLink *, uint8_t *)` and `Rdk_SendDiscRfidOk(RdkLink *, uint8_t)`.

- [ ] Add tests for ordered action events, duplicate/out-of-order failure, exact auxiliary wire data, unchanged transaction timing/state, timeout preservation, and normal final DONE.
- [ ] Build/run `test_rdk_link` and confirm failure because intermediate protocol support is absent.
- [ ] Add the minimal one-shot action event and single auxiliary TX slot.
- [ ] Rebuild/run native tests and confirm green.

### Task 4: STM32 capture gate and mission defense

**Files:**
- Modify: `App/disc_task_config.h`
- Modify: `App/path_ports.h`
- Modify: `App/path_ports.c`
- Modify: `App/path_mission.c`
- Modify: `tests/test_path_ports.c`
- Modify: `tests/test_path_mission.c`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: indexed action events and auxiliary RFID-OK TX.
- Produces: closed-at-start capture, one-new-distinct-UID per gate, diagnostics appended after existing fields, and five-ID final completion requirement.

- [ ] Replace the obsolete mission contract with literal cases for RFID counts 0..5 and add capture tests for pre-gate bytes, repeated UIDs, between-gate bytes, five gates, two frames in one ring, and cancel/error/timeout closure.
- [ ] Build/run focused tests and confirm failures match the old eager-capture and zero-ID completion behavior.
- [ ] Implement the count constant, capture lifecycle, immediate parser stop, indexed acknowledgment, failure cleanup, and appended diagnostics.
- [ ] Run all native tests and confirm green.

### Task 5: Documentation and complete verification

**Files:**
- Modify: `docs/ZHY_MIGRATION.md`
- Modify: `docs/PATH_TESTING.md`
- Modify: `RDK/README_DISC_FINAL.md`
- Modify: `RDK/README_STM32_INTEGRATION.md`

- [ ] Replace record-only/independent-completion text with the indexed action/RFID sequence and explicitly state that vision remains active while G102 is suppressed.
- [ ] Run native configure/build/CTest, both Python suites, Debug firmware build, and Release firmware build.
- [ ] Verify the Release `chassis_motor.hex` path and inspect all changed files for out-of-scope obstacle/stair or calibrated-parameter edits.
