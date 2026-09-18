> 当前 ZHY 迁移版本请先看 [ZHY_MIGRATION.md](ZHY_MIGRATION.md)；下文旧协议及任务说明为历史记录。

# PATH migration implementation plan

## Approved scope

Work directly in this project. Preserve tuned chassis parameters and motor/IMU drivers.
After ARM, PATH starts the legacy competition route; STOP cancels the route.
STM32 owns chassis, RFID, turntable and line/IR sensors. RDK owns vision and all servo groups.
Do not split or edit the stored G101/G102 action groups. Do not flash or run hardware during development.

## Reference

aw-121-wa/licang-2026, snapshot-2026-08-24-servo-arm,
commit 6c43dc02094d089f557b278fd7a6fe5280b4fd45.
Use executable source rather than stale comments (the final lateral move is 1650 mm).

## Implementation and verification tasks

- [x] Add failing host-command tests for PATH parsing, disarmed rejection, busy rejection and STOP priority.
- [x] Add a bounded, nonblocking mission state machine with the legacy route and substage behavior.
      Keep the chassis task as the sole motion-state owner; expose movement completion and controlled rotation/body-speed operations.
      Completion must include a transmitted stop and settling, not merely command acceptance.
- [x] Port USART6 PC6/PC7 at 115200 for turntable address 5; UART8 PE0/PE1 at 115200 for RFID raw IDs 1..9.
      Keep turntable frame encoding, direction and slot pulses from the old source. Use nonblocking TX and timed settle.
- [x] Add a bounded RDK link with transaction identity, acknowledgement, completion/error, cancellation and deadlines.
      Add the matching deployable Python bridge without copying models, caches or generated artifacts.
- [x] Implement disc total deadline DISC_TASK_TIMEOUT_MS=60000U from stage start, never reset by ACTION_DONE or RFID_OK.
      Count unique IDs; each accepted deposited ball causes exactly one slot advance. Five IDs plus final turn completion completes the stage.
      Timeout prevents new grabs; wait for current servo action to finish before releasing the chassis, and report timeout distinctly.
- [x] Preserve legacy RZ approach/orbit and stair Part3->Part2->Part1 order, plus warehouse unloading.
      Never silently skip an unavailable upper-computer action or substitute a successful no-op.
- [x] Run native behavior tests for full route, duplicate IDs, timeout boundary/wraparound, stale replies, STOP and motion/action interlocks.
- [x] Build STM32 firmware and check size; review the complete changes independently.
- [x] Document deployment of both sides, commands, configuration macros and staged hardware tests.

## Hardware assumptions and limits

The user explicitly requested old-engineering peripheral mappings. Hardware remains untested here.
Legacy turntable completion uses estimated travel plus settle time, not measured position feedback.
The current vision package only provides the disc-specific detector configuration; other stage image framing requires field validation.
Do not overwrite the RDK's calibrated config.yaml during deployment.
