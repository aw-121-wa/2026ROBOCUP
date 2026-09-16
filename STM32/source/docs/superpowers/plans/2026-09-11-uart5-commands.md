# UART5 commands implementation plan

User approved the text protocol; implement directly in the existing workspace without commits.

Goal: ARM, FORWARD signed_mm, SHIFT signed_mm, STOP via UART5.
Architecture: RX interrupt only buffers bytes. Chassis task drains a bounded buffer,
parses lines and executes commands after its safety checks. No second task writes control state.
Tech: C11, STM32 HAL interrupt RX, existing 200 Hz control and JustFloat TX.

1. Add App/host_command.h/.c with incremental line parsing and pure state policy.
   Add tests/test_host_command.c and CMake target; verify failure before implementation.
   Accept uppercase commands, LF/CRLF/CR, signed decimal 1..5000 mm.
   Reject garbage, extra arguments, nonfinite values, overlong lines; ignore empty lines.
2. Add App/host_uart.h/.c: 256-byte ISR/task ring, 1-byte RX IT.
   Overflow/UART errors discard incomplete input, stop/disarm and recover RX.
   No text TX; do not interrupt telemetry TX during RX recovery.
3. Integrate in chassis_control.c and root CMake.
   Execute only in chassis task. ARM checks existing guards, repeated ARM is idempotent.
   Reject distance/ARM while planner or jog busy. STOP clears queued input.
   Default speed 50 mm/s, acceleration/deceleration 100 mm/s².
   Append armed/fault/result/sequence/busy/bias_ready to existing 14 telemetry channels.
4. Update docs/TESTING.md: wiring PD2, newline, commands, feedback, low-speed test,
   no disconnect watchdog or physical position feedback. Run all host tests, Debug build,
   git diff --check. Do not flash or initiate motion.

Verification cases: all four commands, signed distances, CRLF, blank lines,
split input, invalid tokens, trailing junk, range edges, line overflow and recovery,
disarmed/busy policy, STOP always accepted. Hardware acceptance remains required.

## Verification record

- Parser test failed against the initial unimplemented interface, then passed.
- Eight host CTest targets pass, including UART buffer wrap, overflow, flush,
  receive rearm failure and recovery, and interrupt-mask preservation.
- Debug firmware builds: FLASH 55,608 bytes; RAM 42,152 bytes.
- HAL RX-only abort was checked to preserve TX state and clear RX error/data flags.
- No flash, motor enable or movement was performed. UART electrical timing,
  task stack headroom and physical travel still require board-level validation.
