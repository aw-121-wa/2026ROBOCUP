# Chassis control and diagnostics

## Ownership

- `path_chassis.c` / `path_warehouse.c`: route sequencing and vision/action parallelism.
- `path_line.c`: nonblocking gray acquisition, reversal/braking, first heading admission. Gray changes lateral position only.
- `path_policy.h`: derives explicit chassis policy from mission state. Only this task-side layer interprets route phases for heading hold and blue compensation.
- `path_ports.c`: command conversion and motion timeout ownership. RPM is converted here; chassis APIs use mm/s and rad/s.
- `chassis_control.c`: IMU update, trajectory execution, mode selection, wheel limiting and the single motor TX owner. Diagnostic snapshots are output only.
- `Algorithm/heading.c`: exactly one selected yaw calculation per control cycle. Motion, dynamic rotation, precision alignment and stationary hold retain their existing gains.

## Tuning

- `Algorithm/heading_tuning.h`: moving gain scales, fine correction limits and rates.
- `App/chassis_tuning.h`: body acceleration/braking, settled delay, home trims.
- `App/stair_heading.h`: red/blue stair targets and heading admission tolerances.
- `App/path_config.h`: warehouse targets, gray search speeds/deadlines and movement defaults.

Both sides use stair heading 180 degrees and warehouse heading 0 degrees, with no blue-side angular offset. Heading feedback retains the 0.2-degree moving and 0.1-degree stationary tracking goals. Gray admission is position-only; home alignment remains independent.

`Chassis_LineSearch`, `Chassis_CalibrateLine` and `Chassis_AlignZero` remain legacy diagnostic APIs; the route does not emit their commands. They are not a second automatic line-calibration algorithm.

## Distance estimation

Distance and braking speed are derived from successfully transmitted wheel commands. There is no encoder position query, feedback parser, feedback DMA owner, freshness gate, or encoder-based odometry in the application. The existing CubeMX UART/DMA configuration remains available but no motor RX capture is started. Gyroscope heading control is still closed loop.

## Release diagnostics

The 49-channel VOFA wire layout is unchanged. The heading snapshot survives LTO and can be watched in a debugger:

- `chassis_heading_diagnostics`: selected mode, requested angular rate, wheel-command-quantized angular rate and measured gyro rate, all rad/s. Mode values are `HeadingMode` in `heading.h`.

The snapshot is a diagnostic task update. Inspect actual gyro response before adding any minimum-speed compensation: quantization and physical deadband are different problems.

## Verification

Host suite: `cmake --build build/host-path -j4`, then `ctest --test-dir build/host-path --output-on-failure`.
Firmware: Release LTO builds in `build/Release-Size-Audit` (vision) and `build/Red-Chassis-Test` (chassis only, runtime RED/BLUE).
Physical heading stability still requires a vehicle run. No automatic run or flash is part of this refactor.

## White-line admission update

Only gray pattern 0110 is accepted at work-area lines. After a gray hit, brake and recheck; invalid patterns use bounded bidirectional lateral search with gyro heading hold. Gray never requests edge-scanning rotation or a stationary heading alignment. The yaw-error-dependent translation slowdown is removed; normal acceleration/braking and wheel limits remain.
