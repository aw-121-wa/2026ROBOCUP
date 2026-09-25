# Chassis stability implementation plan

Goal: retain PATH heading intent across stops and smooth unplanned body velocity transitions.
Architecture: existing chassis task remains the sole motor owner. PATH initializes a heading reference once; planned turns update it, Holds preserve it. Sensor detection does not measure alignment. Normal body/hold commands use a common velocity interpolation; planned translation retains its existing distance braking envelope. STOP and faults bypass interpolation.

Approved scope: first two improvements from the code comparison. Encoder feedback and mode-specific tuning need later hardware measurements. No route distance/direction changes or automatic vehicle run.

- [x] Add actual-controller host regressions for residual angle, orbit pause, white-line reference, body slew and emergency stop; observe failure.
- [x] Initialize PATH heading on accepted start, retain it across holds/moves, update commanded turns, keep manual behavior.
- [x] Add coordinated body velocity slew and angular output slew; normal brake remains busy until zero. Fault paths retain immediate zero.
- [x] Run host suite, Debug and Release builds; document default limits and remaining hardware validation.

Normal body acceleration 550 mm/s^2 and angular acceleration 3 rad/s^2; normal stopping 800 mm/s^2 and 6 rad/s^2. These are initial engineering limits, not measured tuning. Common interpolation preserves a commanded orbit ratio during ramp-up/down. No stationary heading loop.

## Validation

Before implementation the actual-controller host harness reproduced four failures: rotation/MoveRotate targets lost on Hold, orbit pause overwritten heading, and body start velocity step. Existing immediate STOP/IMU-loss checks passed.

After implementation the host suite and Debug/Release builds pass. Tests exercise production chassis_control.c with only hardware/transport/mission dependencies stubbed; they do not simulate traction or motor dynamics. Normal stop is still followed by the existing 80 ms command-settled guard. Fault/cancel/watchdog paths use immediate Hold rather than normal braking. No stationary heading correction was introduced.

Remaining hardware validation: actual start/brake comfort, white-line stopping travel, post-orbit heading recovery, and angular slew tracking during blended turns. New slew parameters may change physical stop positions despite unchanged route distances. rpm_applied remains a transmitted command, not measured wheel speed; encoder feedback is deferred pending protocol checks.
