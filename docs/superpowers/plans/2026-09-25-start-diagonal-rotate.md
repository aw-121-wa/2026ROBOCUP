# Continuous start diagonal rotation implementation plan

Goal: Keep the current displacement and white-line alignment, but complete +180 degrees during the diagonal and carry 60 RPM through the arc handoff. Vision remains bypassed. Departure position is adjusted by the user; do not retune route distances.

Architecture: Add a boundary-speed variant of Chassis_MoveRotate; preserve the existing stopping variant for warehouse travel. Use a quintic distance-indexed yaw profile with zero endpoint angular feedforward. Continuous completion releases motion busy without Hold, carrying the final yaw reference and coordinate frame into the arc and straight. Preserve STOP/fault handling. Report the active blend target in telemetry.

- [x] Regress route command boundary speeds, motion_done gating and no Hold at either handoff; verify old implementation fails.
- [x] Test smooth yaw endpoints, derivative and signed integral, and heading-frame tangent geometry.
- [x] Implement motion helper, controller boundary API and chain reference; wire ports and mission.
- [x] Run host suite, build Release and symbol-bearing Debug, inspect final diff.

Hardware smoothness remains subject to real vehicle tracking; do not claim physical validation from host results.
