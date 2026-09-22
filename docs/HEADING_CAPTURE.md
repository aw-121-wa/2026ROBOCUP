# Heading diagnostic capture

Default full telemetry now has 49 floats (200 bytes including tail). CH0--CH37 are unchanged; CH20 remains checksum-valid raw JY60 yaw. New read-only channels:

| Channel | Meaning |
|---|---|
| CH38 | MCU milliseconds modulo 16777216 |
| CH39 | Telemetry sequence modulo 16777216 |
| CH40 | Latest health-accepted JY60 yaw, degrees |
| CH41 | Accepted angle frame counter modulo 16777216 |
| CH42 | Mode: 0 idle/output held, 1 planned translation/arc, 2 relative rotation, 3 measured-JY60 zero correction, 4 body velocity, 5 jog |
| CH43 | Heading target, degrees |
| CH44 | Feedback in matching target coordinates, degrees |
| CH45 | Target-minus-feedback error, degrees; mode 2 preserves continuous turns, other modes wrap to +/-180 |
| CH46 | Stair point 1--8; 0 outside stairs |
| CH47 | Time since mission entered timestamp, milliseconds modulo 16777216; timestamp is also refreshed by some command starts |
| CH48 | Age of latest accepted angle frame, milliseconds |

Modes 1/4/5 use the software relative heading frame, mode 2 uses continuous software heading, mode 3 uses sensor yaw directly. During a body command with nonzero angular velocity or a jog, the command overrides the heading controller: CH45 is not the active yaw-rate command; use CH2. CH43--45 are snapshots of the current controller state, not external measurements of physical orientation.

Build: `cmake --build build/Release-ChassisOnly -j 4`. This does not flash the board.

Passive capture after flashing the new firmware:
```
python tools/capture_justfloat.py --port COM3 --channels 49 --duration 120 --out build/captures/path_diagnostic
```
For the older firmware use `--channels 38`. Close VOFA first. Raw .bin, decoded .csv and summary .json are saved. The decoder checks fixed frame spacing; it does not split blindly on the tail bytes. MCU timestamps/counters help detect resets or missing frames. Host timestamps indicate reception time, not exact sampling time.

Only when the chassis is positioned and a route run is intended, append `--run-path`: the script sends ARM, verifies acknowledgement, then PATH. It sends STOP on timeout or error after ARM. Without this option it sends no commands. A completed or failed mission is recorded for another two seconds before closing the port.
