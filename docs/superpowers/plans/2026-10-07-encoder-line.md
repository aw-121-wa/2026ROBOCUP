# Encoder distance and line edge alignment

Authorized: replace command odometry with measured motor positions; use reference
75 mm wheels / 65535 counts per revolution; shared UART sequential polling.
1. Add independent encoder-delta decoder with stale/jump/sign checks and tests.
2. Gate ARM and motion on fresh feedback, use measured travel in planner, retain
   speed commands only as conservative braking prediction. Never silently fall back.
3. Add bounded, nonblocking two-edge line scan and circular midpoint return.
   Use software heading reference, 0.1 degree gate; no sensor EEPROM writes.
4. Host regression, Release builds, document commissioning limits. No autonomous
   flash/run requested in this turn.
