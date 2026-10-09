# HWT101CT setup

- Same USART2 wiring; 115200 baud, 8 data bits, no parity, 1 stop bit.
- Configure the sensor externally for 200 Hz gyro + angle output and save it.
  Firmware does not automatically rewrite sensor settings.
- Legacy `JY60_*` API is retained. Receiver expects WIT 11-byte 0x52/0x53
  frames with additive checksum; Z data is signed little-endian at bytes 6–7.
  Scale remains 2000/32768 deg/s and 180/32768 degrees, matching the reference
  HWT101 parser. Verify actual CT frames before enabling motors.
- Circular DMA supports batches: plausibility uses a minimum nominal 5 ms
  interval, while freshness uses real processing time. Good <=25 ms, lost >60 ms.
- Moving heading tolerance: 0.2 degrees against the current target (including
  a changing turn target). Yaw error does not reduce translation speed.
  Debugger `chassis_heading_diagnostics.within_tolerance` reports the 0.2/0.1
  degree tracking goal. Feedback remains active inside
  tolerance; this is not a guaranteed bound on physical tracking error.
- Stationary calibration/rotation admission and active stair hold use 0.1 degrees.
  Gray acquisition does not request stationary angle calibration; emergency STOP remains.
- Red uses map targets 180 degrees at stairs and 0 degrees at warehouse; blue uses 0 and 180 degrees respectively.
  No blue-side angular compensation is applied.

## Commissioning

With motors disabled, verify fresh angle/gyro frames near 200 Hz, checksum health,
static bias, and matching angle/gyro signs during manual rotation. Verify wrap
through +/-180 degrees, then test motion. Host tests do not validate sensor wiring,
actual protocol scaling, mounting offsets, or physical accuracy.
