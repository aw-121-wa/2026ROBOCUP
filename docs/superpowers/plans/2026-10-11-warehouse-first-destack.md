# Warehouse First Destack Implementation Plan

Goal: implement the user-approved entry, destack-first and nearest-column unloading sequence.

- Establish the warehouse origin when rear PD1 goes dark; staircase uses the same rear-probe condition.
- Set physical centers to 187/374/561/748 mm from entry origin.
- Observe rows 3/2/1 with groups 112/115/118; after moving one block skip the rest of that row.
- Select the nearest unserved column after destacking. Stream bottom digits during travel; require a known column code before unloading, infer only after two distinct known codes.
- Keep strict 0110 recovery and arm/chassis exclusion. Keep fresh-frame and stop acknowledgement checks.
- Double column transfers from 35 to 70 rpm; increase task-external travel and home cruise 25 percent without increasing acceleration.
- Run host route tests (both sides, empty rows, stale frames, arm interlocks), full CTest and Release/LTO build.
