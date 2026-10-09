# Blue route coordinate contract

Both start poses use the red start heading as map 0 degrees. Blue reflects map Y, leaving map X unchanged. Body-frame X/Y must not be mirrored blindly after a heading change.

| Stage | Blue command / target |
| --- | --- |
| Opening diagonal | Map (+1558.8922, -567.3904) mm; rotate +180 degrees while moving |
| Following straight | Blue length 2223.9384 mm (red 2028.9384 mm + 195 mm), negative body X at heading 180 |
| Disc to pillar | Body (+1395, -715) mm from heading 180; rotate -90 to 90 |
| Pillar | Keep negative rotation direction and existing orbit speeds; accumulate 530 degrees, including pauses |
| Stair entry / work | Map heading 0; positive body X for progress, mirrored lateral search |
| Warehouse transfer / work | Turn -180 to map heading 180; positive body X for column scanning |
| Home | Keep the unloading heading; body +X 2200 mm, then body right (-Y) 1000 mm; then align map 180 degrees within 0.1 degree |

The opening and warehouse arcs are transformed in the final body heading frame. Gray admission stays 0110 and bidirectional lateral search remains enabled. Red route behavior is unchanged by this mirror conversion. Both sides share the red baseline speed, acceleration/deceleration and return trims. Blue opening turn speed and its exit speed are reduced by 10%; the following arc starts at that reduced boundary speed and returns to the shared exit speed. Blue opening straight adds 195 mm. Blue opening dynamic-turn KP is 8.0; feedforward and damping retain their existing settings. Blue stair triggers are 280 and 675 mm with a 900 mm endpoint (red 160 and 630 mm, endpoint 860 mm); the former blue 500 mm intermediate stop is removed. Side-specific exceptions are the mirrored geometry, opening half-turn, 530-degree orbit and work-area headings.

## Orbit exit heading

90 - 530 = -440 degrees, equivalent to 280 degrees. Reaching stair heading 0 requests approximately +80 degrees after the orbit. Blue first moves 20 mm toward its current body right, holding its current map heading. It then travels 250 mm along map +Y toward the stair, retaining heading before the exit turn. The following exit translation remains 10 mm, followed by a 50 mm radius arc. The controller carries residual yaw error into the following movement; verify physical heading and clearance on the chassis.

## Verification

Run `cmake --build build/host-path -j4` and `ctest --test-dir build/host-path --output-on-failure`. Coverage includes opening heading commands, cumulative orbit threshold, stair and warehouse targets, signed digit-advance retargeting, and mirrored return direction. Firmware was compiled using `cmake --build build/Release-Size-Audit -j4`; compilation does not flash or start the robot.

Dynamic yaw feedforward now uses applied wheel-command path speed and is limited together with feedback. The blue 10 mm orbit exit ends at 15 rpm; its 50 mm arc accelerates from 15 to 30 rpm. Hardware RDK UART failures retain the stop response and expose HAL error/count in PathDiagnostics RAM; no action is resumed based on a missing/corrupted reply.

Blue warehouse entry completes the half-turn translation at 40 rpm, then directly begins 40 rpm line acquisition. Both entry arcs and the 110 mm straight are skipped.

After blue stair line admission, back up at 20 rpm along body -X while rear PD1 is on the line. Brake on rear-off, then reset the stair origin after settling. An already-off rear probe skips reverse motion; a 10-second guard prevents unbounded travel. Red retains its 5 mm entry advance.

Blue warehouse first admission directly moves 10 mm toward body right, without the map retreat or 25 mm map offset. Confirmed digits proceed to unloading without lateral reacquisition.

Blue initial stair arrival is latched through braking and skips the initial 0110 recheck so rear-off retreat can execute. Before the first orbit starts, align map heading to 90 degrees within 0.2 degree.

Only PD0 (front, bit 4) and PD1 (rear, bit 2) are read; PD3/PB13 are disconnected and ignored.

Blue orbit radius is reduced by 5 percent using -76.89408 translation rpm with angular command -58.653 unchanged.
