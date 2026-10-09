# Continue after missing RFID

A completed disc, pillar or stair capture opens a 1000 ms RFID window (`PATH_RFID_WAIT_MS`). A valid unique UID releases the existing handshake immediately. When no new UID arrives, STM32 releases that same handshake after the window, provided the mission is running and the chassis and RDK link are healthy. Arm completion is still mandatory.

Capture count is independent of UID count. An unread capture reserves a pocket with UID/code zero and advances the collection turntable once. At warehouse planning, unknown destinations are assigned unused codes from 11,12,13,21,22,23,31,32,33 in order. With eight distinct known codes the ninth is uniquely determined; multiple missing codes are a best-effort assignment. The inferred slot mask preserves provenance; raw UIDs stay unchanged. Warehouse unloads assigned destinations and returns home. Starting a new mission cannot silently discard occupied pockets.

Camera, arm, link, motor and motion faults retain their existing handling. Missing ID alone no longer ends a task.

Validation: host tests include five unread disc captures and a full route with unread pillar/stair captures, including warehouse and return.

## Departure preparation

G100 is dispatched once after the departure PING response, before the first translation. Travel proceeds in parallel. Disc line entry does not dispatch G100 again; DISC_START still waits until preparation succeeds. Disc-only tests also wait for G100 before recognition.
