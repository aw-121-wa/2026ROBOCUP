# 当前规则（地图夹线恢复）

阶梯和仓库目标为0110：PD1/PD0在线，PD3/PB13离线。停稳并连续满足100 ms后更新航向。

搜索持续变差100 ms先反向试探，反向尝试耗尽后执行恢复：停稳、对齐起步地图基准最近的0或180度、地图左移30 mm、向地图右侧最多搜索100 mm。检测0110立即制动，停稳重检后校准。地图基准独立于白线校准；恢复最多两次，总30秒超时及普通搜索10度偏角保护保留。恢复对齐阶段允许转至地图轴线。

以下为历史记录，以以上规则和当前代码为准。

# Four-probe line alignment

Physical front-to-rear probes: PD3, PD1, PD0, PB13. Active-low mask: PD3=8, PD1=2, PD0=4, PB13=1. User confirmed all four probes can simultaneously detect the white line.

Stair entry keeps the +25 RPM lateral approach until the inner pair detects the line. After the 55 mm approach and at every subsequent stopped stair point, all four probes must be active (mask 15). Hold first, wait for settled, then require 100 ms continuously at mask 15. A missing probe restarts alignment. No STAIR request is sent before success; a loss immediately before request returns to alignment. An already-running arm transaction is not interrupted by chassis movement.

Alignment searches yaw offsets -6, +6, 0 degrees around the starting measured heading at 4 wheel RPM rotation command. After an unsuccessful sweep it tries alternating low-speed lateral strokes (+8/-8 RPM, 250/500/750/1000 ms, duration capped at 1000 ms), then sweeps again. Each reversal waits for normal braking. Yaw excursion beyond 10 degrees or 30-second total timeout stops/cancels the mission; it never proceeds to recognition on timeout. Lateral displacement is timed, not encoder measured. Values are initial hardware tuning settings.

Search commands bypass the previous yaw target to avoid fighting the physical line correction. Success updates both the route heading and raw-IMU software reference from measured yaw, so following moves hold the aligned body angle. No hardware IMU zero command is sent. Merely detecting a line at disc/warehouse still retains the intended route reference rather than calibrating it.

Disc keeps the inner-pair detection-only behavior. Warehouse step 12 first detects the inner pair, then uses the same four-probe alignment and measured-heading calibration before step 13; calibration failure prevents warehouse entry. Stair checks occur after moves and before recognition/grabbing, not continuously while translating. Four digital detections define the accepted alignment band, not a high-resolution angle measurement.

Host regression and Debug/Release builds pass. This change has not been flashed or physically validated.

Rear input migrated from PD8 to PB13 after static pin checks. PD0/PD1/PD3 unchanged; mask bit 0 remains the rear probe.
