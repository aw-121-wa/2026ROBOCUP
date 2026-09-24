> 2026-09-24：阶梯结束后执行 G3、后退 100、固定左移 1300 同时逆时针转 180°、向右灰度找线，再按 RFID 块 1 编号完成三列仓库放球。以 [阶梯后续流程](STAIR_EXIT.md) 和 [仓库任务](../RDK/README_WAREHOUSE_TASK.md) 为准。旧版阶梯结束即 PATH_DONE 以及旧仓库往返流程不再适用。

# 上电动作与 PATH 绕桩视觉联动（2026-09-19）

> 当前变更优先：绕桩沿原圆周反向运行，前进/旋转分量同时取负，目标从 +352° 改为 -352°。视觉恢复共享圆盘 `config.yaml`，但绕桩只按正常 `valid=True` 检测停车，不用提前触发或独立球心窗口。部署见 [README_PILLAR_NORMAL.md](../RDK/README_PILLAR_NORMAL.md)；下方早期正向/独立 ROI 描述已被此变更替代。

本说明优先于旧版“纯底盘绕桩”“手动 PING 后直接 ARM”说明。主控和 RDK 必须一起更新；幻尔舵控仍使用原二进制动作组协议，无需新增 ASCII PING/PONG 固件。

## 执行顺序

1. 主控上电自动发送 `PING`。未收到 `PONG` 时，2 秒超时后等待 1 秒重试，不执行动作、不允许 ARM/PATH。
2. 收到 `PONG` 后，主控发送 `GROUP 0`；RDK 执行 G0，等待幻尔 `0x08` 动作完成回包，再回 `GROUP_DONE 0`。收到完成后才允许 ARM/PATH（原 IMU、标定、故障检查仍生效）。G0 失败不自动重放，需排障后 `RDK_RESET` 或重新上电。
3. `ARM` → `PATH`：保留原两段位移与 180.5° 旋转 → G100 完成 → 原灰度对齐 → 圆盘任务。
4. 圆盘保留 G101 与相机预热并行、5 次 G102、每次动作完成后等待新的 distinct UID、确认后只使用新鲜图像触发下一次，以及原转盘联动。
5. 圆盘全部完成 → G1 完成 → 原 1750 mm 前进 → 横移，红外触发立即请求停车并做原 30 ms 消抖。
6. 底盘停稳 → `PILLAR_START`：RDK 执行 G103、预热相机；两者完成后回 `PILLAR_READY`，主控才开始原绕桩运动。
7. 绕桩识别到球：RDK 回 `PILLAR_BALL N` → 主控请求停车 → `Chassis_IsSettled()` 成立后回 `PILLAR_STOPPED N` → RDK 执行 G104 → 完成回包后发送 `PILLAR_ACTION_DONE N`。
8. 主控收到动作完成才清理 RFID 缓冲、打开门控；新 distinct UID 到来后发送 `PILLAR_RFID_OK N`。RDK 确认后回 `PILLAR_RESUME N`，主控继续剩余绕行。
9. 累计航向相对绕桩起点达到原 352° → 停稳 → `PILLAR_END` → RDK 释放相机/舵控资源并回 `PILLAR_DONE` → 保留原绕桩后路线。

独立 `DISC` 调试指令仍只做原地圆盘任务，不自动接 G1 或后续路线。已有 `PING` 可用于手动链路检查，不重复执行已完成的上电 G0。

## 程序复用与限制

- 圆盘、绕桩共同调用 `RDK/tools/vision_servo_direct_test.py:run_disc_task`，共享相机、HSV、尺寸/形状过滤。圆盘保留原 ROI 和 `RightToLeftDiscTrigger`；绕桩独立读取 `rdk_vision/pillar.yaml` 的 ROI 与球心触发窗口，准备组 103、抓取组 104，并保留停稳许可回调。详见 [绕桩 ROI 与预览](../RDK/README_PILLAR_ROI.md)。
- 保留现有“新鲜帧立即重新识别”逻辑。绕桩实际视角、球大小、运动方向和停车惯性必须实车验证；截图配置是初值，并非动态抓取精度保证。
- RFID 去重覆盖整个 PATH（圆盘和绕桩共用 64 个 UID 缓冲）。圆盘占 5 个，绕桩最多余下 59 个；容量耗尽按异常处理，不作为成功绕桩条件。
- 绕行运动累计超时保留 15 秒；G104/RFID 等待不占这 15 秒，恢复运动保留原航向起点。准备及每轮动作/RFID 等待最多 60 秒；RDK 绕桩任务总超时 300 秒；结束确认最多 5 秒。单个舵机动作完成回包等待 30 秒。
- `Chassis_IsSettled()` 是现有控制器的停稳判断，未新增实际轮速传感器。所谓“立马停车”是主控收到检测帧后当次控制循环发出 HOLD，实际停止时间包含相机、串口和机械减速延迟。
- STOP/超时/故障关闭新动作许可并尽力向 RDK 发取消。已经发给幻尔的动作仍按原驱动等待结束；没有新增硬件急停协议。

## 新增协议

| 主控 → RDK | RDK → 主控 |
| --- | --- |
| `GROUP 0` / `GROUP 100` / `GROUP 1` | `GROUP_ACK N`，随后 `GROUP_DONE N` 或 `GROUP_ERROR N` |
| `PILLAR_START` | `PILLAR_ACK`，G103/相机准备完成后 `PILLAR_READY` |
| `PILLAR_STOPPED N` | 检测时 `PILLAR_BALL N`，G104 完成后 `PILLAR_ACTION_DONE N` |
| `PILLAR_RFID_OK N` | `PILLAR_RESUME N` |
| `PILLAR_END` | `PILLAR_DONE` |
| `PILLAR_CANCEL` / `DISC_CANCEL` | 活动任务终止时相应 ERROR；主控故障锁定后忽略迟到回包 |

全部 ASCII + CRLF；绕桩 N 从 1 开始递增。任意时刻只有一个 RDK 动作/视觉 worker，避免并发占用同一个舵控串口或相机。原 DISC 协议保持兼容。

## 构建与验证

在工程目录运行：

```powershell
cmake -S tests -B build/host-path -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build/host-path
ctest --test-dir build/host-path --output-on-failure
python -m unittest discover -s RDK/tests
python -m unittest discover -s tests -p 'test_*.py'
cmake --preset Release -B build/Release-Path
cmake --build build/Release-Path
```

在 `RDK` 目录运行 `python -m unittest discover -s rdk_vision/tests`。

固件位于 `build/Release-Path/chassis_motor.hex`，与旧 `build/Release` 分开。本次未烧录、未运行实车动作。

本次桌面验证结果：40 项 C 测试、48 项 RDK 任务测试、28 项视觉组件测试、1 项旧桥接兼容测试全部通过；Release 编译通过，FLASH 使用 47,144 B / 64 KB。覆盖完整 PATH 串口模拟、上电重试/G0 完成门控、停稳许可、RFID 去重、暂停恢复、结束与检测交错、超时/STOP、接收故障时取消及发送口暂忙重试。

RDK 将更新后的整个 `RDK/` 内容部署到原服务工作目录；首次安装用 `bash install_service.sh`，已有服务更新后执行 `sudo systemctl restart rdk-disc.service`。查看日志用 `journalctl -u rdk-disc.service -f`。主控、RDK 两端一起更新后再上电联调。

建议实机按顺序验收：G0 完成前 ARM 被拒绝；G100/G1/G103 顺序正确；检测球先停稳再 G104；旧 UID 不放行、新 UID 才恢复；暂停后保持原绕桩终点；最后验证 STOP、无完成回包、RFID 缺失和串口故障。
