# 阶梯任务（2026-09-20）

> 2026-09-24 后续流程已扩展：阶梯结束 → G3 → 后退 100 → 固定方向左移 1300 同时逆时针转 180° → 向右找线 → 三列仓库任务。以 [阶梯后续流程](../docs/STAIR_EXIT.md) 和 [仓库任务](README_WAREHOUSE_TASK.md) 为准；下文“第八点即 PATH_DONE”的旧描述不再适用。

## 当前流程

绕桩结束并收到 PILLAR_DONE 后：后退 330 → 动作组 2 完成 → 原地正转 180° → 灰度找线（最多 50 秒）→ 停稳 → 动作组 105 完成 → 八个阶梯识别点。

取消旧阶梯流程的起始前进 18。第八点完成后停车并置 PATH_DONE，不再进入原来的横移 1650 和仓库路线。

| 点位 | 识别到球执行 | 本点完成后移动 |
|---|---|---|
| 1 | G106 | 后退 90 |
| 2 | G106 | 后退 117 |
| 3 | G107 | 后退 90 |
| 4 | G107 | 后退 90 |
| 5 | G107 | 后退 90 |
| 6 | G107 | 后退 117 |
| 7 | G108 | 后退 90 |
| 8 | G108 | 停车结束 |

各点停稳后观察新鲜画面 1 秒，正常有效球立即触发，不使用圆盘提前触发。无球则跳过动作组，继续表中移动。每次抓取等待舵控完成回包，再等待 RFID 确认，确认后才移动。阶梯计数从零开始，与圆盘机和绕桩计数分开；有效 UID 按原逻辑全程去重。确认两球后不再启动识别或夹取，仍走完所有剩余点位。

每个识别点单独打开、预热、关闭摄像头；1 秒是取得本点新鲜画面后的观察时间，不包含摄像头启动和白平衡预热。摄像头启动失败、无新鲜帧或画面停滞是错误，不能当作无球。动作完成回包超时 30 秒；完成后 RFID 确认超时 30 秒；单点通信总超时 70 秒。错误/取消停车，已开始的舵控动作按既有逻辑等待完成，不再发后续动作。

尺寸来自 `rdk_vision/task_sizes.yaml` 的 `stair`，HSV、ROI、形状判定来自公共 `config.yaml`。此次不覆盖已在 RDK 保存的这两个文件。圆盘机、绕桩原有参数和触发方式继续保留。

## 增量上传（已安装 task_config.py 和 task_sizes.yaml 的设备）

先让机器人停止任务。在 RDK 远程桌面终端：

```bash
mkdir -p ~/Desktop/stair-update
```

Windows PowerShell：

```powershell
cd "F:\licang2026\2026ROBOCUP-ZHY\2026ROBOCUP-ZHY\RDK"
scp "./tools/stair_task.py" "./tools/rdk_stm32_bridge.py" "./tools/disc_rfid_gate.py" "sunrise@192.168.128.10:/home/sunrise/Desktop/stair-update/"
```

RDK 远程桌面终端：

```bash
cd /home/sunrise/licang_vision
sudo systemctl stop rdk-disc.service
backup_dir="$HOME/rdk-stair-backup-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$backup_dir"
cp -a tools rdk_vision "$backup_dir/"
cp ~/Desktop/stair-update/stair_task.py tools/
cp ~/Desktop/stair-update/rdk_stm32_bridge.py tools/
cp ~/Desktop/stair-update/disc_rfid_gate.py tools/
python3 -m py_compile tools/stair_task.py tools/rdk_stm32_bridge.py tools/disc_rfid_gate.py
python3 - <<'PY'
from rdk_vision.task_config import load_task_config
for task in ('disc', 'pillar', 'stair'):
    cfg = load_task_config('rdk_vision/config.yaml', task)
    print(task, cfg.ball)
PY
```

检查通过后还需更新 STM32 固件，两个端都更新后再启动任务。不要上传本地默认 `config.yaml` 或 `task_sizes.yaml` 覆盖设备上已调好的参数。

## STM32 编译及 OpenOCD 下载

Windows PowerShell，从工程根目录执行：

```powershell
cd "F:\licang2026\2026ROBOCUP-ZHY\2026ROBOCUP-ZHY"
cmake --build build/Release-Path --parallel
```

编译成功后使用已安装的 OpenOCD 和正点原子无线 DAP：

```powershell
$OpenOcd = "E:/cubeIDE/STM32CubeIDE_1.18.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.win32_2.4.100.202501161620/tools/bin/openocd.exe"
$OpenOcdScripts = "E:/cubeIDE/STM32CubeIDE_1.18.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.debug.openocd_2.3.100.202501240831/resources/openocd/st_scripts"
& $OpenOcd -s $OpenOcdScripts -f interface/cmsis-dap.cfg -c "cmsis_dap_backend hid" -c "transport select swd" -f target/stm32f7x.cfg -c "adapter speed 1000" -c "program build/Release-Path/chassis_motor.hex verify reset exit"
```

最后在 RDK 上恢复服务：

```bash
sudo systemctl start rdk-disc.service
journalctl -u rdk-disc.service -n 60 -f -o cat
```

## 日志和协议

STM32 只在停稳时发送 `STAIR_CHECK N`（N=1..8）。RDK 回复 `STAIR_ACK N`。

- 无球：`STAIR_NONE N`，STM32 继续下个点位。
- 有球：运行对应动作组，完成后发送 `STAIR_ACTION_DONE N`；STM32 此时开放读卡；确认新 UID 后发送 `STAIR_RFID_OK N`；RDK 回复 `STAIR_DONE N`，STM32 才计数并移动。
- 失败：`STAIR_ERROR N`，停车并取消后续动作。
- 满两个球：后续点位不再发送 STAIR_CHECK，底盘继续移动至最后一点。

不同点位的确认消息不能互相放行，圆盘机/绕桩的 RFID 回复也不能放行阶梯。RDK 单个 STAIR_DONE 仅表示一个点位完成；整个任务的结束标志是 STM32 的 `PATH_DONE`（step=9，point=7）。
