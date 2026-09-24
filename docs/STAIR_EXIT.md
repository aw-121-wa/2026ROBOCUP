# 阶梯结束后的移位与找线

阶梯仍访问全部八个点位，最多夹取两个 RFID 确认的球。完成最后点位后继续：

1. step=10：发送 `GROUP 3`，等待舵控完成回包，最长 30 秒；完成后先后退 100 mm。
2. step=11：相对于本段起始朝向，沿固定左侧方向直线移动 1500 mm，同时逆时针旋转 180°。平移速度上限为 60 rpm 对应的线速度，最长 30 秒。
3. 平移目标完成且最终角度误差小于 0.5°、角速度小于 2°/秒持续十个控制周期后请求停车；等待现有底盘停稳条件成立。
旋转横移完成并停稳后，先沿此时车身方向后退 200 mm，再进入下一步找线。该后退独立于仓库第一列前的后退 200 mm。

4. step=12：向此时车身右侧横移找线，速度为 25 rpm 对应的线速度，最长 50 秒。两路中间灰度同时检测到线后立即请求停车，检测稳定 50 ms 且底盘停稳后进入 step=13 仓库任务。
5. step=13：每列之前后退 200 mm，共三列；按 RFID 记录选择槽位，再以 G109/G110/G111 放置对应行，无球跳过。第三列完成才报告 `PATH_DONE`。详见 [仓库任务及增量部署](../RDK/README_WAREHOUSE_TASK.md)。

同步段按路径进度推进目标航向，按实际航向变化转换车身平移速度；移动中车身转向，场地中的目标平移方向保持不变。沿用当前轮速指令积分作为距离估计，并非新增编码器位置闭环。实车需验证地面打滑、实际位移和制动误差。若平移先完成，继续校正剩余航向再结束该段。

找线结束后执行上述按 RFID 编号入库流程。故障、取消和超时走已有停车/任务取消路径。

## 部署

需要更新 STM32 固件及 RDK 的 `tools/rdk_stm32_bridge.py`，两端动作组白名单包含 3、109、110、111。RDK 视觉配置无需更新。舵控板应已存有这些动作组。

Windows PowerShell，在本地 RDK 目录执行，先上传至远程桌面暂存：

```powershell
cd "F:\licang2026\2026ROBOCUP-ZHY\2026ROBOCUP-ZHY\RDK"
scp .\tools\rdk_stm32_bridge.py sunrise@192.168.128.10:/home/sunrise/Desktop/rdk_stm32_bridge.py
```

RDK 远程桌面终端，在任务未运行时安装：

```bash
cd ~/licang_vision
sudo systemctl stop rdk-disc.service
cp -a tools/rdk_stm32_bridge.py "tools/rdk_stm32_bridge.py.bak-$(date +%Y%m%d-%H%M%S)"
cp ~/Desktop/rdk_stm32_bridge.py tools/rdk_stm32_bridge.py
python3 -m py_compile tools/rdk_stm32_bridge.py
sudo systemctl start rdk-disc.service
journalctl -u rdk-disc.service -n 50 -f -o cat
```

本次已生成主控固件 `build/Release-Path/chassis_motor.elf`。本地验证命令：

```powershell
cmake --build build/host-path
ctest --test-dir build/host-path --output-on-failure
python -m unittest discover -s RDK/tests
cmake --build build/Release-Path
```
