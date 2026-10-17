# 当前方案：反向绕桩，正常识别停车

本说明取代独立绕桩 ROI / 球心窗口方案。

- 圆盘：保持原 `config.yaml`、ROI、HSV、尺寸/形状参数、右向左提前触发。
- 绕桩：同样读取设备上的 `config.yaml`，正常 `BallDetector.valid=True` 且有 bbox 的新鲜帧立即请求停车。不检查提前触发线，不读取 `pillar.yaml`，不要求球心进入特定窗口。
- 底盘停稳后才允许 G104；动作完成后等新 distinct RFID，再继续绕桩。G103 准备、RFID 去重、超时和 STOP 逻辑保持原样。
- 反向沿原圆周运行：车体系前进分量从 +58.9 改为 -58.9，旋转分量从 +49 改为 -49；保持速度大小、半径和原一圈目标，结束判定从 +352° 改为 -352°。抓取暂停后也按反方向恢复。
- “正常识别”仍要求原尺寸/形状/面积和 ROI 边缘过滤全部通过。`reason=edge_margin` 不触发；用户此前截图正是这一状态。若实际整段经过都被边缘裁剪，需根据日志重新评估共享 ROI，此修改不放宽公共过滤参数。
- 旧 `pillar_profile.py`、`pillar.yaml` 和 `pillar_preview.py` 可以保留，但不再参与正式任务；旧专用预览不能用于判断当前触发条件。普通 `rdk_vision.calibrate` 的 valid 状态对应当前基础检测条件（须先停任务和服务）。

## 增量更新

本次必须同时更新主控固件和 RDK。先 STOP，再在 RDK 远程桌面终端：

```bash
sudo systemctl stop rdk-disc.service
backup_dir="$HOME/rdk_normal_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$backup_dir"
cp -a ~/licang_vision/tools "$backup_dir/"
echo "$backup_dir"
mkdir -p ~/Desktop/rdk_normal_patch
```

Windows PowerShell 上传三个文件（不覆盖 config.yaml）：

```powershell
cd "F:\licang2026\2026ROBOCUP-ZHY\2026ROBOCUP-ZHY\RDK"
scp .\tools\rdk_stm32_bridge.py .\tools\vision_servo_direct_test.py .\tools\pillar_ball_trigger.py sunrise@192.168.128.10:~/Desktop/rdk_normal_patch/
```

RDK 远程桌面终端：

```bash
/usr/bin/python3 -m py_compile ~/Desktop/rdk_normal_patch/*.py
cp ~/Desktop/rdk_normal_patch/rdk_stm32_bridge.py ~/licang_vision/tools/
cp ~/Desktop/rdk_normal_patch/vision_servo_direct_test.py ~/licang_vision/tools/
cp ~/Desktop/rdk_normal_patch/pillar_ball_trigger.py ~/licang_vision/tools/
cd ~/licang_vision
/usr/bin/python3 -c "import sys; sys.path.insert(0,'tools'); import rdk_stm32_bridge; print('Import OK')"
```

每步无报错再继续。主控新固件：`build/Release-Path/chassis_motor.hex`，按已验证的 OpenOCD/CMSIS-DAP 方法烧录。两端均更新后启动 RDK 服务：

```bash
sudo systemctl restart rdk-disc.service
journalctl -u rdk-disc.service -n 50 -f -o cat
```

绕桩日志应显示 `PILLAR NORMAL DETECTION`；识别成功依次出现 `PILLAR_BALL N`、`PILLAR_STOPPED N`、G104 完成、RFID 确认、`PILLAR_RESUME N`。主控复位握手后会执行 G0。
