# 圆盘与绕桩分离识别位置

> **此方案已停用。** 当前使用共享圆盘 ROI、正常识别停车和反向绕桩，部署请看 [README_PILLAR_NORMAL.md](README_PILLAR_NORMAL.md)。以下仅保留历史记录。

圆盘继续读取设备上的 `rdk_vision/config.yaml`，沿用原 ROI 和 `RightToLeftDiscTrigger` 提前触发条件。无需修改主控协议或重新烧录。

绕桩每次启动时先读取同一份 `config.yaml`，继承 HSV、相机、尺寸、面积、填充率等参数；再用 `rdk_vision/pillar.yaml` 只覆盖 ROI，使用独立球心位置窗口触发。**不要用本地 config.yaml 覆盖 RDK 的实测配置。**

## 初始位置与判定

根据用户最佳抓取位置截图，候选框约 `(266,125,152,148)`，中心约 `(342,199)`。图中原圆盘 ROI 为 `(92,29,352,244)`，其底部裁剪球，显示 `edge_margin`。

新增绕桩 ROI `(230,90,230,230)`，球心目标 `(342,199)`，X/Y 容差各 12 像素。ROI 是检测区域；青色的小窗口是球心允许触发位置，不能混为一谈。

球必须通过共享的全部基础过滤，并且球心同时落入 X/Y 容差，才发送 `PILLAR_BALL N`。不再接受圆盘的部分露出/边缘提前触发。之后仍然由主控停稳许可 → G104 → 新 RFID → 恢复绕桩。

参数是截图估计的初值，不是完成实车标定的保证。静止时最佳抓取位置与运动时最佳停车触发位置会因串口和制动延迟不同。预览确认后低速实测，必要时向球运动来向调整 `center_x/center_y`；扩大容差也会使首次触发更早。不要为了绕桩更改公共 HSV/尺寸参数，否则会同时影响圆盘。

## 增量更新

在 RDK 远程桌面终端，先让主控 STOP，再停止服务并备份：

```bash
sudo systemctl stop rdk-disc.service
backup_dir="$HOME/rdk_roi_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$backup_dir"
cp -a ~/licang_vision/tools "$backup_dir/"
cp -a ~/licang_vision/rdk_vision "$backup_dir/"
echo "$backup_dir"
mkdir -p ~/Desktop/rdk_roi_patch
```

Windows PowerShell 上传五个文件，无需 SSH 交互登录：

```powershell
cd "F:\licang2026\2026ROBOCUP-ZHY\2026ROBOCUP-ZHY\RDK"
scp .\tools\rdk_stm32_bridge.py .\tools\vision_servo_direct_test.py .\tools\pillar_profile.py .\tools\pillar_preview.py .\rdk_vision\pillar.yaml sunrise@192.168.128.10:~/Desktop/rdk_roi_patch/
```

回到 RDK 远程桌面终端，检查文件完整再替换：

```bash
ls -l ~/Desktop/rdk_roi_patch/
cp ~/Desktop/rdk_roi_patch/rdk_stm32_bridge.py ~/licang_vision/tools/
cp ~/Desktop/rdk_roi_patch/vision_servo_direct_test.py ~/licang_vision/tools/
cp ~/Desktop/rdk_roi_patch/pillar_profile.py ~/licang_vision/tools/
cp ~/Desktop/rdk_roi_patch/pillar_preview.py ~/licang_vision/tools/
cp ~/Desktop/rdk_roi_patch/pillar.yaml ~/licang_vision/rdk_vision/
cd ~/licang_vision
/usr/bin/python3 tools/pillar_preview.py
```

此预览仅使用摄像头，不打开舵控串口、不发送任何动作。黄色是检测 ROI，青色是目标球心窗口，绿色球框代表基础检测有效；`STOP POSITION (preview only)` 才表示满足正式绕桩的触发条件。红色候选框和 `reason` 用于检查过滤失败原因。`STALE` 帧不会触发。

预览是独立工具，不可和正式服务同时运行。按 q 或 Esc 退出。需要调位置时编辑 `~/licang_vision/rdk_vision/pillar.yaml`，重新打开预览；程序不自动保存截图位置或滑条值。保留相机 640×480 分辨率。

确认预览后恢复服务：

```bash
sudo systemctl start rdk-disc.service
journalctl -u rdk-disc.service -n 50 -f -o cat
```

绕桩日志应出现 `PILLAR POSITION TRIGGER`，圆盘仍显示原提前触发提示。若修改 pillar.yaml，下一次绕桩任务会重新读取。文件缺失或参数无效会报错停止该绕桩任务，不会自动退回圆盘 ROI。
