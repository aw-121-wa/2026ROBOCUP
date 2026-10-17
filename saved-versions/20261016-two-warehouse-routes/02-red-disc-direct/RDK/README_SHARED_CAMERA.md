# 绕桩至阶梯连续取图

正式桥接服务在 G103 阶段打开并预热摄像头。绕桩结束及阶梯移动期间继续取图，但不会触发阶梯夹取。每次 STAIR_CHECK 到达后，仅检测该请求之后的新帧，观察窗口仍为 1 秒。低／高／中点位继续分别加载 stair_low.yaml、stair_high.yaml、stair_mid.yaml。

G3（阶梯结束）或 G0（重启任务）开始前关闭保留的摄像头。任务异常、取消后工作线程结束、服务退出或重新启动圆盘机任务时也释放摄像头。满两个球后虽然跳过剩余识别点，G3 仍会关闭摄像头。独立调用 stair_task 的默认行为不变，正式桥接才使用共享摄像头。

摄像头设备、宽、高、帧率必须一致；不一致时报错，不静默使用不匹配的视频。ROI、HSV、球尺寸与图像过期阈值仍按任务配置独立使用。白平衡从绕桩预热后持续锁定，不再在每个阶梯点重新预热，因此需要在真实光照下验证识别。

## 更新 RDK

电脑 PowerShell：

```powershell
cd "F:\licang2026\2026ROBOCUP-ZHY\2026ROBOCUP-ZHY\RDK"
scp .\tools\shared_task_camera.py .\tools\stair_task.py .\tools\rdk_stm32_bridge.py sunrise@192.168.128.10:~/Desktop/
```

RDK 终端（机器人停止时）：

```bash
cd /home/sunrise/licang_vision
sudo systemctl stop rdk-disc.service
backup_dir=$(mktemp -d ./backup-shared-camera-XXXXXX)
cp tools/rdk_stm32_bridge.py tools/stair_task.py "$backup_dir/"
if [ -f tools/shared_task_camera.py ]; then
    cp tools/shared_task_camera.py "$backup_dir/"
fi
cp ~/Desktop/shared_task_camera.py ~/Desktop/stair_task.py ~/Desktop/rdk_stm32_bridge.py tools/
python3 -m py_compile tools/shared_task_camera.py tools/stair_task.py tools/rdk_stm32_bridge.py && sudo systemctl start rdk-disc.service
systemctl is-active rdk-disc.service
journalctl -u rdk-disc.service -n 40 --no-pager -o cat
```

仅更新上述3个程序文件，不上传 YAML，也不覆盖板端已经修好的 camera.py。本次 STM32 无修改，无需编译烧录。

## 实车验收

日志从绕桩开始到阶梯结束应仅出现一次 `SHARED CAMERA: opened`；各阶梯点不再重复 AWB warm-up；G3 前出现 `SHARED CAMERA: closed`。检查各点 STAIR CONFIG 的 low/high/mid 与 G106/G107/G108 对应正确。移动时不得触发夹取，停稳发出 STAIR_CHECK 后才允许判定。

摄像头预览仍需先停止正式服务，避免两个程序占用同一摄像头。出现无新帧、取流中断或配置不匹配时应报错，不作为无球继续。
