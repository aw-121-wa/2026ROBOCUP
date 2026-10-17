# 阶梯连续慢速扫描

## 正式流程

绕桩后原阶梯找线和航向纠正保留，删除入口退距（旧55、最新源码25）。执行G105，完成起始对齐后记录距离零点。沿原阶梯后退方向以速度参数20运动：

|累计距离/mm|配置|发现球后的动作|结束动作|
|--|--|--|--|
|0–120|stair_low.yaml|停稳、G106、RFID确认|120停稳执行G4|
|120–520|stair_high.yaml|停稳、G107、RFID确认|520停稳后继续|
|520–860|stair_mid.yaml|停稳、G108、RFID确认|860停稳执行G3|

取消G3之后原有后退100。后续横移旋转及仓库任务保持此次修改前的代码（包括最新的旋转横移后退180）。

距离采用既有轮速里程的起始轴投影，不是计时累积，也不是编码器实测。夹取暂停不归零，刹车位移计入；再次运动只规划到下一边界的剩余距离。实车需校验打滑造成的距离误差。速度20是原路径速度参数，与原40一致的单位换算，并非20mm/s。

RFID最多确认2球，满额后仍走完三段、执行G4和G3。每次夹取后等待3张无有效球的新帧才允许再次触发，避免未离开ROI的同一球重复夹取。观察不再限制1秒。检测到球先发送停车请求，必须收到STM32停稳授权才执行动作组。阶段边界优先，未授权的发现可被阶段结束撤销；已经授权的夹取必须完成RFID后再结束。摄像头全程复用，G3前释放；保留现有圆盘机和绕桩流程。

## 协议和超时

新增 `STAIR_SCAN 1/2/3` 请求。对应使用低/高/中配置，不发送G103或重复G105。为复用已验证RFID通道，扫描内部仍使用 `PILLAR_ACK/READY/BALL/STOPPED/ACTION_DONE/RFID_OK/RESUME/END/DONE` 帧，日志同时打印 `STAIR_SCAN level=... group=... continuous` 区分任务。

扫描每段超时180秒；阶梯开始移动至结束总时限300秒（包含夹取等暂停）；单次夹取等待30秒、RFID等待30秒、停车授权等待5秒、阶段结束等待5秒。无新帧或取消属于错误，不按无球继续。CH46的阶梯点位现在表示分段编号1/2/3，不再是原8点。

## 更新

本次必须同时烧录新STM32固件和更新RDK。只更新一端不能完成新协议握手。固件：`build/Release/chassis_motor.elf`；尚未自动烧录。

电脑PowerShell上传（仅程序，不覆盖YAML、camera.py）：

```powershell
cd "F:\licang2026\2026ROBOCUP-ZHY\2026ROBOCUP-ZHY\RDK"
ssh sunrise@192.168.128.10 "mkdir -p /home/sunrise/Desktop/stair-continuous-update"
scp .\tools\stair_scan.py .\tools\rdk_stm32_bridge.py sunrise@192.168.128.10:/home/sunrise/Desktop/stair-continuous-update/
```

前提：上次共享摄像头更新已安装（shared_task_camera.py及支持camera参数的stair_task.py）。

机器人停止时，在RDK执行：

```bash
cd /home/sunrise/licang_vision
sudo systemctl stop rdk-disc.service
backup_dir=$(mktemp -d ./backup-stair-continuous-XXXXXX)
cp tools/rdk_stm32_bridge.py "$backup_dir/"
if [ -f tools/stair_scan.py ]; then cp tools/stair_scan.py "$backup_dir/"; fi
python3 -m py_compile ~/Desktop/stair-continuous-update/stair_scan.py ~/Desktop/stair-continuous-update/rdk_stm32_bridge.py &&
cp ~/Desktop/stair-continuous-update/stair_scan.py ~/Desktop/stair-continuous-update/rdk_stm32_bridge.py tools/
python3 -c "import sys; sys.path.insert(0,'tools'); import rdk_stm32_bridge; print('IMPORT OK')"
```

确认复制、导入成功且STM32新固件已烧录后，再启动：

```bash
sudo systemctl start rdk-disc.service
systemctl is-active rdk-disc.service
journalctl -u rdk-disc.service -f -n 20 -o cat
```

首次验收：先不放球检查120、520、860位置及G4/G3；再检查三段各自的停车夹取；最后验证两球提前满额后仍走完路程。回退时STM32和RDK需同时回到旧版。
