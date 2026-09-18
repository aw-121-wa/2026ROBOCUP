# RDK 圆盘任务：STM32 融合基线

本包是当前实机测试后的 RDK 圆盘任务基线，用于继续与 STM32F750 底盘任务层融合。

## 已固定功能

- STM32 专用链路：`/dev/ttyUSB0`，115200 8N1
- 幻尔舵控板：`/dev/ttyS1`，9600 8N1
- STM32 -> RDK：`PING\r\n`、`DISC_START\r\n`、`DISC_RFID_OK N\r\n`、`DISC_CANCEL\r\n`
- RDK -> STM32：`PONG\r\n`、`DISC_ACK\r\n`、`DISC_ACTION_DONE N\r\n`、`DISC_DONE\r\n`、`DISC_ERROR\r\n`
- 圆盘动作：G101 ×1，G102 ×5
- 红球右向左运动提前触发：`trigger_x = 380`
- Camera 启动时固定基础 UVC 参数，先开启 AWB 预热 45 帧，再关闭 AWB 锁定内部颜色增益
- Camera/AWB 预热与 G101 并行，二者都完成后才进入红球识别
- systemd：`rdk-disc.service` 上电自启动，未收到 `DISC_START` 时不会自行执行机械臂任务

## 当前最终标定

- ROI：x=92, y=29, width=352, height=244, edge_margin=4
- 红色 HSV：`[0,100,60,10,255,255]` + `[170,100,60,179,255,255]`
- reference ball size：148 × 150

## STM32 侧建议接口

第一阶段继续保持最小协议：

1. 上电后可周期性发送 `PING\r\n`，收到 `PONG\r\n` 表示 RDK 在线。
2. 底盘到达圆盘任务点后发送 `DISC_START\r\n`。
3. 收到 `DISC_ACK\r\n` 后 STM32 保持底盘静止。第一次视觉 trigger 可直接执行 G102。
4. 每次收到 `DISC_ACTION_DONE N\r\n` 才清空 RFID ring/partial frame 并打开 gate N；新 distinct UID 到达后回复 `DISC_RFID_OK N\r\n`。
5. RDK 等待 RFID 时继续逐帧 capture、detect 和 trigger update，但 suppress G102；确认后只允许确认后拍到的 fresh、non-stale frame 触发下一次动作。
6. `DISC_ACTION_DONE 5` 后仍等待 RFID 5 且绝不开放第六次；收到 `DISC_RFID_OK 5` 后 RDK 才发送 `DISC_DONE\r\n`。
7. 收到 `DISC_ERROR\r\n` 后进入异常处理，不得按完成处理。

`DISC_CANCEL` 只用于 PC_CANCEL、mission timeout 或本地故障安全收尾：它关闭 RDK action gate，不开启新事务，也不重置圆盘总 deadline；已经发出的动作允许等待完成，但禁止新的 G102。

bridge 主线程始终处理 STM32 UART，圆盘任务在唯一 worker thread 内运行；因此任务活动期仍可回复 `PING/PONG` 并接收 RFID 确认。串口发送由同一把锁保护，重复 `DISC_START` 不会创建第二个 Camera/Hiwonder worker。

## 安装

```bash
cd ~/licang_vision
unzip -o rdk_disc_stm32_integration_latest.zip
chmod +x install_service.sh uninstall_service.sh
./install_service.sh
```

日志：

```bash
journalctl -u rdk-disc.service -f
```
