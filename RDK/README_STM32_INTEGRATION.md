# RDK 圆盘任务：STM32 融合基线

本包是当前实机测试后的 RDK 圆盘任务基线，用于继续与 STM32F750 底盘任务层融合。

## 已固定功能

- STM32 专用链路：`/dev/ttyUSB0`，115200 8N1
- 幻尔舵控板：`/dev/ttyS1`，9600 8N1
- STM32 -> RDK：`PING\r\n`、`DISC_START\r\n`
- RDK -> STM32：`PONG\r\n`、`DISC_ACK\r\n`、`DISC_DONE\r\n`、`DISC_ERROR\r\n`
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
3. 收到 `DISC_ACK\r\n` 后 STM32 保持底盘静止并等待。
4. 收到 `DISC_DONE\r\n` 后进入下一任务节点。
5. 收到 `DISC_ERROR\r\n` 后进入异常处理，不得按完成处理。

当前 bridge 在圆盘子任务运行期间使用阻塞子进程，因此不支持任务执行中实时 `STOP/ABORT`；后续若需要中止机制，再升级为 `Popen + poll` 状态机。

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
