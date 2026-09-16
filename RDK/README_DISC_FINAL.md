# RDK X5 圆盘机红方最终实测包

本包基于已实机验证的圆盘机程序，并固化当前最新标定数据。

## 当前最新标定数据

- ROI: x=92, y=29, width=352, height=244
- edge_margin=4
- reference_width=142
- reference_height=144
- aspect_ratio=0.75~1.30
- fill_ratio=0.50~0.90
- 红球 HSV:
  - [0,100,60,10,255,255]
  - [170,100,60,179,255,255]

## 圆盘抓取逻辑

- 圆盘顺时针；当前相机画面中球从右向左运动。
- G101 启动时执行 1 次。
- G102 完成 5 次后任务结束。
- 默认 trigger_x=380；X 越大触发越早，X 越小触发越晚。
- 支持 TRIGGER_EARLY / TRIGGER_EDGE / TRIGGER_VALID。
- G102 完成后立即重新识别，不等待目标离开两帧。

## 运行

```bash
cd ~/licang_vision
python3 tools/vision_servo_direct_test.py \
  --config rdk_vision/config.yaml \
  --color red \
  --servo-port /dev/ttyS1 \
  --servo-baud 9600 \
  --prep-group 101 \
  --trigger-group 102 \
  --repeat 1 \
  --max-actions 5 \
  --trigger-x 380 \
  --servo-timeout 30
```

`--trigger-x 380` 可省略，因为已固化为默认值 380。

## 上电自启动 / STM32 任务触发

本包新增 `tools/rdk_stm32_bridge.py`，开机后只等待 STM32 指令，不会自行执行机械臂任务。

RDK 串口分工：

- `/dev/ttyS1` @ 9600: 幻尔舵控板
- `/dev/ttyUSB0` @ 115200: STM32 UART4 (PC10/PC11 经 USB-TTL)

第一版 ASCII 协议（均以 `\r\n` 结尾）：

- STM32 -> RDK: `PING`；RDK -> STM32: `PONG`
- STM32 -> RDK: `DISC_START`
- RDK 先回复 `DISC_ACK`
- 然后运行现有圆盘任务：G101 x1 -> 红球识别 -> G102 x5
- 成功后回复 `DISC_DONE`
- 任务进程异常退出时回复 `DISC_ERROR`

### 手动运行 bridge（安装 systemd 前先测试）

```bash
cd ~/licang_vision
python3 tools/rdk_stm32_bridge.py \
  --stm32-port /dev/ttyUSB0 \
  --stm32-baud 115200
```

### 安装为 systemd 上电自启动

```bash
cd ~/licang_vision
chmod +x install_service.sh uninstall_service.sh
./install_service.sh
```

查看状态：

```bash
systemctl status rdk-disc.service
```

实时日志：

```bash
journalctl -u rdk-disc.service -f
```

停止服务：

```bash
sudo systemctl stop rdk-disc.service
```

重新启动：

```bash
sudo systemctl restart rdk-disc.service
```

取消开机自启并删除服务：

```bash
cd ~/licang_vision
./uninstall_service.sh
```

如果 `/dev/ttyUSB0` 上电时尚未出现，bridge/systemd 会自动重试；不会因此自行执行 G101/G102。只有收到 `DISC_START\r\n` 才启动圆盘任务。
