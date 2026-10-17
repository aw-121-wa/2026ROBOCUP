# RDK X5 红蓝球视觉 V1

该目录是现有 MaixCAM 红蓝球视觉的 **兼容替换模块**。V1 只负责感知和 `FOUND` 事件，不接管底盘、机械臂或仓库转盘。

```text
USB Camera -> RDK X5 -> UART -> STM32 UART4 -> existing ServoAction -> Hiwonder
```

RDK **绝不发送幻尔舵控板动作帧**。机械臂动作仍由 STM32 `ServoAction` 层唯一管理。

## 1. 电气与串口合同

STM32 UART4 当前配置：

```text
PC10 = STM32 TX
PC11 = STM32 RX
115200 baud, 8 data bits, no parity, 1 stop bit, no flow control
```

RDK 与 STM32 使用 **3.3 V TTL**，TX/RX 交叉并共地：

```text
RDK TX  -> STM32 PC11 / UART4 RX
RDK RX  <- STM32 PC10 / UART4 TX
RDK GND -- STM32 GND
```

RDK 实际串口节点由 `config.yaml` 的 `serial.device` 决定。接线前必须核对当前 RDK X5 系统映射，不能仅凭 `/dev/ttyS1` 名称猜物理针脚。

协议保持现有 MaixCAM 合同不变：

```text
STM32 -> RDK: raw byte '1'  = 请求红球（无换行）
STM32 -> RDK: raw byte '2'  = 请求蓝球（无换行）
RDK   -> STM32: bytes 1\n   = 本次请求成功，且最多回复一次
```

没有 `ACK`、`READY`、`NOT_FOUND`、错误帧或 CRC。未找到目标时 RDK 保持静默，由 STM32 现有 BALL / STAIR / RZ 超时策略决定后续流程。

## 2. 查找摄像头与串口节点

```bash
ls -l /dev/video*
ls -l /dev/ttyS* /dev/ttyUSB* 2>/dev/null
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --list-formats-ext
v4l2-ctl --device=/dev/video0 --list-ctrls
```

目标采集模式为 **640x480 @ 30 FPS**。如果摄像头不是 `/dev/video0`，修改 `camera.device`。

## 3. Python 依赖

先确认板载/系统 OpenCV 可用：

```bash
python3 -c "import cv2, numpy, yaml; print(cv2.__version__)"
python3 -m pip install -r rdk_vision/requirements.txt
```

`requirements.txt` **故意不安装 `opencv-python` / `opencv-python-headless`**，优先使用 RDK 镜像已经验证过的 OpenCV。

## 4. 视觉判定

V1 使用：

```text
latest frame only
-> HSV mask
-> 3x3 open/close morphology
-> contour area / bounding size
-> ROI edge margin
-> aspect ratio
-> reference-size range
-> mask pixel count
-> fill ratio
-> 2 hits among newest 3 processed frames
-> FOUND
```

只有请求之后拍到的新 `frame_id` 才能参与判断；帧龄超过 **250 ms** 的画面禁止进入识别和 2-of-3 历史。

初始 ROI：

```text
x=240, y=150, width=160, height=140, edge_margin=4
```

仓库中的 HSV 参数只是软件/合成测试的**起始值，不是比赛最终阈值**。必须使用实际 RDK X5、USB 摄像头、球、补光和机械安装姿态重新标定。

## 5. 现场标定

红球：

```bash
python3 -m rdk_vision.calibrate --config rdk_vision/config.yaml --color red
```

蓝球：

```bash
python3 -m rdk_vision.calibrate --config rdk_vision/config.yaml --color blue
```

按键：

```text
r       切换红球参数
b       切换蓝球参数
s       保存当前 ROI / HSV；若当前有有效球，同时保存其 bbox 宽高为参考尺寸
q / ESC 退出，不自动保存
```

Trackbar 可调整 ROI 和 HSV。黄色框是抓取 ROI；有效候选用绿色框，最强被拒候选用红色框。保存前要求球的完整 bbox 至少在 ROI 边界内留出 `edge_margin`。

### 曝光和白平衡

使用：

```bash
v4l2-ctl --device=/dev/video0 --list-ctrls
```

先让自动曝光/白平衡在现场光照下稳定，再记录并尽可能锁定实际 USB 摄像头支持的曝光、白平衡和增益。**不要复制 OpenMV/MaixCAM 的 gain/exposure 数值。**

## 6. 比赛视觉服务

```bash
python3 -m rdk_vision.main --config rdk_vision/config.yaml
```

启动顺序固定为：加载配置 -> 打开摄像头 -> 等待第一帧 -> 打开 UART -> 进入 IDLE。调试文本只写终端/日志，不会写入协议 UART。

## 7. UART-only 冒烟测试

在接机械臂自动动作前先验证协议：

1. 向 RDK 发送原始字节 `'1'`，**不要发送换行**。
2. 将红球完整放入 ROI，保持至少两个合格新帧。
3. 只允许收到一次 `1\n`。
4. 球保持原位 10 秒，不允许收到第二次回复。
5. 再发送新的 `'1'`，才允许重新识别。
6. 发送 `'2'` 时红球不得触发；蓝球满足条件后只回复一次 `1\n`。

## 8. 比赛验收顺序

严格按以下顺序联调：

```text
UART-only -> BALL -> STAIR -> RZ
```

### BALL

至少完成 10 次独立请求：

```text
正确触发 >= 10/10
错误颜色 = 0
重复触发 = 0
无视觉事件却动作 = 0
```

随后至少完成 3 次不中断的完整 BALL 流程，RDK 进程不重启。

### STAIR

分别验证：

- 初始位置已有球：静态识别后抓取；
- 90 mm 搜索过程中进入窗口：视觉 early-stop 能及时停车；
- 全段无球：RDK 保持静默，STM32 正常走完当前搜索段。

日志中的 `software_latency_ms`：

```text
目标 < 80 ms
可接受 < 120 ms
> 150 ms 必须调查
> 250 ms 判 V1 不通过
```

发生明显过冲时，先排查 USB 缓存和实际 FPS，不要先改 90 mm 距离或机械动作组。

### RZ

最后验证四轮连续事务：

```text
request -> fresh-frame detection -> FOUND -> stop/settle
-> Group4 -> turntable one slot -> new request
```

四次新请求都必须清空前一轮 2-of-3 历史；Group4/转盘期间 RDK 已回到 IDLE，不得重复回复。

## 9. 软件测试

```bash
python3 -m unittest discover -s rdk_vision/tests -v
python3 -m unittest tests/test_competition_cleanup.py -v
python3 licang_BLUE_RED_BALL.py --selftest
```
