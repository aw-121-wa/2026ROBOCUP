# 2026-09-15 新基线：先测试 PING/PONG

## 本次工程

STM32 基于用户上传的 `2026ROBOCUP-master (1).zip`，替换工作区内的 `2026ROBOCUP-master` 源码目录。原工作区完整保存在同级 `before-20260915-replacement`。

RDK 基于 `rdk_disc_stm32_integration_latest_verified.zip`。保留 AWB 预热锁定、相机与 G101 并行准备、148×150 参考球尺寸、ROI 和触发参数。仅修复桥接 normalize_command 对 bytearray 的解码，并增加相应测试及 Windows 测试路径兼容。

上传的 STM32 原包使用带会话号的 Q/R 协议，与本次 RDK 的 PING/DISC_START 不兼容。已适配为 ASCII 协议；不再发送 GROUP/VISION/HELLO 等旧协议串口报文。旧协议辅助源码和 tests/legacy_q_protocol 仅用于历史参考，不代表当前任务行为，旧 tools/rdk/path_bridge.py 不应部署。

当前 PATH 范围：先重新握手 → 第一段平移 (1691.4467,615.6363) mm、85 RPM等效速度 → 第二段 (2300,0) mm、130 RPM等效速度 → 相对当前连续航向原地 +180° → 双中间灰度对齐 → DISC_START → 等待 RDK 整套圆盘任务完成 → 停留原地。方向沿用原工程正转约定。没有继续执行绕桩、阶梯、仓库。

原始机械参数及电机映射沿用这次上传的底盘工程，不用昨天旧底盘文件覆盖。Chassis_Rotate 使用连续航向目标，可跨 ±180°。

## 接线

| STM32 | RDK 侧 USB-TTL |
|---|---|
| UART4 PC10 TX | RX |
| UART4 PC11 RX | TX |
| GND | GND |

通信为 115200、8 数据位、无校验、1 停止位。TTL 电平需与板卡匹配，不能将 RS232 电平直接接入 PC10/PC11。RDK 当前桥接端口为 `/dev/ttyUSB0`。

机械臂仍由 RDK `/dev/ttyS1`、9600 控制，STM32 不发机械臂动作组命令。

灰度只读取 PD0(IN2)、PD1(IN1)，输入上拉、低电平有效。两路同时有效立即请求停车，持续 50 ms 且底盘输出已静止后启动圆盘任务；其他灰度输入不配置、不参与判定。寻找对齐限时 5 秒。中间灰度值：均未触发=0，仅PD1=2，仅PD0=4，两路=6。

## 部署

1. 烧录本次包内 STM32/firmware/Debug/chassis_motor.hex。工作区 Debug/Release 均已构建，先用 Debug 便于观察。
2. RDK 若已经装好本次新版视觉，只需将包内 RDK/tools/rdk_stm32_bridge.py 上传到 `/home/sunrise/rdk_stm32_bridge.py.new`，然后执行：

```bash
sudo systemctl stop rdk-disc.service
cp -n /home/sunrise/licang_vision/tools/rdk_stm32_bridge.py /home/sunrise/licang_vision/tools/rdk_stm32_bridge.py.before_20260915
cp /home/sunrise/rdk_stm32_bridge.py.new /home/sunrise/licang_vision/tools/rdk_stm32_bridge.py
sudo systemctl restart rdk-disc.service
sudo systemctl cat rdk-disc.service
```

确认 ExecStart 指向 `/home/sunrise/licang_vision/tools/rdk_stm32_bridge.py`。不要因 root SSH 登录而使用 `/root/licang_vision`。完整 RDK 源码也随包交付；已有实测视觉时不必重装全部文件或覆盖现场配置。

若此前已经修复 bytearray，先检查运行文件的 normalize_command 中是否为 `isinstance(raw, (bytes, bytearray))`。满足则不必重复替换。

## 现在只做握手

不发送 ARM、DISC、PATH。PING 不需要使能电机，也不会启动相机或机械臂。

RDK 终端：

```bash
sudo journalctl -u rdk-disc.service -b -n 10 -f
```

STM32 上位机连接原主机 UART5（115200），发送 ASCII 文本，逐条带 CRLF，间隔约 1 秒：

```text
STOP
RDK_RESET
PING
```

注意这三条发给 STM32，不是直接往 RDK 串口写。自动重复发送关闭。

正常 RDK 日志：

```text
STM32 RX: PING
STM32 TX: PONG
```

JustFloat 完整模式共 35 个 float，编号从 0 开始；以下编号适用于本次 Debug/Release 默认完整遥测：

| 通道 | 含义 | 握手通过 |
|---|---|---|
| CH14 | 电机使能 | 0 |
| CH16 | 最近主机命令结果 | 1 |
| CH18 | 运动/任务/通信忙或锁定 | 0 |
| CH27 | 任务结果 | 0（未启动） |
| CH28 | PATH 路段 | 0 |
| CH29 | 独立 PING 测试通过标志 | 1 |
| CH30 | 任务 IO 故障位 | 0 |
| CH31 | RDK 阶段 | 2（PONG 已收到） |
| CH32 | RDK 错误 | 0 |
| CH33 | 中间灰度位 | 随输入变化 |
| CH34 | PATH 子阶段 | 0 |

CH31：0空闲、1等PONG、2就绪、3等DISC_ACK、4圆盘运行、5完成、6锁定。
CH32：0正常、1超时、2RDK任务错误、3协议/顺序错误、4串口IO错误、5本地取消。
CH27：0空闲、1执行中、2完成、3取消、4超时、5错误。
CH30：1接收初始化失败、2接收环溢出、4重启接收失败、16UART错误、32运动超时；可按位组合。
CH15 是底盘故障，不替代 CH30/CH32。握手成功也不代表 IMU/电机/相机/机械臂已验证。

未收到 PONG 时约 2 秒后锁定，CH31=6。排除串口/服务问题后，在底盘已停止且未使能时发送 RDK_RESET，再发送 PING。STOP 不会取消独立 PING，正在等待的探测先等其回复或2秒超时再复位。

请先保存日志和 CH14、CH16、CH29、CH30、CH31、CH32 的值，确认握手后再安排 PATH。当前固件在本次上电没有独立 PING 成功记录时，会拒绝 PATH/DISC；RDK_RESET 会清除记录，PATH 启动时还会再做一次新握手。

## 后续任务约束

DISC_START 只发送一次，避免阻塞式 RDK 桥接在任务结束后执行重传的第二轮抓取。必须依次收到 DISC_ACK、DISC_DONE 才认为完成；任务总等待上限180秒，ACK等待2秒。完成后停留原地。

原版 RDK 不支持执行中远程 STOP：STM32 STOP 只能停止底盘并锁定本地任务，不能撤回机械臂已发送动作。任务超时/取消后，先在 RDK 确认任务已结束（必要时停止服务并确认机械臂状态），再重启服务、复位 STM32 链路；不要用重复 DISC/PATH 试图打断任务。

## 验证范围

本地 CTest 22 项、RDK 55 项测试通过，Debug/Release 编译通过。检查了 UART4 GPIO 复用、中断转发、115200参数、ASCII解析、超时和错误顺序、先握手门禁、双中间灰度及180度命令。保留未参与当前固件的旧底盘模块单测用于回归。

没有连接实物完成 PING/PONG，也没有运行完整 PATH；物理接线与运行结果以本次握手和后续现场测试为准。
