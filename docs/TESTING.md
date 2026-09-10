# RoboChassis V2 测试

## 编译及桌面测试

在工程根目录 PowerShell 执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake -S tests -B build/host -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

固件位于 `build/Debug/chassis_motor.elf`，同目录生成 HEX、BIN、map。
使用已有 STM32 下载器下载 ELF/HEX；BIN 起始地址为 `0x08000000`。本次不自动烧录。

## 接线及协议

- JY60：USART2，PD6 RX 接传感器 TX，9600、8N1，共地。沿用当前工程实际接线。
- X42S：USART3，PB10 TX，PB11 RX，921600、8N1。驱动器须选 Emm F6 速度协议、固定 0x6B 校验，并已使能。
- 轮序 FL/FR/RL/RR，默认 ID 2/1/3/4；必须按实机调整。
- 默认 `chassis_config.command_mode=ZDT_MULTI_COMMAND`：`00 AA 00 25` + 四个8字节F6子命令（同步位00）+ `6B`，共37字节，一次DMA发送。手册长度字段包含整帧，不能填0020。
- `ZDT_LEGACY_SYNC` 保留四个同步F6及FF66、至少3ms帧间隔；需在解除解锁且发送空闲时切换，或修改配置后重启。无需在多机模式中追加FF66。
- 这是发送命令里程计，无编码器反馈；遥测 velocity 不代表实测轮速。串口发送成功不等于驱动器执行成功。
- DMA 缓冲区要求 D-cache 关闭，当前启动代码未开启它；若改缓存策略需另行实现缓存一致性。

## 上电测试

1. 架空底盘，保持静止至少2秒。默认 `calibrated=false`，不允许启动运动。
2. 调试器观察 `Chassis_GetState()` 对应的静态 `state`：`dt` 约0.005秒，`updates` 每秒约增加200，`deadline_misses` 不持续增长，`fault=0`，`bias_ready=true`。核对实际任务优先级为AboveNormal。计时应在持续运行时观察，调试暂停会改变dt。
3. 观察 JY60 驱动的 `s_jy60`：yaw/gyro 随动作变化，静止时 gyro 接近零；trust 2=GOOD、1=DEGRADED、0=LOST。断开传感器超过180ms必须 LOST；重新连接需收到新的角度和角速度，并通过连续5个合格角度帧恢复检查；freshness表示新鲜度，confidence表示数据质量，trust综合两者。
4. UART5 的 PC12（TX）接 USB-TTL 的 RX，并共地，115200、8N1；VOFA+ 选 **JustFloat**，20Hz，14通道：vx目标、vy目标、wz目标、vx命令里程计、vy命令里程计、wz命令里程计、yaw度、yaw误差度、FL/FR/RL/RR发送RPM、trust、dt秒。PD2 为 UART5 RX，但当前尚未实现上位机串口指令接收，运动命令仍使用下述调试器邮箱。

## 启动短距离运动

`chassis_config` 定义于 `App/chassis_control.c`。轮半径按70mm直径设为35mm；半轮距、半轴距默认0，必须实测填写正值。先测量后修改，逐轮核对ID和 `motor_sign`（+1/-1），验证电机已使能、通信协议和正反向，再设置 `calibrated=true`。首次 RPM 上限建议20。

在调试器 Watch 中使用 `chassis_debug`，**先填字段，最后递增 sequence**；恢复运行后检查 acknowledged 等于 sequence、result=0。不要在电机运动时长时间暂停CPU，暂停后上位机无法发送停止命令。

1. `command=2`，递增 sequence：解锁，失败返回-1；要求标定确认、IMU GOOD、偏置已校准、fault=0。
2. `command=3,x=100,y=0,vmax=50,amax=100,dmax=100`，递增 sequence：车体系前进100mm规划。
3. `x=-100,y=0` 测后退；`x=0,y=100` 测左移；`x=0,y=-100` 测右移。每次等待完成再发下一次。
4. `command=1`，递增 sequence：停止并解除解锁。下次运动需重新解锁。
5. 原地旋转测试：解锁后设 `command=4,x=0,y=0,vmax=0.2,amax=1`，递增 sequence；以0.2rad/s旋转约1秒。负值反转。该命令的 x/y 单位为mm/s，vmax为rad/s，amax为持续秒数（最多2秒）；仅用于低速调试，启动/结束不经过距离规划器。

所有控制API应从底盘任务调用；其他任务和调试器通过邮箱提交。邮箱一次仅一条未确认命令，不能有多个并发写入者。

## 标定及验收

确认四轮方向后落地低速测试，再测前进/横移1000mm的实际距离。横向命令增益为 `left_gain/right_gain`，横向里程计比例为 `left_odom_scale/right_odom_scale`，分别标定，不能靠固定RPM偏置代替。调节航向 kp/ki/gyro_damping，逐步增加速度与加减速度。

故障位：1=控制周期超过50ms，2=运动中IMU丢失，4=电机TX错误或超时，8=参数无效。故障解除后复位重新校准。通信故障、CPU暂停或掉电时无法保证软件停止帧送达，应使用驱动器超时停机或外部断使能。

规划仅使用rpm_applied的分段投影积分决定减速和结束，可补偿限幅造成的命令进度落后。
rpm_requested是限幅后的浮点轮速；rpm_pending是量化后待发送轮速；rpm_inflight在发送批次内保持不变。
只有完整多机帧或legacy最终FF66发送完成，才将rpm_inflight发布为rpm_applied。
打滑、通信延迟及驱动器未执行命令仍会造成实际距离误差；不构成真实位置闭环。当前说明见 ARCHITECTURE_FIX_V04.md。
