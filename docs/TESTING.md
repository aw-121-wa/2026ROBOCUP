# RoboChassis V2 测试

## V0.6 当前测试入口（优先于下方历史记录）

正式候选固件使用普通 Release；Debug 保留 -O0 -g3，不启用 LTO。

```powershell
cmake --preset Release
cmake --build --preset Release
```

产物：`build/Release/chassis_motor.hex`。本轮没有自动烧录或驱动车辆。
不要误烧旧的 `build/Debug` 文件。尺寸对比见 [V0.6 报告](ARCHITECTURE_FIX_V06.md)。

当前保留用户实调参数：半径 35 mm、上限 200 RPM、串口动作速度 450.519 mm/s、加减速度 550 mm/s²。没有调整 Heading、IMU、电机方向或里程计比例。

### 纯前进右漂补偿

只修改 `App/chassis_control.c` 中 `.forward_lateral_comp = 0.0f`。
这是无量纲前馈系数 K：`vy_comp = K * vx`，正值向左抵消右漂。
默认 0 关闭。只对路径方向 y=0 且 vx>0 生效；斜向（即使低速时 vy 很小）、后退、左右横移、原地旋转均不启用。调试 jog 同样依据原始 jog_y 判断。

原左右横移增益先计算，再叠加前进补偿，不修改 odom scale。它补偿的是固定横向漂移，不保证消除真实航向误差或轮胎打滑。

1. K 保持 0，架空确认四轮方向和 STOP，再在清空的场地依次测试短距离前进、后退、左右横移。先不要自动循环发送。
2. 复位后静置等待 bias_ready=1、trust=2、fault=0，ASCII 发送 `ARM` 加换行，再发送 `FORWARD 1000` 加换行。确认空间足够并准备物理断电。
3. 测量实际前进长度 L 和向右偏移 D，记录车头角度。距离尚不准时先排查，不能直接用名义 1000 代替 L。
4. 理论 K≈D/L，第一轮只取约 70%。例如 L=1000、D=30 mm，先设约 0.02，而不是直接 0.03。
5. 重新编译烧录、复位静置，再测相同路线；出现左漂则减小 K。每次只改 K，不同时改 PID/速度/里程计。
6. 回归 `FORWARD -1000`、`SHIFT 1000`、`SHIFT -1000`；斜向/原地旋转通过现有调试入口在安全条件下验证。对这些动作补偿通道必须为 0。

VOFA 继续使用 UART5 115200、JustFloat，默认 **27 通道、112 字节/帧**，原 0..24 编号不变：

| 通道（从 0 开始） | 数据 |
| --- | --- |
| 0 | vx_cmd，mm/s |
| 1 | vy_final，叠加补偿后的命令，mm/s |
| 7 | yaw_error，度 |
| 25 | forward_comp_vy，主动横向前馈，mm/s |
| 26 | vy_original，原左右增益处理后、前馈叠加前的 vy，mm/s |

这些是轮速限幅前的命令；8..11 为已发送应用的模型 RPM，不是编码器实测转速。轮速饱和时实际应用补偿会减少。

### Telemetry 裁剪与 LTO 实验

开发默认完整遥测 ON，不使用 printf。可在独立目录测试裁剪，避免改变普通 Release 缓存：

```powershell
cmake --preset Release -B build/Release-Slim -DCHASSIS_TELEMETRY_FULL=OFF
cmake --build build/Release-Slim
cmake --preset Release -B build/Release-NoTelemetry -DCHASSIS_TELEMETRY_ENABLE=OFF
cmake --build build/Release-NoTelemetry
```

FULL=OFF：22 通道、92 字节；0..19 不变，20=forward_comp_vy，21=vy_original，不再发送原 IMU 诊断。ENABLE=OFF 仅停止 VOFA TX，UART5 命令 RX/STOP 仍启用。

```powershell
cmake --preset Release-LTO
cmake --build --preset Release-LTO
```

LTO 是未实车验收的独立实验，不是推荐首次烧录版本。先验证普通 Release：启动遥测、ARM/STOP、IMU 丢失停车、四方向/旋转、稳定 5 ms 周期及任务栈余量；之后才做相同实机 LTO 对照。主机测试不覆盖实际调度与串口中断时序。

## 电机速度单位修正（当前配置）

已确认驱动器F6速度单位为0.1 RPM：所有ZDT接口仍接收真实RPM，
仅在线路编码时乘10（100 RPM编码为1000，即03 E8）。
多机AA及legacy同步帧共用该编码；方向、同步位、长度及校验规则不变。
多机接口保留±3000 RPM输入检查；legacy接口超过此范围饱和，防止乘10溢出。
rpm_applied、轮速遥测、rpm_limit及里程积分均保持真实RPM，不能再次乘10。

历史说明：单位修正时曾设置半径35mm、上限100RPM、串口目标366.519mm/s、加减速度100mm/s²；之后用户已实调为上方V0.6当前参数，以下不再作为当前速度配置依据。
重新编译烧录后先架空确认方向，再低速发送FORWARD 100并测量实际距离。
最高匀速段约100RPM（受航向修正限幅影响），短距离不一定达到最高速度。
不要沿用旧配置下480对应5m的经验比例；单位统一后重新做距离标定。

## 相对角度累加试验

当前通道6及航向控制使用通过健康检查的IMU角度帧增量：
`delta = wrap(yaw_now - yaw_previous)`，`yaw = wrap(yaw + delta)`。
首次IMU GOOD时建立零点；同一帧不重复累加，跨±180°取最短角差。
暂时停用原来的gz积分和原始yaw拉回融合。gz仍用于健康检查、偏置校准，
以及配置非零gyro_damping时的角速度阻尼；本次不改PID或几何参数。

IMU LOST会取消解锁、清除相对零点，恢复GOOD后重建零点；若运动中丢失，
原故障仍锁存，必须复位。STOP和新距离命令不会重置相对零点，
但ARM/距离命令仍以当前相对航向作为保持目标。
本算法不能消除原始yaw漂移，连续数据下等价于当前角度相对初始角度。

先断开电机动力，静止等待通道12=2，然后手动转动、回零：
比较通道6（相对角度）、20（过滤前原始yaw）、23/24（原始帧计数）。
原始计数增长不保证角度被健康检查接受。
当前用户配置轮半径0.35mm、轮速上限1000RPM未被本次修改；
在确认真实几何和安全限速前不要进行落地闭环测试。

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

1. 架空底盘，保持静止约3秒。当前配置已设 `calibrated=true`，仍须人工确认几何尺寸、轮序和正反向。上电不会自动解锁。
2. 调试器观察 `Chassis_GetState()` 对应的静态 `state`：`dt` 约0.005秒，`updates` 每秒约增加200，`deadline_misses` 不持续增长，`fault=0`，`bias_ready=true`。核对实际任务优先级为AboveNormal。计时应在持续运行时观察，调试暂停会改变dt。
3. 观察 JY60 驱动的 `s_jy60`：yaw/gyro 随动作变化，静止时 gyro 接近零；trust 2=GOOD、1=DEGRADED、0=LOST。断开传感器超过180ms必须 LOST；重新连接需收到新的角度和角速度，并通过连续5个合格角度帧恢复检查；freshness表示新鲜度，confidence表示数据质量，trust综合两者。
4. UART5 的 PC12（TX）接 USB-TTL 的 RX，PD2（RX）接 USB-TTL 的 TX，并共地，使用3.3V TTL、115200、8N1、无流控；VOFA+ 选 **JustFloat**，约20Hz。现为20通道、84字节一帧：原0～13通道不变（vx/vy/wz目标、vx/vy/wz命令里程计、yaw度、yaw误差度、FL/FR/RL/RR发送RPM、trust、dt秒），新增通道见下节。

## UART5 文本运动指令（推荐）

发送区使用文本/ASCII，而不是HEX；每条以实际 LF、CRLF 或 CR 结束。
例如发送 `ARM` 并勾选追加换行；不要发送字面字符反斜杠+n。
命令区分大小写，距离单位为mm，支持正负小数，绝对值限制1～5000。
默认速度50mm/s，加速度和减速度均100mm/s²，本版不接受额外速度参数。

| 文本指令 | 动作 |
| --- | --- |
| `ARM` | 解锁；需要标定有效、偏置就绪、IMU GOOD、无故障 |
| `FORWARD 100` | 前进100mm |
| `FORWARD -100` | 后退100mm |
| `SHIFT 100` | 左移100mm |
| `SHIFT -100` | 右移100mm |
| `STOP` | 停止、取消解锁，并清除当前已缓冲的后续串口输入 |

先架空核对轮序、方向和驱动器使能。上电静止约3秒后发送ARM，
确认通道14=1、16=1，再单独发送FORWARD 100。
确认方向正确后落地低速测试；每次等通道18回到0再发下一条。
STOP后通道14应为0，下次运动需重新ARM。ARM后即使不平移也会进行航向保持，
可能出现轮子转动。STOP是软件停止请求，不是切断电机使能，不能替代物理急停。

| 新通道 | 含义 |
| --- | --- |
| 14 | armed：1已解锁，0未解锁 |
| 15 | fault：原故障位掩码 |
| 16 | 最近命令结果：0尚无结果，1已接受，-1格式错误，-2未解锁/解锁条件未满足，-3运动忙，-4距离超限，-5接收错误/溢出 |
| 17 | 已处理非空指令/接收错误计数，24位循环；确认它变化后再判断通道16 |
| 18 | 规划或点动忙：1运动中，0空闲（不是物理到位反馈） |
| 19 | bias_ready：1静止偏置校准完成 |
| 20 | IMU原始yaw，单位°：校验正确的角度帧，健康过滤前，未经主控航向融合 |
| 21 | IMU原始gz，单位°/s：未减去主控偏置 |
| 22 | 主控校准得到的gyro_bias，单位°/s；通道19=1后才表示校准完成，并非扣偏后的角速度 |
| 23 | 校验正确的原始角度帧计数，包含随后被健康检查拒绝的帧 |
| 24 | 角速度帧计数 |

当前遥测为25通道、104字节一帧，0～19编号不变。通道23/24发送计数低24位，
达到16777216后回绕，以避免JustFloat单精度丢失逐帧计数精度。
静止时角度数值可不变，但帧计数应持续增长。先不解锁，手动缓慢转动车头，
同时比较通道20与6（原始yaw和融合yaw）、21与22（原始gz和偏置）。
原始yaw是IMU内部算法输出，不是独立的真实角度基准。

不发送文本回包，以免破坏JustFloat；结果通过新增通道反馈。
运动中拒绝新的距离指令和ARM；静止且已解锁时重复ARM不会重置航向。
未解锁时拒绝距离指令。不要开启周期自动发送，也不要串口和调试器同时发命令。
计数与结果仅代表主控接受/拒绝，不代表电机执行或到位。

接收行最多63字节；无效/超长行丢弃到换行，半条命令间隔超过500ms也丢弃到换行。
接收错误或256字节环形缓存溢出会停止并取消解锁、清空缓存、恢复接收。
恢复后先单独发送一个空换行，再重新ARM。线路断开但没有UART错误时不会自动停止：
本版没有主机心跳，已接受的有限距离任务会继续；必须具备独立断使能手段。

## 启动短距离运动

`chassis_config` 定义于 `App/chassis_control.c`。当前轮半径35mm，半轮距128.5mm、半轴距130.5mm；必须核对实机尺寸。逐轮核对ID和 `motor_sign`（+1/-1），验证电机已使能、通信协议和正反向，再确认 `calibrated=true`。首次 RPM 上限建议20。

调试器邮箱仅作为备用接口：使用支持运行时内存写入的工具操作 `chassis_debug`，**先填字段，最后递增 sequence**；持续运行时检查 acknowledged 等于 sequence、result=0。不要暂停CPU来发运动指令，暂停后串口STOP无法执行。

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
