# V0.6 前进补偿与 Flash 报告

日期：2026-09-11。软件构建与主机测试通过；实车尚未验证。基准 HEAD：4607538，原始工作区 git status 干净。

## 不变约束

保留现有参数：轮半径 35 mm、轮速上限 200 RPM、串口动作 vmax 450.519 mm/s、amax/dmax 550 mm/s²；PID、左右增益、里程计比例、电机映射/方向、IMU、Heading、ZDT 协议和 applied 链路不修改。

## 基准与计量口径

执行 Debug/Release 配置构建、arm-none-eabi-size、size -A、nm --print-size --size-sort --radix=d；保留 ELF/map/bin 于 build/v06-baseline/{Debug,Release,Release-LTO}/。

Flash used 采用链接器实际占用（与本工程 bin 长度一致），包含向量表、只读数据、初始化数据加载镜像及对齐；比 size 的 text+data 大 8 B。RAM used 为链接器占用，包含预留 heap/stack，不等于运行时高水位。

| 阶段 | text | data | bss（size） | Flash used | RAM used |
| --- | ---: | ---: | ---: | ---: | ---: |
| 原始 Debug -O0 -g3 | 55836 | 156 | 42016 | 56000 | 42176 |
| 原始 Release -Os -g0 | 34168 | 156 | 42024 | 34332 | 42184 |
| 原始 Release + LTO 实验 | 29004 | 152 | 42008 | 29164 | 42168 |

仅 Debug → Release 节省 21668 B。Release 已低于 48 KiB，无需为凑目标重写数学库或 CMSIS。

LTO 首次实验暴露汇编调用 vTaskSwitchContext 被优化移除的问题；实验链接使用 --undefined=vTaskSwitchContext 保留。LTO 默认 OFF，独立 Release-LTO 预设。编译通过不代表中断/调度实机验证通过。

## 原始 Release Flash Top 30

只统计 Flash 地址中的 t/T/r/R 符号，不把 ucHeap、Timer_Stack 等 RAM 符号算入 Flash。符号尺寸不包含所有填充和无名常量。

| 排名 | Symbol | 字节 |
| --- | --- | ---: |
| 1 | `Chassis_Update` | 2100 |
| 2 | `__kernel_rem_pio2f` | 1676 |
| 3 | `HAL_RCCEx_PeriphCLKConfig` | 1304 |
| 4 | `HAL_UART_MspInit` | 932 |
| 5 | `HAL_RCC_OscConfig` | 884 |
| 6 | `JY60_Process` | 820 |
| 7 | `two_over_pi` | 792 |
| 8 | `__udivmoddi4` | 760 |
| 9 | `HAL_UART_IRQHandler` | 660 |
| 10 | `__ieee754_rem_pio2f` | 620 |
| 11 | `service_tx` | 540 |
| 12 | `Planner_UpdateProgress` | 516 |
| 13 | `UART_SetConfig` | 508 |
| 14 | `prvTimerTask` | 508 |
| 15 | `config_valid` | 472 |
| 16 | `HAL_GPIO_Init` | 456 |
| 17 | `send_telemetry` | 448 |
| 18 | `HostCommand_Feed` | 436 |
| 19 | `__ieee754_fmodf` | 388 |
| 20 | `HAL_DMA_IRQHandler` | 380 |
| 21 | `xQueueGenericSend` | 372 |
| 22 | `xQueueReceive` | 330 |
| 23 | `HAL_TIM_IRQHandler` | 320 |
| 24 | `HAL_RCC_ClockConfig` | 316 |
| 25 | `xTaskIncrementTick` | 308 |
| 26 | `pvPortMalloc` | 296 |
| 27 | `__ieee754_hypotf` | 284 |
| 28 | `Planner_Start` | 276 |
| 29 | `__kernel_cosf` | 260 |
| 30 | `prvAddNewTaskToReadyList` | 252 |

## 原始 Release 按模块归类

按 map 的已分配输入节地址统计，排除 discarded sections。重叠合并字符串只计一次；跨模块重叠列为 Shared/merged。Telemetry 的 send_telemetry 单独计，其余状态/内联可能在 Chassis/Host 内。

| 模块 | 字节 |
| --- | ---: |
| HAL | 8875 |
| FreeRTOS | 6691 |
| Chassis/Host | 6336 |
| libm | 5409 |
| Core/startup | 3377 |
| IMU | 1282 |
| libgcc/startup | 812 |
| CMSIS-RTOS2 | 494 |
| Telemetry | 448 |
| libc | 288 |
| ZDT | 224 |
| Shared/merged | 31 |
| 对齐/未归属 | 65 |
| 合计 | 34332 |

libm 5409 B 含 cosf/sinf 共用的范围归约与常量；不能把整个 libm 当成 cosf 独占成本。Command Odom 同样使用三角函数，本轮保留原数学实现。无 printf/vfprintf/sprintf/sscanf/strtod/_printf_float/_scanf_float 引入；HAL 源列表保持不变，不把已被 gc-sections 丢弃的模块算成优化收益。

## 进度

- 已完成：基准、模块/符号分析、前馈、10项主机测试、五种固件构建、空任务与registry裁剪、测试手册。
- 待用户实车：K=0 回归、补偿标定、Release/LTO 时序与安全回归；没有烧录、驱动车辆或宣称实车通过。

## 最终尺寸

| 阶段/变体 | text | data | bss（size） | Flash used | RAM used |
| --- | ---: | ---: | ---: | ---: | ---: |
| 最终 Debug，完整 telemetry | 55828 | 160 | 41928 | 55996 | 42088 |
| 最终 Release，完整 telemetry（默认） | 33948 | 160 | 41960 | **34116** | 42120 |
| 最终 Release + LTO（实验） | 28596 | 156 | 41944 | 28760 | 42104 |
| 最终 Release，精简 telemetry | 33900 | 160 | 41928 | 34068 | 42088 |
| 最终 Release，关闭 telemetry TX | 33112 | 160 | 41824 | 33280 | 41984 |

默认 Release：33.32 KiB，剩余 **65536 - 34116 = 31420 B**。相比原始 Debug 节省 21884 B；相比原始 Release 净省 216 B（已包含新增功能）。LTO 对最终 Release 另省 5356 B，但默认关闭。

开发过程的可复查快照（build/v06-baseline）：ForwardComp 为 34440 B，EmptyTasks 为 34140 B，registry 关闭时为 34084 B；这些期间有补偿 signed-zero/telemetry整理，不能把阶段差值全部归因为单一 RTOS 开关。以最终固件表为准。数学优化未执行，收益记 0；不是遗漏。

## FreeRTOS 审计与取舍

应用只创建 ChassisTask，使用 osKernelGetTickFreq/GetTickCount、osDelayUntil；UART命令已在 Chassis_Update 内处理。原 defaultTask 和 UartCommandTask 都只 osDelay(1)，没有业务。

- 删除 defaultTask 的创建和空实现；UartCommandTask 以 ENABLE_UART_COMMAND_TASK 默认 0 编译关闭。200 Hz ChassisTask 主体未改。
- CubeMX 的 .ioc 任务配置尚未同步；重新生成工程可能恢复空任务和registry配置，生成后必须复核 freertos.c、FreeRTOSConfig.h 及此报告，不能直接覆盖后烧录。
- 单独将 configQUEUE_REGISTRY_SIZE 从 8 改为 0。CMSIS 对 registry 访问有预处理保护，编译成功；不再保留队列名称注册调试信息。
- 没有应用层 timer/mutex/semaphore/event/stream/message buffer 调用，但当前 cmsis_os2.c 的 timer、mutex、trace 等包装函数仍无条件引用对应 API（例如 osTimerNew 约890行、uxTaskGetSystemState 约667行）。因此本轮保留 timers/trace/mutex/INCLUDE_*，不靠制造隐式声明警告或重写 wrapper 裁剪它们。
- 保留 timers.c、Timer Task、HAL源列表及所有安全检查。32 KiB RTOS heap 不缩减，空任务省的是动态堆分配，不应把其 4096 B 栈预算直接从 ELF bss 中扣除。
- 本轮未逐阶段 Git commit；各阶段用构建快照与报告留证，所有修改留在当前工作区供审阅。

## 前馈与验证

ChassisConfig 仅新增 forward_lateral_comp，默认0。正值注入 +Y；根据原始路径方向 dy==0（jog依据jog_y）判断，避免斜向低速阶段被1 mm/s阈值误判为纯前进。

原左右增益公式完整保留。零补偿直接保留 vy_original 位表示，包括负零。测试使用真实 Mecanum/ZDT 验证零补偿 F6/AA 帧，不调整协议。

10/10 CTest：relative_yaw、host_uart、host_command、dwt_init、test_progress、test_health、motion、control_math、multi_frame、forward_comp。新测试覆盖K=0、正K、后退、左右增益、原地旋转、低速斜向、signed-zero、非法非有限K。signed-zero 用例曾失败，修复后通过。

五种构建均成功：Debug、Release、Release-LTO、Release-Slim、Release-NoTelemetry；实际 ELF/bin 均已测量。主机测试不等于实机 200 Hz 调度或物理轨迹验收。

最终 map/nm 复查：Chassis_Update 2204 B、send_telemetry 460 B、config_valid 492 B；数学大户 __kernel_rem_pio2f 1676 B、two_over_pi 792 B 仍保留。HAL/I2C未盲删，未引入浮点printf。

编译、VOFA通道和逐步实车回归见 [TESTING.md 的 V0.6 入口](TESTING.md)。

## 最终 Release Flash Top 30

| 排名 | Symbol | 字节 |
| --- | --- | ---: |
| 1 | `Chassis_Update` | 2204 |
| 2 | `__kernel_rem_pio2f` | 1676 |
| 3 | `HAL_RCCEx_PeriphCLKConfig` | 1304 |
| 4 | `HAL_UART_MspInit` | 932 |
| 5 | `HAL_RCC_OscConfig` | 884 |
| 6 | `JY60_Process` | 820 |
| 7 | `two_over_pi` | 792 |
| 8 | `__udivmoddi4` | 760 |
| 9 | `HAL_UART_IRQHandler` | 660 |
| 10 | `__ieee754_rem_pio2f` | 620 |
| 11 | `service_tx` | 540 |
| 12 | `Planner_UpdateProgress` | 516 |
| 13 | `UART_SetConfig` | 508 |
| 14 | `prvTimerTask` | 508 |
| 15 | `config_valid` | 492 |
| 16 | `send_telemetry` | 460 |
| 17 | `HAL_GPIO_Init` | 456 |
| 18 | `HostCommand_Feed` | 436 |
| 19 | `__ieee754_fmodf` | 388 |
| 20 | `HAL_DMA_IRQHandler` | 380 |
| 21 | `xQueueGenericSend` | 372 |
| 22 | `xQueueReceive` | 330 |
| 23 | `HAL_TIM_IRQHandler` | 320 |
| 24 | `HAL_RCC_ClockConfig` | 316 |
| 25 | `xTaskIncrementTick` | 308 |
| 26 | `pvPortMalloc` | 296 |
| 27 | `__ieee754_hypotf` | 284 |
| 28 | `Planner_Start` | 276 |
| 29 | `__kernel_cosf` | 260 |
| 30 | `prvAddNewTaskToReadyList` | 252 |
