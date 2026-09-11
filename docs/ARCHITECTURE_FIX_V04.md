# V0.4：200Hz调度与X42S多机帧

## 协议依据

已核对用户提供的《ZDT_X42S第二代闭环步进电机用户手册V1.0.4_260401.pdf》第37、49、50、53页。
第50页示例的0022是34字节整帧长度，不是子命令长度。
四个8字节F6加4字节帧头和1字节末尾校验，共37字节：

```text
00 AA 00 25
[ID F6 DIR RPM_H RPM_L ACC 00 6B] × 4
6B
```

内部校验保留；速度子命令同步位为00；无额外FF66。
固定校验0x6B、Emm固件，默认整数RPM（驱动器速度缩小10倍选项须关闭）。
921600、8N1理论线路时间约0.401ms；不是实测驱动器响应时间。

## 调度

freertos.c实际使用osPriorityAboveNormal和osDelayUntil；周期由内核tick频率除以200计算。
不支持精确5ms的内核tick配置在启动时拒绝。
超期不补跑历史周期，重建下一次唤醒点并计入state.deadline_misses。
DWT提供真实dt。软件配置200Hz仍需板上检查dt和deadline_misses，不能以编译通过代替实测。
.ioc ChassisTask优先级数值32，其他任务24；函数名后的Default是代码生成选项。

## 轮速生命周期

- rpm_requested：限幅后的浮点逻辑轮RPM。
- rpm_pending：量化后、尚未开始发送的最新逻辑轮RPM，可被覆盖。
- rpm_inflight：本次不可变发送批次对应的逻辑轮RPM。
- rpm_applied：完整多机帧（或legacy最终FF66）发送完成后生效的命令模型轮RPM。

发送完成后先用旧rpm_applied积分到完成时间，再替换新值。物理方向只在编码帧时应用。
applied不表示驱动器已应答或真实测得转速，里程计仍是开环命令模型。

默认chassis_config.command_mode = ZDT_MULTI_COMMAND。
需要fallback时，在解除解锁且发送空闲时切换为ZDT_LEGACY_SYNC，或修改默认配置后重启；
legacy保留四帧F6+FF66、至少3ms帧间隔。发送模式在批次开始时锁定。

## 测试

原有测试加multi_frame协议向量测试：37字节长度、负转速、四ID、内部同步位、
内部/外部校验、缓冲区边界、重复ID拒绝以及legacy同步位。
烧录后先架空低速测试并检查逻辑分析仪实际帧，确认UART5遥测dt约0.005秒。

本轮仅处理V0.4三项；未新增驱动器状态读取、心跳自动配置，未修改cache和creep策略。
后续更新：Planner制动改用applied_path_speed，详见APPLIED_SPEED_BRAKING.md；保留低速爬行策略。
V0.3记录中的“0xAA待手册确认”由本记录取代。尚未进行实机验证。
