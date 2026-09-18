# ZHY 基线迁移与底盘扩展

基线：2026ROBOCUP 的 ZHY 提交 058994a。当前需求优先于旧 PATH 迁移文档。

## 保留的 ZHY 行为

- RDK 视觉算法、标定、AWB 与动作组保持 ZHY 实测值；bridge 已扩展为并发 indexed RFID gate。
- host_command.c/h 与 chassis_control.c 保持 ZHY 行为；App/rdk_link.c/h 仅扩展圆盘中间协议与取消辅助帧。
- 原有 PING 门禁、RDK_RESET、DISC 原地测试、PATH 二次握手和原有完整遥测通道保留。CH29/accepted_ids 仍是独立 PING 通过标志，不是 RFID 数量。
- PATH 圆盘前路线以 ZHY 为基线，当前距离经现场调整：左前位移 (1691.4467, 615.6363) mm → 前进 2200 mm → 转 180° → 双中间灰度对齐。
- DISC_START 只发送一次；RDK 执行 G101 一次、G102 五次。每次 G102 完成后按序发送 `DISC_ACTION_DONE N`，STM32 读到本 gate 的新 UID 后回复 `DISC_RFID_OK N`；第五次 RFID 确认后 RDK 才返回 `DISC_DONE`。

## STM32 新增内容

1. `DISC_TASK_TIMEOUT_MS=60000` 从圆盘命令开始计时，包含 RDK 相机/G101、五次 G102 与五次 RFID gate；任何中间消息都不重置总超时。到时停车、锁定，不继续后续路线。
2. UART7 (PE7 RX/PE8 TX，9600、8N1) 接收厂家数据帧，校验长度、地址 0x20、状态和校验和后提取完整四字节 UID；128 字节有序接收环，溢出只记录 rfid_fault 位 64，不覆盖已有数据，也不停车。RFID 初始化和接收错误均单独诊断，不阻止握手或底盘任务。
3. RFID UART7 接收中断始终运行，但只有收到当前 `DISC_ACTION_DONE N` 后才打开 capture gate。gate 打开时清 ring 与 partial frame；第一个新 distinct UID 使计数增长后立即关闭 capture 并发送 `DISC_RFID_OK N`。重复 UID、gate 关闭期间的卡和同一 ring 中后续帧都不能预授权下一次动作。视觉在等待 RFID 时持续 capture/detect/trigger，只抑制 G102。
4. PathPorts_CopyIds(uint32_t *out, size_t capacity) 读取列表，容量和返回值单位为 UID 个数；UID 按线上字节顺序表示为大端整数，例如 45 96 B7 8A 保存为 0x4596B78A。path_diagnostics.rfid_count 查看数量，旧 ids 位图不再使用，rfid_fault 查看独立采集诊断。列表保留到下一次被接受的 PATH/DISC；STOP、错误和 RDK_RESET 不清除，MCU 掉电/复位不保留。读取顺序不能直接视为已验证的仓位对应关系。
5. App/path_chassis.c 单独承接后续底盘：保持圆盘结束时朝向 → 直行（x=-1710 mm） → 红外靠桩/绕行 352° → 前进 330 mm → 台阶各段移动 → 左移 1650 mm → 仓库区域底盘移动。绕桩保留 5 秒靠桩、30 ms 红外稳定和 15 秒绕行保护；台阶保留对齐、-18 mm 起步、各点 -90 mm、段间 -117 mm。仓库区域只执行转 180°、对齐、左移 50 mm、后退 200 mm 三次、前进 200 mm 两次。
6. 后续底盘模块不发视觉、机械臂、转仓指令，也不要求绕桩/台阶 RFID 数量。PD10 输入为此扩展加入；ZHY 原有 PD0/PD1 灰度读取保持。
7. DISC 原地测试完成后仍留在原地；只有 PATH 接续后续路线。

## 测试与产物

- CTest 覆盖 indexed link、capture gate、ID 顺序/去重、缺少 ID、总超时、迟到完成、STOP/ERROR 和原地 DISC。
- ZHY 原版 Python 测试：RDK/tests 27 项 + rdk_vision/tests 28 项通过。
- 旧部署入口 tools/rdk/path_bridge.py 仅委托 ZHY 原版桥接，入口测试通过；不再部署旧 Q/R 服务。
- Release 产物：build/ZHY-Release/chassis_motor.elf、chassis_motor.hex、chassis_motor.bin。
- 未烧录、未连接实物验收。

复现（工程根目录）：

    cmake -S tests -B build/zhy-tests -G Ninja -DCMAKE_BUILD_TYPE=Debug
    cmake --build build/zhy-tests
    ctest --test-dir build/zhy-tests --output-on-failure
    py -3.13 -m unittest discover -s RDK/tests
    py -3.13 -m unittest discover -s tests -p test_path_bridge.py

视觉包测试需先进入 RDK 目录，再执行：

    py -3.13 -m unittest discover -s rdk_vision/tests -t .

RDK 实机继续使用原版 tools/rdk_stm32_bridge.py 和既有服务/配置；迁移不意味着需要覆盖队友已验证的现场标定。

## VOFA 的 RFID 数量通道

完整模式追加 CH35（从 0 开始，第 36 通道）：path_diagnostics.rfid_count，已读取的不同标签 UID 数量，范围 0..64。重复 ID 不增加数量；启动下一次 PATH/DISC 时清零。原 CH0..34 不变，CH29 仍是 PING 通过标志。

完整 JustFloat 帧现在为 36 个 float + 4 字节帧尾，共 148 字节。精简模式追加 CH30（第 31 通道），共 128 字节。该通道仅用于观察，不参与运动控制。

## RFID 读写器设置与 UID 含义

读写器须使用厂家工具设为“自动读卡号、主动上传”，地址 0x20，9600 8N1。
当前 STM32 被动接收，不主动发送轮询命令。仅设置波特率并不能保证读写器主动上传。
支持自动卡号帧（04 0C 02）、卡号加块数据帧（04 1C 04）以及读卡号响应（01 0C A1）；只含块数据的帧不会作为 UID 入库。
读写器 TX 接 PE7，RX 接 PE8，共地；接口电平须与板卡匹配。USART6 PC6/PC7 保留给步进电机。

UID 是标签唯一标识，并不是标签数据块里写入的球编号；编号不限于 1～9。
当前按用户确认的临时方案保存 UID，后续确认标签数据布局或建立 UID 与球编号映射后再使用。
完整 UID 不通过 float 遥测，避免精度丢失；VOFA 仅回传不同 UID 的数量。
