# 仓库连续扫描与倒垛方案

## 范围

在当前版本上增加仓库连续倒垛流程，保留仓库之前的路线和当前返程。STM32负责运动、灰度与动作组，RDK负责列号和方块识别。适用于红蓝两侧，方向沿用各自仓库坐标转换。

## 灰度与航向

恢复四路有效低电平输入，位序为PD3、PD0、PD1、PB13。仓库入口及后续移动仅接受0110，不将1111或两路输入伪装成0110。

入口找到线后直接横移对准0110，取消旧入口后退至单灯和固定右移。移动时失去0110，暂停纵向行驶，保持陀螺仪航向并双向横移找回0110，然后从实际进度继续。相机采集保持运行，纠偏期间不接受夹取触发。机械臂操作期间底盘静止，完成后再恢复灰度检查。数字灰度无法保证每一瞬间都是0110，因此控制目标是持续检测并恢复到0110。

## 列号与连续扫描

持续前进识别三个物理列对应的仓库数字，保存列中心位置及映射；禁止边识别边放球。三个不同列号明确后到第三列中心。第三个数字允许由两个不同的1/2/3推断，但列中心仍须按位置确定。全部未知时不静默采用123，等待有效映射。

第三行从3→2→1扫描，第二行1→2→3，第一行3→2→1。识别姿态切换在端点停稳完成，扫描过程中不逐格停车。行首先检查当前画面，避免漏过端点方块。相机持续识别数字1/2/3和位置，数字稳定且进入图像中心容差窗口时通知STM32刹停；停车后再次确认居中，必要时小步纵向调整，再夹取。

## 搬运与放球

动作组第三行112/113/114，第二行115/116/117，第一行118/119/120，分别为识别/夹取/放置。来源行决定夹取动作；方块数字决定第四列目标行。第四列位于第三列外一个当前列间距。

等待夹取完成回包后移动到第四列，放置完成回包后返回中断位置，恢复原行原方向扫描。保存已处理的位置，避免重复识别；不沿用旧版“同一行只处理一个”的跳过规则。目标行已占用时停稳报告冲突，不覆盖。

三行扫描完成后回第三列，按物理3→2→1列顺序与已保存列号映射放球，复用现有球库存与转盘选择逻辑；全部完成后从最后放球列进入当前返程。

## 通信、恢复与验证

新增独立方块扫描会话，区分启动就绪、有效数字、居中、停止；避免旧回包在新行或纠偏结束后触发夹取。STM32保存扫描方向、行号、沿仓库轴的中断位置和处理记录。当前位置仍是命令积分估计，不宣称编码器闭环精度。

相机不就绪不开始扫描；断连或无新鲜图像时停稳等待恢复，不能把识别失败当空仓。空行通过走完扫描范围判定，取消逐格识别等待。机械臂异常保留现有故障停车。

测试覆盖四路位序、严格0110、双向纠偏与恢复、连续列号去重、行扫描往返、端点检测、陈旧回包拒绝、居中刹停复核、动作组映射、搬运返回和3→2→1放球。运行STM32主机测试、RDK识别/协议测试和Release/LTO构建，检查64KB Flash余量。此次实施不自动烧录或发车。


## Approved revision: destack before bottom-column recognition

This revision supersedes the earlier map-first and fixed 3-2-1 unloading sequence.
Warehouse and stair entry reverse until rear middle PD1 is off. Warehouse longitudinal origin is captured after braking; centers are 187, 374, 561 and 748 mm. After entry, strict 0110 lateral recovery remains active outside arm operations.
Use fixed positions for the row3/row2/row1 scans. Each row has one block; transporting it completes that row. When all rows finish, visit the nearest unserved ball column, stream its bottom digit during travel, and unload only with a known identity. Two unique column identities infer the remaining identity. Missing recognition holds at the column for a fresh result.
Column transfers are doubled to 70 rpm (initial approach 40 rpm). Task-external cruise and home speed increase 25%; acquisition, orbit, task scan, acceleration and deceleration settings remain independent.
