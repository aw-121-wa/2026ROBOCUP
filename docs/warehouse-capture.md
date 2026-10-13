# 仓库现场采集版（2026-10-13）

编译开关：PATH_WAREHOUSE_CAPTURE_START，默认 OFF。

## 编译
```powershell
cmake --preset Full-Vision
cmake --build --preset Full-Vision -j 8
cmake --preset Warehouse-Capture
cmake --build --preset Warehouse-Capture -j 8
```
两种固件分别在 build/Full-Vision 和 build/Warehouse-Capture。
完整视觉 Flash 63532/65536 字节；仓库采集 63200/65536 字节。
缩减来自将 RDK 协议通用 printf 替换为整数编码，不关闭视觉或遥测。

## 使用
将车摆在仓库入口找线前，朝向与正常进入仓库时一致。红方短暂遮挡红外后放开；蓝方持续超过5秒。
开机仍需正常握手、初始化和颜色同步。启动后 G3 准备，入口找线、0110 对齐、角度校准、后退至中间灰度一明一灭，然后进入原有倒垛及底部数字识别流程。结束停车，不回家。
蓝方在启动时以当前朝向作为仓库180度方向，避免从开局0度基准额外旋转。
注意：直启没有前序收球，库存为空时会识别底部列号、完成流程但跳过实际放球动作；不会虚构已装球的库存。

## 相机采集
需部署随包 RDK 代码；新版接受 WH_CAPTURE 指令。正常 WH_SCAN 不自动存图。
识别相机工作期间独立线程按约5张/秒保存全幅 JPG（质量95），行1/2/3为倒垛，行0为底部数字。机械臂动作、相机切换和识别暂停期间不保证有画面。
目录：/home/sunrise/licang_vision/captures/warehouse_field/<时间戳>-<red或blue>-row<行号>/
frames.jsonl 记录帧号、相机单调时间、存盘时间、颜色和行号。journalctl -u rdk-disc.service 中 WH_CAPTURE_STATUS 记录识别耗时和图像年龄。
少于512MiB空间或写入失败会停止本段存图并输出 WAREHOUSE_CAPTURE_ERROR，不删除旧数据。实际采集帧率取决于磁盘速度，不堆积无界队列。

## 卡顿排查
仓库扫描、去第四列、返回及底部数字移动中，灰度偏离0110不再先刹停，保留前后速度并叠加5 rpm横移修正。1001/1111视为太近：红方向车身右、蓝方向车身左。恢复0110立即取消横移速度指令。
0000以及未标定的其他读数不能确定方向，采用600ms远离、1200ms靠近、600ms远离的有限试探周期。仅连续4秒未恢复0110时停车报错，不设置25mm横向偏移停车条件。不会保证任意丢线都能恢复，需要现场验证。
入口后退/起算动作不变；相机居中、夹取/放置及转盘动作前仍需停稳对齐。视觉回包超过250ms未更新仍会暂停扫描，因此不能保证完全消除所有卡顿。
参数在 App/route_tuning.h 的 ROUTE_WAREHOUSE_DISTANCE_*。本次相关4项仓库回归测试通过，尚未实跑。

## 验证
98项主机测试94项通过。4项旧失败（path_mission、blue_pillar、blue_stair_pause、stair_continuous）在修改前HEAD基线同样复现；不计为通过。
新增红蓝直启、等待G3完成、结束停车、WH_CAPTURE报文测试通过；整数报文最大token与回绕测试通过。
Python语法检查与使用模拟写入器的采集线程去重、元数据及停止测试通过。
RDK连接超时，本次新增采集功能尚未实机验证，未烧录、未部署、未发车。
