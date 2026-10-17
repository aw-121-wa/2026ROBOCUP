# 两份代码

01-current-route.zip：保留现有路线。

02-red-disc-direct.zip：红方圆盘架起完成后后退1400mm，仓库0度，向左找线。蓝方路线不变。

每包均含完整源码、RDK视觉程序及firmware目录的ELF/HEX/BIN。RDK-common.tar.gz是两版共用视觉。

两版共用仓库流程：入口后退至中间两灯一亮一灭后，等待底部相机READY并向车身前方扫描。
前灯PD0先亮后灭：停止旧识别并等待STOPPED，转为后退扫描；后灯PD1先亮后灭：结束第二趟。
识别1/2/3后，以识别时位置为基准向车身前方20mm（含刹车位移补偿），停车并放对应列的球。
已经放完的数字不重复处理；两趟后仍有球则以返程后灯灭位置为入口基准，按物理1/2/3放剩余球。
物理回退位置：红方215/415/615mm，蓝方200/400/600mm。列间距200mm。
扫描无时间超时、无距离上限，依靠灰度亮到灭结束；初始已灭不会立即触发掉头。转盘和放球沿用现有动作。
扫描速度参数 App/path_config.h: PATH_WAREHOUSE_CREEP_SPEED_RPM=16。
两版均等待圆盘DISC_DONE及架起G1完成回包后才离场。红方回家直行1885mm保持。
RDK必须同步更新包内RDK目录：新增WAREHOUSE_STOP/WAREHOUSE_STOPPED握手，warehouse_number.yaml timeout_s=0。
非仓库扫描的路线超时以及电机/IMU故障保护保持原设置；未修改数字模型阈值。
未进行实车烧录、部署或场地验证。Windows Python313下36项相关视觉测试通过；两版C状态机及协议回归通过。
