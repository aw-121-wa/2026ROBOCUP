# 02-red-disc-direct

第二版：仅红方圆盘结束后后退1400mm，校准到仓库地图0度，向车身左侧找线，接仓库入口；蓝方路线不变。

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

Flash 64576/65536 bytes；RAM44088 bytes。

Windows PowerShell进入此文件夹：
```powershell
cmake --preset Route-Red-Direct
cmake --build --preset Route-Red-Direct -j 8
openocd -f interface/cmsis-dap.cfg -f target/stm32f7x.cfg -c "adapter speed 1000; program build/Route-Red-Direct/chassis_motor.elf verify reset exit"
```
无需重新编译，烧录包内预编译固件：
```powershell
openocd -f interface/cmsis-dap.cfg -f target/stm32f7x.cfg -c "adapter speed 1000; program firmware/chassis_motor.elf verify reset exit"
```
编译要求CMake、Ninja及arm-none-eabi工具链在PATH。两个包的RDK相同。
默认版本开关App/path_config.h及CMake选项PATH_RED_DISC_TO_WAREHOUSE：0现有路线，1红方直达。编译预设优先。
测试入口tests/test_two_pass.c、tests/test_import_route.c、tests/test_rdk_link.c；RDK/tests/test_two_pass_protocol.py、test_warehouse_digits.py、test_rdk_stm32_bridge.py。其他历史测试未作为本次流程验证依据。
