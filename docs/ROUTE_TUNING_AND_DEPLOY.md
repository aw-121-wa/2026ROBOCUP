# 路线调参与部署

## 修改参数

只编辑 `App/route_tuning.h`，按开局、绕桩、阶梯、仓库、返程分组。
距离单位 mm；正负号表示原路线指令方向，缩短距离时保持符号。
阶梯触发位置为后退完成后的累计距离。仓库间距187 mm，对应中心187/374/561/748 mm。

速度为直接数值，RPM表示含原倍率的名义目标转速；MM_S表示mm/s。无需修改倍率。
`route_speed_adapter.h` 是内部编译期换算，不是调参文件。电机限幅、加减速和航向保持会使实际速度低于目标；圆弧切入/切出的边界速度独立于巡航上限。
绕桩转速参数是轮速等效指令，不是车体每分钟旋转圈数。蓝方开局巡航和边界速度有独立数值配置。
标记“旧流程”的参数用于关闭连续倒垛时的备用路径；当前后退由PD1熄灭控制，不由固定距离控制。
航向PID、硬件轮径、灰度引脚和视觉ROI保持原文件。

## STM32 编译和烧录（Windows PowerShell）

```powershell
cd D:\stm32project\2026ROBOCUP
cmake -S . -B build/Release-Size-Audit -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release -DCHASSIS_RELEASE_LTO=ON -DCMAKE_C_FLAGS=-fno-inline-functions-called-once
if ($LASTEXITCODE -ne 0) { throw "CMake configure failed" }
cmake --build build/Release-Size-Audit
if ($LASTEXITCODE -ne 0) { throw "Build failed; do not flash stale firmware" }
openocd -f interface/cmsis-dap.cfg -f target/stm32f7x.cfg -c "gdb_port disabled; tcl_port disabled; telnet_port disabled; adapter speed 1000; program build/Release-Size-Audit/chassis_motor.elf verify reset exit"
if ($LASTEXITCODE -ne 0) { throw "Flash failed" }
```

使用CMSIS-DAP，烧录包含校验和复位。当前固件接近64KB容量上限，必须看到链接成功后再烧录。烧录不发送发车指令，但固件保留红外启动功能。

## RDK 仓库视觉部署（Windows PowerShell）

```powershell
cd D:\stm32project\2026ROBOCUP
powershell -ExecutionPolicy Bypass -File tools/deploy_rdk.ps1 -RdkAddress 10.205.175.232
```

IP变化时替换`-RdkAddress`。电脑与RDK应连接同一网络；脚本使用本机已有`build/rdk_deploy_key`，不要把私钥提交Git。
部署将打包仓库识别Python代码和方块模板，上传后备份、停止视觉服务、解压、校验SHA256、加载六组模板、重启`rdk-disc.service`。
此脚本部署仓库视觉模块及串口桥，不替换球视觉ROI配置、模型或整个系统。仅修改STM32路线参数时不需要重部署RDK。
看到文件校验成功、六组模板就绪以及`ActiveState=active`、`SubState=running`后再进行实跑。

## 倒垛后放球触发（2026-10-12）

倒垛完成后选择最近的未放球列方向，开启底部数字连续识别。估算列中心仅用于选择移动方向，不触发固定距离停车。收到新的有效数字1/2/3时刹停，停稳并关闭本次扫描后按列号放球；该列完成后继续移动识别下一列。已经完成的列号和过期帧不触发放球。数字漏识别时在前三列范围内往返扫描；相机数据不新鲜时等待新帧。严格0110横移恢复和机械臂操作停稳条件继续有效。

列间距继续用于倒垛固定位置和扫描范围；修改放球阶段移动速度用`ROUTE_WAREHOUSE_TRANSFER_RPM`。
