# 应用路径速度制动

Planner_UpdateProgress现在接收分段进度、应用路径速度、在途路径速度和响应延迟。
应用/在途速度分别从rpm_applied/rpm_inflight正解，按与分段进度相同的左右横移比例和方向投影计算。
不读取可能滞后的state.velocity，也不使用平移速度模长。

制动参考速度为max(0, applied_path_speed, committed_path_speed)。没有在途批次时，后者等于前者。
S = πv²/(4d)，延迟余量为v × max(dt, response_delay)。
多机模式预留10ms加两帧线路时间；legacy预留60ms。这是软件预算，实机响应仍需测量。
进入减速时记录该参考速度，从剩余距离中预留延迟段，再按半余弦距离曲线减速。
减速输出上限只降不升；应用速度因限幅进一步下降时同步收紧上限。
横移增益/里程计比例不能把减速输出重新放大。低速crawl逻辑保持原有上限5mm/s；
四轮整数RPM量化仍可能产生小幅路径速度波动，因此不是严格的物理加速度保证。

调试观察state.applied_path_speed、state.committed_path_speed、
planner.braking、planner.brake_speed、planner.brake_output和segment_progress。
VOFA通道数量及UART5配置保持不变。

测试涵盖：持续限幅后完成距离、700与1000mm/s制动时机差异、在途速度更高、
legacy延迟余量、负投影速度、限幅解除后直到停车无再次提速、横移比例及方向投影。
本次仅验证桌面命令模型与固件编译，未进行实车制动标定。
