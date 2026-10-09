# 绕桩与阶梯相机启动重试

RDK 的共享球相机在打开、AWB 预热或首帧获取失败后，释放失败实例，间隔 1 秒重试。
重试不重新运行 G103；机械臂准备失败仍正常报错。取消信号可以退出重试。
获得新鲜帧且准备动作完成后才发送现有 PILLAR_READY，STM32 随后开始绕桩/阶梯扫描。

新增 PILLAR_CAMERA_WAIT 仅在绕桩/阶梯启动期间续期双方等待期限，不放行运动、不延长抓球或运动超时。
没有新通知时原有超时仍生效。底层 USB 驱动调用若永久阻塞，Python 无法保证强制中断。

必须配套更新 STM32 固件和 RDK 文件：
- tools/shared_task_camera.py
- tools/vision_servo_direct_test.py
- tools/rdk_stm32_bridge.py
- tools/stair_scan.py

旧 STM32 不认识新等待通知，会报协议错误，不能单独更新 RDK。
当前不改变圆盘相机和仓库数字识别的重试策略。

## 圆盘相机启动重试

圆盘与绕桩共用可重试相机租约。圆盘 G101 只执行一次；相机打开、预热或新帧检查失败后重新打开，每秒重试，直到成功或任务取消。启动期间发送 DISC_CAMERA_WAIT，STM32只在圆盘启动阶段续期等待；新帧就绪后发送 DISC_CAMERA_READY，恢复正常圆盘任务计时。颜色仍由 COLOR RED/BLUE 选择。
