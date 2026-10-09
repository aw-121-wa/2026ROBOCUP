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
