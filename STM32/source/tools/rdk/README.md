> 历史 Q/R 全流程文档，不适用于 2026-09-15 圆盘适配。当前请使用 docs/PING_FIRST_20260915.md；不要部署这里的旧 path_bridge.py。

# RDK 配套桥接程序

`path_bridge.py` 与本工程 PATH 固件配套使用，不能只更新 STM32。
复用原视觉压缩包里的 `rdk_vision` 和 `tools/hiwonder_action.py`、`tools/disc_rtl_trigger.py`。
不修改舵控板中的 G101/G102，也不覆盖标定后的 `rdk_vision/config.yaml`。

## 首次部署

1. 在 RDK 执行 `systemctl cat rdk-disc.service`，记录原服务的 WorkingDirectory 和 Python 解释器。
2. 停止旧服务：`sudo systemctl stop rdk-disc.service`。关闭其他摄像头预览和舵机调试程序。
3. 将此目录的 `path_bridge.py` 复制到原视觉工程的 `tools/path_bridge.py`。
4. 进入原视觉工程目录，使用原服务的 Python 环境运行：

```sh
python3 tools/path_bridge.py --project-root . --stm32-port /dev/ttyUSB0 --arm-port /dev/ttyS1
```

默认 STM32 115200、舵机 9600、目标色 red、trigger-x 380。
如原服务使用虚拟环境，以上 `python3` 换成原解释器的完整路径。
等待 `PATH bridge ready`；启动桥接本身不会执行机械臂动作。
连接设备文件可能随 USB 插拔改变，部署时核对实际路径。

验证通过后才修改原 systemd 服务的 ExecStart 为这个入口（项目根目录使用绝对路径），保持正确的工作目录与解释器。
执行 `sudo systemctl daemon-reload` 后启动该服务；不要同时运行旧入口与新入口。
使用 `journalctl -u rdk-disc.service -f` 查看结果。

## 协议

ASCII 行，以 CRLF 或 LF 结束，最多 95 字节，不支持旧版 DISC_START 整批协议。

```text
Q <session> <sequence> HELLO 0
Q <session> <sequence> GROUP <group>
Q <session> <sequence> VISION <timeout_ms>
Q <session> <sequence> DISC <timeout_ms>
Q <session> <sequence> STOP 0
R <session> <sequence> ACK|DONE|NONE|ERROR
```

会话与序号是 1..4294967295。GROUP 是 0..255。识别预算是 1..60000 毫秒。
VISION 只报告完整目标是否出现，不执行机械臂；DISC 检测成功后执行一次完整 G102。
G101 由 STM32 单独请求。DISC 的 DONE 只代表此次动作完成，不代表 RFID 已读到或五球任务完成。
NONE 表示未识别到或识别被取消，ERROR 表示执行失败。

STM32 必须每 500 ms 重发尚未完成的同一请求，包括收到 ACK 以后；不会重复执行机械动作。
超过 2 秒没有当前请求续期，取消识别，不再发起新的抓取。
STOP 取消识别，但等待已在运行的动作组完成后才回复 DONE。
舵机动作最长等待 30 秒；没有完成反馈时锁定 ERROR，不能假定机械臂已经停好。

记录最近 16 个事务；已移出缓存的旧序号也不能重执行。
进程最多接受初始会话加 32 次会话切换，达到上限后拒绝新会话，而不是遗忘旧会话。
遇到锁定错误或会话上限，应先人工确认机构安全，再重启桥接程序；不要将重启当作自动故障恢复。

## 图像诊断

每 0.5 秒输出当前判定（即使失败原因未变化），触发时立即输出。
`valid=False` 配合 `event=TRIGGER_EARLY/TRIGGER_EDGE` 可以正常抓取。
判断不能只看 valid；同时看 reason、bbox、aspect、fill 和 event。
相机实际分辨率与保存配置不一致会报错，停止出新帧超过 1 秒也会报错。
圆柱和楼梯调用同一目标色检测器，其视角和参考尺寸仍需按实际姿态验证。

## 软件测试（不操作硬件）

在 STM32 工程根目录运行：

```sh
python -m unittest discover -s tests -p test_path_bridge.py -v
```
