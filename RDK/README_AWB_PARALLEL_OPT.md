# RDK 圆盘任务：AWB 预热与 G101 并行优化补丁

本补丁建立在上一版 `rdk_awb_lock_fix_patch.zip` 的 AWB 预热/锁定修复之上。

## 改动

- 保留摄像头冷启动流程：基础 UVC 参数 -> AWB ON -> 丢弃 45 帧 -> AWB OFF 锁定。
- 圆盘任务启动时，不再先等待 G101 完成后才启动摄像头。
- 现在 Camera/AWB startup 与 G101 同时进行；两者都完成后才进入红球识别。
- 不修改 STM32 协议、G102 逻辑、trigger_x、ROI、HSV、reference size。
- 本补丁不包含 `rdk_vision/config.yaml`，不会覆盖 RDK 上最新标定数据。

## 更新

```bash
cd ~/licang_vision
sudo systemctl stop rdk-disc.service
cp tools/vision_servo_direct_test.py tools/vision_servo_direct_test.py.before_parallel_awb
unzip -o rdk_awb_parallel_g101_patch.zip
find . -name "__pycache__" -type d -exec rm -rf {} +
```

## 软件测试

```bash
cd ~/licang_vision
python3 -m unittest rdk_vision.tests.test_camera -v
python3 -m unittest tests.test_parallel_camera_prep -v
```

## 单独实物测试

```bash
python3 tools/vision_servo_direct_test.py \
  --config rdk_vision/config.yaml \
  --color red \
  --servo-port /dev/ttyS1 \
  --servo-baud 9600 \
  --prep-group 101 \
  --trigger-group 102 \
  --repeat 1 \
  --max-actions 5 \
  --trigger-x 380 \
  --servo-timeout 30
```

启动日志应体现：Camera/AWB warm-up 和 G101 同时开始；只有两者完成后才出现 `PREP + CAMERA READY G101` 并进入识别。

如果 G101 执行时间 >= AWB 预热时间，新增的 AWB 等待基本可被完全隐藏；如果 G101 更短，只会等待剩余的 AWB 时间。
