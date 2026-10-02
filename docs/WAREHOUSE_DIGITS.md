# 仓库数字识别部署

## 功能

第一物理列连续三个新鲜帧识别到同一数字 1/2/3，进入数字模式；没有可信数字（含数字相机未配置、不可用、超时），整次仓库锁定默认物理 1→2→3，后续不再请求数字识别。下次 PATH 重新判定。

数字模式下，后两列仍沿原路线移动，识别到未使用的数字就发 HOLD；等待底盘停稳，完成原灰度到位和地图 0° 检查后放球。数字仅决定 RFID 列号，高半字节行号及 GROUP 109/110/111 不变。不做像素测距、视觉角度修正或额外定位移动。后两列数字识别失败会停止并保留库存，不混用默认映射。

## 配置两路 USB 相机

1. RDK 上运行 `v4l2-ctl --list-devices` 和 `ls -l /dev/v4l/by-id /dev/v4l/by-path`。逐个检查可采集的 video 节点，区别图像节点和 metadata 节点。
2. 保持原球相机配置及原 ROI/HSV 参数。把数字相机稳定路径填入 `RDK/rdk_vision/warehouse_number.yaml` 的 `device`。已配置本次确认的 RGB 数字相机稳定路径；若设备变更，请重新核对。
3. 两路相机不能填同一采集节点。数字模块在仓库使用独立采集对象，先释放已结束球任务的相机，减少 USB 带宽占用。
4. 调整数字 `roi: [x,y,width,height]`，只包含底部标牌；优先保留完整数字，避免积木块数字、反光和文字进入 ROI。上下截断时使用局部模板，模板至少保留 65% 高度，并采用更严格的 0.72 相似度和 0.16 领先间隔；左右截断或轮廓不明仍拒绝识别。这些阈值不代表实际识别准确率。默认 640×480/30 fps、完整画面 ROI、三帧确认、3 s 识别窗口、250 ms 新鲜度。
5. 数字模板按规则参考 Times New Roman 生成，随代码分发，无额外 OCR 模型。现场字体、透视、模糊和曝光仍需测试；合成图通过不能代替实际标牌验证。

## 更新与检查

STM32 与 RDK 必须一起更新。旧 RDK 不支持 `WAREHOUSE_CHECK`，会回未知命令，不能用旧视觉程序验证此功能。更新 RDK/tools/rdk_stm32_bridge.py、RDK/rdk_vision/warehouse_digits.py、warehouse_number.yaml 和 digit_templates/1.png、2.png、3.png，然后重启现有 bridge 服务。无需更改现有球识别参数或机械臂动作组。

先分别验证：数字 1/2/3、数字移出视野、相机拔除、洗牌顺序 3/1/2、第一列遮挡后第二列可见。最后一种情况必须整仓保持默认 123。实跑验证识别时机及刹停距离，确认数字触发停车时机械臂不提前动作。

串口请求 `WAREHOUSE_CHECK <token> <excluded_mask>`；回复 `WAREHOUSE_DIGIT <token> <0|1|2|3>`，0 为未识别。已使用数字对应位被排除，旧 token 回复不影响当前动作。STM32 可选请求上限 5 s，RDK 识别上限 3 s（相机驱动阻塞由独立采集线程隔离），默认模式不再请求数字。

## 2026-10-02 现场验证与更新

RDK 通过 10.232.199.232 连接，数字相机已确认是 RGB Camera，稳定路径为 `/dev/v4l/by-id/usb-RYS_RGB_RGB_Camera_200901010001-video-index0`。原 Sonix 球相机和球识别参数保留。

更新了桥接程序、数字识别模块、相机配置和三个数字模板，并重启 `rdk-disc.service`，服务状态 active。原程序备份在 `/home/sunrise/licang_vision/.warehouse-backup-20261002-28617bfa`。

在本次用户摆放的静止位置，数字 2、1、3 分别连续五次成功识别：2 相似度约 0.88，1 约 0.94～0.96，3 约 0.84。单次含打开相机约 1.0～1.2 秒。早期快速重开有一次相机打开失败；已增加正常相机释放等待，随后 1、3 连续重开没有出现该问题。

实际截断 3 的笔画被图像上边缘分割成两个连通块，已加入同一上/下边缘附近的片段组合，不降低局部识别阈值。三个实际画面均加入主机回归。最终代码在 RDK 回放三个记录画面分别得到 1/2/3，且以服务用户 sunrise 完成一次真实相机桥接测试，得到 `WAREHOUSE_DIGIT 1001 3`。主机 RDK 测试 99 项通过。

这些是当前摆放下的静态识别结果，不代表移动、不同距离/光照、其他截断情况的准确率，也不表示模板相似度就是正确概率。STM32 新固件尚未烧录，本次没有驱动底盘或机械臂，没有验证实车识别后的刹停与放球。

## Current maintenance entry points (2026-10-02)

`App/path_warehouse.h` names warehouse phases and modes without changing telemetry IDs. `App/path_config.h` centralizes creep distance/rpm/acceleration, digit deadlines, and stair entry advance; line braking remains independent.

`NumberCameraSession` owns persistent capture and the reusable detector. Each request owns a new confirmation gate and ignores repeated/pre-request frames. Capture owns driver release; close only signals and waits for a bounded time. Reconnection occurs in the background. Both first-column fallback and digit success brake before alignment and unloading.

The persistent-camera, early stair preparation, and 5 mm entry changes pass local host tests and Release LTO compilation but still require deployment and vehicle validation. Earlier deployment/static results do not validate these newer changes.
