# LeKiwi 网球机器人使用说明书

本文面向现场使用和调试，说明当前 C++ 工程的设计、运行方法、校准方法、抓取调参、桶识别和常见问题处理。

## 1. 工程用途

本工程运行在 RK3588/Orange Pi 上，用 C++ 实现完整用户态闭环：

```text
摄像头采集
-> RKNN 网球检测
-> 三轮底盘追球和对准
-> Feetech 机械臂抓球
-> 红色桶搜索
-> 靠近桶并放球
-> 回到追球
```

工程保留旧 aka-rk3588 平台代码，但当前机器人使用 LeKiwi/Feetech 路径。

当前硬件约定：

```text
设备地址：orangepi@10.3.10.24
工程目录：/home/orangepi/robot/aka-rk3588
Feetech 总线：/dev/ttyACM0
机械臂电机：1-6
底盘电机：7-9
默认模型：models/tennis.rknn
校准文件：config/lekiwi_calibration.json
抓取配置：config/lekiwi_pick_config.txt
```

## 2. 代码结构

主要文件：

```text
tennis.cpp
```

主状态机。负责初始化平台、读取图像、调用检测、切换追球/抓球/找桶/放球状态。

```text
detect/
3rd/yolov8_src/
```

RKNN YOLOv8 检测相关代码。

```text
capture/uvc_capture.*
```

UVC 摄像头采集。

```text
feetech/feetech_bus.*
```

Feetech STS3215 总线通信。

```text
robot/omni_base.*
```

三轮 LeKiwi 底盘控制。

```text
robot/feetech_arm.*
```

机械臂关节映射、校准使用、姿态写入。

```text
robot/lekiwi_task_controller.*
```

当前 LeKiwi 闭环控制核心，包括追球视觉伺服、找桶视觉伺服、机械臂 IK 抓取和放球动作。

```text
test_cmds.cpp
```

调试命令入口，包括视觉、桶检测、Feetech 总线、底盘、机械臂测试。

## 3. 模型和配置

模型放在：

```text
models/tennis.rknn
```

`run_lekiwi_loop.sh` 和 `run_vision_once.sh` 默认都会使用该模型。

抓取配置放在：

```text
config/lekiwi_pick_config.txt
```

修改抓取配置后通常不需要重新编译，只需要重新运行命令。

机械臂校准文件放在：

```text
config/lekiwi_calibration.json
```

没有有效校准文件时，机械臂和完整闭环会拒绝运行。

## 4. 编译

在 Orange Pi 上：

```bash
cd /home/orangepi/robot/aka-rk3588
PKG_CONFIG_PATH=/home/orangepi/miniforge3/envs/rknn/lib/pkgconfig \
LD_LIBRARY_PATH=/home/orangepi/miniforge3/envs/rknn/lib:$LD_LIBRARY_PATH \
./build_rk3588.sh -b Release -l INFO
```

常用脚本会自动编译，因此日常完整运行可以直接执行 `./run_lekiwi_loop.sh`。

## 5. 推荐测试顺序

第一次上电、换线、换模型或调机械结构后，按下面顺序测试。

### 5.1 检查 Feetech 总线

```bash
cd /home/orangepi/robot/aka-rk3588
./build/tennis test-feetech /dev/ttyACM0 scan
./build/tennis test-feetech /dev/ttyACM0 read
```

期望结果：

```text
found 9 motor(s): 1 2 3 4 5 6 7 8 9
```

如果缺少某个 ID，先检查电源、舵机线、总线连接和 ID 配置。

### 5.2 检查机械臂校准

```bash
./build/tennis test-new-arm /dev/ttyACM0 calib-check
```

期望结果：

```text
calibration ok
```

如果校准不存在或失效，执行：

```bash
./build/tennis test-new-arm /dev/ttyACM0 calibrate
```

校准时按提示把每个机械臂关节移动到安全两端，程序会记录端点并计算中间映射。

### 5.3 检查机械臂初始姿态

```bash
./build/tennis test-new-arm /dev/ttyACM0 pos
```

如果夹爪里夹着球、机械臂卡住或关节受力，可能会出现 Feetech 错误，例如 6 号夹爪电机返回错误。此时先取下球，确认机械结构没有受力，再重试。

### 5.4 检查视觉模型

```bash
./run_vision_once.sh
```

Linux 下脚本会先编译再运行；Starry 下会直接运行已经部署的 `build/tennis`，因此
需要先在 Linux 下完成编译和根文件系统同步。脚本默认使用 NPU core 0。

输出：

```text
capture.jpg
result.jpg
```

`capture.jpg` 是原图，`result.jpg` 是检测结果。

### 5.5 检查桶识别

```bash
./build/tennis test-bucket 0
```

当前桶检测不是模型检测，而是 HSV 红色区域检测。需要放一个明显的红色桶、红色盒子，或者在桶外贴大面积红色纸。

## 6. 完整闭环运行

```bash
cd /home/orangepi/robot/aka-rk3588
./run_lekiwi_loop.sh
```

等价命令：

```bash
./build/tennis models/tennis.rknn /dev/ttyACM0 0 /dev/ttyACM0 lekiwi
```

运行流程：

```text
CHASE_BALL
-> PICK_BALL
-> FIND_BUCKET
-> PUT_BALL
-> CHASE_BALL
```

关键日志：

```text
LEKIWI_CHASE label=BALL_FORWARD
```

正在追球前进。

```text
LEKIWI_CHASE label=BALL_FINE_LEFT / BALL_FINE_RIGHT
```

球中心偏左或偏右，正在小角度对准。

```text
LEKIWI_CHASE label=BALL_READY ready=1
```

球的位置和距离稳定，准备抓取。

```text
[GAME] -> PICK_BALL
[GAME] PICK_BALL done gripper=45.4 gripper_hold=yes ball_visible=no area=0.000 off=0 size=0 label=IDLE holding=yes
```

抓取完成，夹爪反馈像是夹住，且抓取后视觉里不再看到球，因此判断夹住。

```text
[GAME] -> FIND_BUCKET
[GAME] LEKIWI_BUCKET visible=0 label=BUCKET_SEARCH L=12 R=-12
```

开始找桶。视野内没有红色桶时，会旋转搜索。

```text
[GAME] -> PUT_BALL
[GAME] PUT_BALL done -> CHASE_BALL
```

桶到位，完成放球并回到追球。

## 7. 追球设计

当前追球不是用真实距离，而是用图像中的目标框尺寸和中心偏差。

核心配置在：

```text
config/lekiwi_pick_config.txt
```

相关参数：

```text
ball_target_size = 155
ball_size_tolerance = 5
ball_center_tolerance = 12
stable_frames = 10
```

含义：

```text
ball_target_size
```

球检测框目标尺寸。值越大，车停得越近；值越小，车停得越远。

```text
ball_size_tolerance
```

允许的尺寸误差。当前为 5，表示目标尺寸上下约 5 像素内认为距离合适。

```text
ball_center_tolerance
```

允许的球中心误差。当前为 12，超过该误差会继续微调方向，不会直接抓取。

```text
stable_frames
```

连续稳定帧数。当前为 10。

## 8. 抓取设计

当前抓取是逆运动学与关键姿态的混合控制：接近和夹球使用已经验证的二维 IK，收臂
使用实机记录的 CARRY 关节姿态。

抓取动作大致为：

```text
移动到 home
肩部水平关节偏转 shoulder_pan_delta
打开夹爪
调整腕部角度 wrist_pick_pitch
移动到 pre_grab
移动到 grab
关闭夹爪
沿原路径返回 pre_grab 高度（CLEAR）
使用五次 S 曲线进入 carry
保持 carry 约0.5秒并确认关节反馈稳定
允许启动车轮
```

抓取完成后同时使用夹爪反馈和抓取后视觉复核：

```text
gripper_hold = arm_gripper > 25
ball_visible = 抓取后这一帧仍能检测到球
holding      = gripper_hold && !ball_visible
```

如果没有夹住，程序不会直接去找桶。如果球仍在视野内且仍是 `BALL_READY`，会立即使用下一组小偏移再次抓取；如果球仍在视野内但不再到位，会回到追球状态重新视觉对准。

当前抓取参数：

```text
pre_grab_x = 0.1200
pre_grab_y = 0.1211
grab_x = 0.1200
grab_y = -0.0600
wrist_pick_pitch = 80
carry_shoulder_pan = -11.3
carry_shoulder_lift = -18.3
carry_elbow_flex = -45.0
carry_wrist_flex = 51.8
carry_wrist_roll = 0.1
carry_duration_ticks = 40
carry_settle_ticks = 10
```

## 9. 抓取调参

调参文件：

```text
config/lekiwi_pick_config.txt
```

修改后重新运行命令即可。

推荐先只测机械臂：

```bash
./build/tennis test-new-arm /dev/ttyACM0 ik-pick
```

现场调参规则：

```text
夹爪靠前、伸过球：减小 grab_x，并同步减小 pre_grab_x
夹爪靠后、够不到球：增大 grab_x，并同步增大 pre_grab_x
夹爪太高：减小 grab_y
夹爪太低、压地或压球：增大 grab_y
```

单位是米：

```text
0.005 = 0.5cm
0.010 = 1cm
0.020 = 2cm
```

例如夹球位置向前 2cm：

```text
pre_grab_x += 0.020
grab_x += 0.020
```

例如夹球位置向上 1cm：

```text
grab_y += 0.010
```

通常只改 `grab_y`，不改 `pre_grab_y`，这样机械臂仍然先在安全高度移动，再下探抓球。

夹爪角度调节：

```text
wrist_pick_pitch = 80
```

如果夹爪闭合时不是尽量垂直向下，而是明显前倾或后仰，每次改 5 观察：

```text
wrist_pick_pitch = 75
wrist_pick_pitch = 85
```

## 10. 自动重试

抓取失败后，程序会根据抓取后视觉状态选择立即重试或重新追球，并使用下一组偏移：

```text
(0, 0)
(-0.005, 0)
(-0.010, 0)
(-0.015, 0)
(-0.020, 0)
(+0.010, 0)
(0, -0.010)
(0, +0.010)
(-0.015, -0.010)
(-0.015, +0.010)
```

第一个值加到 `pre_grab_x` 和 `grab_x`，第二个值加到 `pre_grab_y` 和 `grab_y`。

如果某次偏移成功，程序会保存成功参数到：

```text
config/lekiwi_pick_config.txt
```

## 11. 桶识别和放球

当前桶检测是红色 HSV 检测，不是神经网络模型。

需要准备：

```text
亮红色塑料桶
红色收纳盒
外侧贴大面积红色纸的桶
```

不推荐：

```text
暗红、棕红、橙红
反光很强的红色金属桶
红色区域很碎的小物体
背景中有更大的红色物体
```

视野里没有桶时，正常行为是旋转搜索：

```text
LEKIWI_BUCKET visible=0 label=BUCKET_SEARCH L=12 R=-12
```

看见桶后，会根据桶中心和尺寸靠近。到位后进入放球动作。

## 12. 安全操作

关闭所有 Feetech 电机扭矩：

```bash
./build/tennis test-feetech /dev/ttyACM0 torque-off
```

机械臂动作异常、卡住、夹爪里有球导致启动失败时，先取下球或解除受力，再执行：

```bash
./build/tennis test-new-arm /dev/ttyACM0 pos
```

如果完整闭环正在运行，按 `Ctrl-C` 停止。程序会尝试让底盘停止并关闭摄像头。

## 13. 常见问题

### 13.1 Failed to configure Feetech arm

先执行：

```bash
./build/tennis test-feetech /dev/ttyACM0 scan
./build/tennis test-feetech /dev/ttyACM0 read
./build/tennis test-new-arm /dev/ttyACM0 pos
```

如果错误指向 6 号夹爪电机，并且夹爪里有球或夹爪受力，先取下球再试。

### 13.2 一直追球不抓

看日志中的 `size` 和 `off`：

```text
size 接近 155
off 绝对值不超过 12
ready 连续稳定到 1
```

如果尺寸在 149 和 162 之间来回跳，可以适当增大 `ball_size_tolerance`，例如从 5 改到 8。

### 13.3 抓住球后不找桶

正常应看到：

```text
-> FIND_BUCKET
LEKIWI_BUCKET visible=0 label=BUCKET_SEARCH
```

如果没有红色桶，它会旋转搜索，不会放球。

### 13.4 桶不识别

先运行：

```bash
./build/tennis test-bucket 0
```

确保画面里有大面积亮红色区域。桶不是红色、太暗、太小或背景有红色干扰，都会影响识别。

### 13.5 机械臂方向明显不对

优先检查校准：

```bash
./build/tennis test-new-arm /dev/ttyACM0 calib-check
```

必要时重新校准：

```bash
./build/tennis test-new-arm /dev/ttyACM0 calibrate
```

不要优先用原始姿态文件掩盖校准问题。

## 14. 本地和开发板同步

本地同步到 Orange Pi：

```bash
./scripts/sync_to_orangepi.sh
```

从 Desktop-Wanderer 拉取模型到本工程默认位置：

```bash
./scripts/sync_from_orangepi.sh
```

同步后默认模型仍应位于：

```text
models/tennis.rknn
```
