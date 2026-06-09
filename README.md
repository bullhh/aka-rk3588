# aka-rk3588 网球机器人

这是运行在 RK3588/Orange Pi 上的 C++ 用户态程序，用于摄像头采集、RKNN YOLOv8 网球识别、桶识别、底盘控制和机械臂控制。

当前工程同时保留两套硬件路径：

- 旧 aka-rk3588：差速底盘 + ZP10D 机械臂。
- 当前 LeKiwi/Feetech：三轮全向底盘 + STS3215 机械臂。

当前 Orange Pi 设备使用 LeKiwi/Feetech 路径：

```text
Feetech 总线：/dev/ttyACM0
机械臂电机：1-6
底盘电机：7-9
默认模型：models/tennis.rknn
校准文件：config/lekiwi_calibration.json
抓取调参文件：config/lekiwi_pick_config.txt
```

## 编译

在 Orange Pi 上执行：

```bash
cd /home/orangepi/robot/aka-rk3588
PKG_CONFIG_PATH=/home/orangepi/miniforge3/envs/rknn/lib/pkgconfig \
LD_LIBRARY_PATH=/home/orangepi/miniforge3/envs/rknn/lib:$LD_LIBRARY_PATH \
./build_rk3588.sh -b Release -l INFO
```

## 视觉单独测试

该命令只启动摄像头和识别，不初始化底盘和机械臂：

```bash
./run_vision_once.sh
./run_vision_once.sh models/tennis.rknn 0
```

输出文件：

```text
capture.jpg
result.jpg
```

## Feetech 总线测试

```bash
./build/tennis test-feetech /dev/ttyACM0 scan
./build/tennis test-feetech /dev/ttyACM0 read
./build/tennis test-feetech /dev/ttyACM0 torque-off
```

用途：

- `scan`：扫描 1-9 号电机是否在线。
- `read`：读取电机当前位置、电压、温度。
- `torque-off`：关闭扭矩，便于手动移动机械臂。

## 底盘测试

每个动作会短暂执行，然后自动停止：

```bash
./build/tennis test-base /dev/ttyACM0 forward 0
./build/tennis test-base /dev/ttyACM0 backward 0
./build/tennis test-base /dev/ttyACM0 left 0
./build/tennis test-base /dev/ttyACM0 right 0
./build/tennis test-base /dev/ttyACM0 rotate-left 0
./build/tennis test-base /dev/ttyACM0 rotate-right 0
./build/tennis test-base /dev/ttyACM0 stop
```

## 机械臂校准

第一次使用或更换机械臂结构后需要校准：

```bash
./build/tennis test-new-arm /dev/ttyACM0 calibrate
```

校准流程会要求操作者把所有机械臂关节依次转到安全两端。程序记录每个关节的 `range_min/range_max`，并写入：

```text
config/lekiwi_calibration.json
```

检查校准文件：

```bash
./build/tennis test-new-arm /dev/ttyACM0 calib-check
```

如果没有有效校准文件，机械臂命令和完整 LeKiwi 闭环会拒绝运行。

## 机械臂测试

```bash
./build/tennis test-new-arm /dev/ttyACM0 pos
./build/tennis test-new-arm /dev/ttyACM0 grab
./build/tennis test-new-arm /dev/ttyACM0 ik-pick
./build/tennis test-new-arm /dev/ttyACM0 release
./build/tennis test-new-arm /dev/ttyACM0 show
./build/tennis test-new-arm /dev/ttyACM0 torque-off
```

用途：

- `pos`：移动到初始/待机姿态。
- `grab`：执行旧的语义抓取动作。
- `ik-pick`：执行当前闭环使用的逆运动学抓取序列，用于单独调试抓球位置。
- `release`：打开夹爪放球。
- `show`：抬起展示姿态。
- `torque-off`：关闭机械臂扭矩。

夹爪单独测试：

```bash
./build/tennis test-new-arm /dev/ttyACM0 set arm_gripper 60
./build/tennis test-new-arm /dev/ttyACM0 set arm_gripper 0
```

当前实测：

```text
arm_gripper 60：打开
arm_gripper 0：完全关闭
```

## 抓取调参

抓取参数文件：

```text
config/lekiwi_pick_config.txt
```

修改该文件后不需要重新编译，重新执行命令即可生效。

推荐先单独测试机械臂抓取：

```bash
./build/tennis test-new-arm /dev/ttyACM0 ik-pick
```

常用调参规则：

- 夹爪伸过球、靠前闭合：减小 `grab_x`，通常同步减小 `pre_grab_x`。
- 夹爪够不到球：增大 `grab_x`。
- 夹爪太高或太低：调整 `grab_y`。
- 每次建议改 `0.005` 米。
- 如果已经能明显估计偏差，例如夹爪整体靠前约 5cm，可以直接把 `grab_x` 和 `pre_grab_x` 同步减小 `0.05`。
- 日志中 `gripper=18.6 holding=no` 这类结果表示空夹；抓住球后应显示 `holding=yes`。

完整闭环中，如果第一次没有夹住，程序不会去找桶，也不会在原地连续夹取。因为夹爪可能已经碰到球，球的位置会变化。程序会先回到追球状态重新视觉对准，然后使用下一组小偏移再次抓取：

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

如果某次偏移夹住了球，程序会把成功参数写回 `config/lekiwi_pick_config.txt`。如果所有偏移都失败，程序会重新从第一组参数开始追球和对准。

抓取后如果看到机械臂肩部向右转一下，那是 Python 原动作里的肩部回正。现在可以通过配置关闭：

```text
return_shoulder_pan = 0
```

调抓球时建议保持 `0`，避免干扰观察。

底盘停车距离由 `ball_target_size` 控制：

- 值越大，车停得越近。
- 值越小，车停得越远。
- 当前默认 `155`，让车比 `168~170` 像素时稍远一点停车。

进入抓取前还会检查球中心误差：

```text
ball_center_tolerance = 12
```

也就是球不能像之前 `off=27/28` 那样偏得太多，否则会继续慢速对准，不会直接抓取。

夹爪闭合角度由 `wrist_pick_pitch` 控制。如果夹爪不是尽量垂直向下，而是明显倾斜，可以每次改 `5` 观察效果：

```text
wrist_pick_pitch = 75
wrist_pick_pitch = 85
```

更详细说明见：

```text
docs/lekiwi_cpp_closed_loop_migration.md
```

## 完整闭环

确认视觉、总线、底盘、机械臂和抓取位置都正常后运行：

```bash
./run_lekiwi_loop.sh
```

等价命令：

```bash
./build/tennis models/tennis.rknn /dev/ttyACM0 0 /dev/ttyACM0 lekiwi
```

关键日志：

```text
LEKIWI_CHASE      追球视觉伺服
-> PICK_BALL      开始抓球
PICK_BALL done    抓取完成并打印夹爪反馈
grab failed       抓取失败，回到追球
-> FIND_BUCKET    抓取成功，开始找桶
-> PUT_BALL       桶到位，开始放球
PUT_BALL done     放球完成，回到追球
```

## 原始位置姿态调试

原始位置姿态只用于排查硬件方向、关节范围或临时验证姿态，不作为最终自动抓取策略。

保存姿态：

```bash
./build/tennis test-feetech /dev/ttyACM0 torque-off
./build/tennis test-new-arm /dev/ttyACM0 pose-save home
```

回放姿态：

```bash
./build/tennis test-new-arm /dev/ttyACM0 pose-run home
./build/tennis test-new-arm /dev/ttyACM0 pose-list
```

姿态文件：

```text
config/lekiwi_arm_poses.txt
```

## 旧平台命令

旧 aka-rk3588 差速底盘和 ZP10D 机械臂仍保留：

```bash
./build/tennis test-arm /dev/ttyUSB1 pos
./build/tennis test-arm /dev/ttyUSB1 grab
./build/tennis test-motor /dev/ttyS3 speed=30
./build/tennis models/tennis.rknn /dev/ttyS3 0 /dev/ttyUSB1
```
