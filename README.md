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

完整中文说明书见：

```text
docs/lekiwi_user_manual.md
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

Linux/Starry 共用 rootfs 验证时优先使用 `auto`，程序会先尝试 userspace libusb
CDC 后端，失败后再回退到 `/dev/ttyACM0`：

```bash
./build/tennis test-feetech auto scan
./build/tennis test-new-arm auto calib-check
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
./build/tennis test-new-arm /dev/ttyACM0 ik-put
./build/tennis test-new-arm /dev/ttyACM0 release
./build/tennis test-new-arm /dev/ttyACM0 show
./build/tennis test-new-arm /dev/ttyACM0 torque-off
```

用途：

- `pos`：移动到初始/待机姿态。
- `grab`：执行旧的语义抓取动作。
- `ik-pick`：执行当前闭环使用的逆运动学抓取序列，用于单独调试抓球位置。
- `ik-put`：执行高位悬停、慢速下降、放球、抬升和收臂的完整放球序列。
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

完整演示“寻找并靠近桶 → 停车 → 机械臂放球 → 撤离”一次后自动退出：

```bash
./run_bucket_place_demo.sh
```

演示模式不经过找球和抓球，启动时保留夹爪当前位置，并先平滑进入 CARRY。夹爪内
有球时会实际释放球；首次调试应取出球，空载确认停车距离和机械臂轨迹。

推荐先单独测试机械臂抓取：

```bash
./build/tennis test-new-arm /dev/ttyACM0 ik-pick
```

放球必须分阶段确认，确认前两步不会碰桶后再执行完整循环：

```bash
./build/tennis test-new-arm /dev/ttyACM0 config-check
./build/tennis test-new-arm /dev/ttyACM0 task carry
./build/tennis test-new-arm /dev/ttyACM0 task place-approach
./build/tennis test-new-arm /dev/ttyACM0 task place-release
./build/tennis test-new-arm /dev/ttyACM0 task place-cycle
```

`place_id1_deg`～`place_id5_deg` 是精确的桶内最终释放姿态；其中 ID2、ID3、ID4
可直接按实机需要设置，不再经过 IK 偏移换算，也不会影响固定的高位接近姿态。
前后距离优先通过 `bucket_stop_size_px` 调整；桶中心精调使用
`bucket_center_tolerance_px`，距离和中心连续满足 `bucket_stable_frames` 帧后才放球。
机械臂、夹爪以及接近球和桶的速度统一由 `motion_speed_level` 控制：1调试、2稳定、
3快速、4极速。当前4档对应普通机械臂50°/s、HOME 25°/s、夹爪60°/s；
接近球远速65、接近桶远速70，目标附近分别降到20和25。
控制器按50ms绝对周期和真实时间S曲线运行，即使通信延迟，单周期命令仍限幅，不会
突然加速追赶。启动时从未知姿态回HOME仍保持25°/s安全上限。

常用调参规则：

- 夹爪伸过球：减小 `grab_forward_offset_cm`。
- 夹爪够不到球：增大 `grab_forward_offset_cm`。
- 夹爪偏左/偏右：调整 `grab_lateral_offset_cm`，正数向左、负数向右。
- 夹爪过低/过高：调整 `grab_height_offset_cm`，正数升高、负数降低。
- 夹球俯仰角不合适：每次调整 `grab_pitch_offset_deg` 约 `5` 度。
- 位置偏移单位为厘米，每次建议只改 `0.5`。
- `PICK_BALL done` 日志会同时打印夹爪反馈和抓取后视觉复核。只有 `gripper_hold=yes` 且 `ball_visible=no` 时，最终 `holding` 才会是 `yes`。

完整闭环中，如果第一次没有夹住，程序不会去找桶。抓取动作结束后会重新取一帧图像：如果球仍在视野内且仍处于 `BALL_READY` 区域，立即使用下一组小偏移再次抓取；如果球还在视野内但不再满足抓取条件，则先回到追球状态重新视觉对准。

```text
(0, 0)
(-0.5, 0)
(-1.0, 0)
(-1.5, 0)
(-2.0, 0)
(+1.0, 0)
(0, -1.0)
(0, +1.0)
(-1.5, -1.0)
(-1.5, +1.0)
```

这两个数依次是前后、高度偏移，单位也是厘米。如果某次偏移夹住了球，程序会把
成功的偏移写回 `config/lekiwi_pick_config.txt`。如果所有偏移都失败，程序会重新
从第一组参数开始追球和对准。

底盘停车距离由 `ball_stop_size_px` 控制：

- 值越大，车停得越近。
- 值越小，车停得越远。
- 当前默认 `155`，实机成功夹球时检测尺寸为 `154`。

进入抓取前还会检查球中心误差：

```text
ball_stop_tolerance_px = 5
ball_center_tolerance_px = 30
ball_stable_frames = 2
```

当前距离窗口是 `155±5`，即球框尺寸在 `150～160` 像素内；球心左右误差还需
不超过 `30` 像素，并连续满足 `2` 个检测帧才进入夹球。

各档速度、减速距离和等待时间由程序成组选择。程序根据检测尺寸的增长速度预测停车
位置，进入减速区后从远速连续降到近速；停止后再低速修正，因此高速档不会直接用高速
冲到目标。接近球时，左右偏差进入微调区后会
保持向前并通过左右轮差速修正；中心误差退出更小的滞回窗口后才恢复直行，避免检测
结果在阈值附近波动时反复出现“前进、停下原地转、再前进”。
StarryOS 真实 RKNN 闭环稳态约 `16fps`，检测中心会随架空车轮和画面抖动在
`+/-30px` 左右变化；如果窗口太窄，会一直输出 `BALL_FINE_LEFT/RIGHT`
或 `BALL_BACKWARD`，日志里反复出现 `BALL_READY ... ready=0` 但无法进入夹球。

夹球腕部角度由 `grab_pitch_offset_deg` 控制。如果夹爪明显倾斜，可以每次改 `5` 观察效果：

```text
grab_pitch_offset_deg = -5
grab_pitch_offset_deg = 5
```

更详细说明见：

```text
docs/lekiwi_cpp_closed_loop_migration.md
```

## 完整闭环

当前 Linux/Starry 联调阶段建议把机器人架起来，默认只验证“视觉检测到球并输出
LeKiwi 追球轮速控制”，不会继续进入抓球和找桶流程：

```bash
./run_lekiwi_test.sh
```

`run_lekiwi_loop.sh` 目前保留为兼容入口，等价于 `run_lekiwi_test.sh`。

脚本行为：

- Linux 下会先执行 `build_rk3588.sh` 编译，再运行 `build/tennis`。
- StarryOS 下不会编译，只检查并运行共享 rootfs 中已有的 `build/tennis`。
  判断 StarryOS 时同时检查 `uname -s` 和 `hostname=starry`，避免误走编译路径。
- StarryOS 下如果 `build/tennis` 不存在，需要先回到 Linux 编译。
- 脚本使用 `/bin/sh` 语法，避免 StarryOS 没有 bash 或 `/usr/bin/env` 时无法执行。

StarryOS 下直接等价于：

```bash
./build/tennis models/tennis.rknn auto 0 auto lekiwi --stop-after-chase
```

Linux 下默认使用 RK3588 三核 NPU。当前 StarryOS 三核 RKNPU 路径能跑完但输出
bbox 会塌到右下角，例如 `bbox=(640,480,*,0)`，会导致车一直 `BALL_RIGHT` 原地转。
StarryOS 下应先使用单核稳定模式：

```bash
RKNN_CORE_MASK=0 ./build/tennis models/tennis.rknn auto 0 auto lekiwi --stop-after-chase
```

`run_lekiwi_loop.sh` 在 StarryOS 下会自动设置 `RKNN_CORE_MASK=0`。Linux 真实检测路径
不需要该变量，默认仍为三核 `RKNN_NPU_CORE_0_1_2`。

达到连续追球确认条件后会主动停车退出，并打印：

```text
[SAFETY] STOP_AFTER_CHASE ...
```

如果要恢复完整闭环，可以显式关闭脚本的安全退出：

```bash
./run_lekiwi_full.sh
```

完整流程会要求输入 `RUN_FULL_LEKIWI` 确认，避免误触。如果确实需要无人值守启动，
当前调试阶段已去掉交互确认，执行 `./run_lekiwi_full.sh` 会直接进入完整流程。

完整闭环关键日志：

```bash
./build/tennis models/tennis.rknn /dev/ttyACM0 0 /dev/ttyACM0 lekiwi
```

```text
LEKIWI_CHASE      追球视觉伺服
-> PICK_BALL      开始抓球
PICK_BALL done    抓取完成并打印夹爪反馈和抓取后视觉复核
grab failed       抓取失败，立即重试或回到追球重新对准
-> FIND_BUCKET    抓取成功，开始找桶
-> PUT_BALL       桶到位，开始放球
PUT_BALL done     放球完成，回到追球
```

完整闭环会在夹球成功后进入找桶流程；无桶或线束可能被车体拖拽时不要运行完整闭环。

## Linux/Starry 统一入口说明

当前程序为 Feetech 总线提供两个后端：

```text
TTY 后端:
  直接打开 /dev/ttyACM0 等串口设备。

userspace libusb CDC 后端:
  通过 libusb 枚举 CDC ACM 设备，claim control/data interface，
  发送 SET_LINE_CODING 和 SET_CONTROL_LINE_STATE，再用 bulk IN/OUT
  传输 Feetech 协议包。
```

设备参数为 `auto` 时，选择顺序是：

```text
1. userspace libusb CDC
2. /dev/ttyACM0 TTY
```

`auto`、`usb`、`libusb`、`cdc` 会默认打印 FeetechBus 枚举和后端选择日志。
需要强制在普通 TTY 路径也打印调试信息时，可以设置：

```bash
LEKIWI_USB_DEBUG=1 ./build/tennis test-feetech /dev/ttyACM0 scan
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
