# LeKiwi 三轮机器人机械臂调试手册

本文面向当前三轮 LeKiwi/Feetech 机器人，说明如何编译、部署和运行
`aka-rk3588` 用户态程序，以及如何判断机械臂动作是否正常。本文不适用于旧的
两轮 ESP32/ZP10D 方案。

当前硬件约定：

```text
机械臂电机：Feetech STS3215，ID 1-6
三轮底盘电机：Feetech STS3215，ID 7-9
Linux 设备节点：/dev/ttyACM0
Linux/Starry 通用路径：userspace libusb CDC，设备参数使用 auto
模型：models/tennis.rknn
校准文件：config/lekiwi_calibration.json
抓取参数：config/lekiwi_pick_config.txt
```

## 1. 安全准备

1. 把机器人架起，保证三个轮子离地，或确保底盘周围没有障碍物。
2. 取下夹爪中的球，检查线束不会被关节夹住。
3. 保证机械臂运动范围内没有人手、工具和硬物。
4. 第一次执行新参数时随时准备按 `Ctrl-C`。
5. 手动移动机械臂前先关闭扭矩，不要强掰带扭矩的舵机。

关闭扭矩：

```bash
./build/tennis test-feetech auto torque-off
```

## 2. 源码目录和共享 rootfs

本地主源码目录：

```text
/home/szy/work/robot/tripod/aka-rk3588
```

Orange Pi Linux/Starry 共享 rootfs 中的推荐目录：

```text
/home/orangepi/robot/aka-rk3588
```

推荐在开发板 Linux 中同步源码并编译，执行 `sync` 后再重启进入 StarryOS。
StarryOS 直接运行共享 rootfs 中由 Linux 构建的 AArch64 动态 ELF，不在 StarryOS
中编译。

### 2.1 调试阶段启动开发板 Linux

板卡由 board server 管理。以下命令应在 `tgoskits` 仓库根目录执行：

```bash
cd /home/szy/work/robot/tripod/tgoskits
cargo board connect \
  --board-type OrangePi-5-Plus-robot \
  --server 10.3.10.60 \
  --port 2999
```

该命令申请并启动开发板 Linux，同时连接串口。通过串口启动日志或登录后的网络命令
查看机器人当前 IP 地址：

```bash
ip addr
ip -4 addr
hostname -I
```

记录 IP 后，另开一个终端通过 SSH 同步、编译和测试：

```bash
ssh orangepi@<board-ip>
```

不要只依赖同步脚本中的默认 IP；DHCP 地址可能变化。切换系统前先在 Linux 中执行
`sync`，再退出串口并释放板卡租约。串口退出通常使用：

```text
Ctrl-A x
```

### 2.2 调试阶段启动 StarryOS

退出 Linux 会话并释放租约后，在 `tgoskits` 根目录执行：

```bash
cd /home/szy/work/robot/tripod/tgoskits
cargo xtask starry board \
  -c os/StarryOS/configs/board/orangepi-5-plus.toml \
  --board-config apps/starry/orangepi-5-plus-uvc/board-orangepi-5-plus.toml \
  --board-type OrangePi-5-Plus-robot \
  --server 10.3.10.60 \
  --port 2999
```

正常现象是命令取得板卡租约，启动 StarryOS，并通过串口进入 `root@starry` shell。
进入 shell 后再切换到共享 rootfs 中的用户程序目录运行测试。

如果提示没有可用板卡，先确认上一个 Linux 或 StarryOS 会话已经退出并释放租约。
同一时刻不要启动两个板卡会话。

## 3. 同步源码到开发板

使用仓库脚本：

```bash
cd /home/szy/work/robot/tripod/aka-rk3588
./scripts/sync_to_orangepi.sh
```

默认目标是：

```text
orangepi@10.3.10.24:/home/orangepi/robot/aka-rk3588
```

该地址只是脚本默认值。调试阶段应先用 `cargo board connect` 从串口查看当前 IP，
再通过 `REMOTE` 传入实际地址。

开发板地址或目录不同时：

```bash
REMOTE=orangepi@<board-ip> \
REMOTE_PROJECT=/home/orangepi/robot/aka-rk3588 \
./scripts/sync_to_orangepi.sh
```

脚本不会同步本地 `build/`。也可以手工执行：

```bash
rsync -az --delete \
  --exclude .git \
  --exclude build \
  /home/szy/work/robot/tripod/aka-rk3588/ \
  orangepi@<board-ip>:/home/orangepi/robot/aka-rk3588/
```

## 4. 编译程序

### 4.1 推荐：开发板 Linux 原生编译

```bash
ssh orangepi@<board-ip>
cd /home/orangepi/robot/aka-rk3588
PKG_CONFIG_PATH=/home/orangepi/miniforge3/envs/rknn/lib/pkgconfig \
LD_LIBRARY_PATH=/home/orangepi/miniforge3/envs/rknn/lib:${LD_LIBRARY_PATH:-} \
./build_rk3588.sh -b Release -l INFO
```

成功现象：

```text
[100%] Built target tennis
=== Build done: .../build/tennis ===
Size: <非零字节数>
```

检查产物：

```bash
file build/tennis
ls -lh build/tennis models/tennis.rknn
```

`build/tennis` 应为 AArch64 ELF，模型文件必须存在。

### 4.2 可选：开发机交叉编译

开发机安装 `aarch64-linux-gnu-g++` 后执行：

```bash
cd /home/szy/work/robot/tripod/aka-rk3588
./build_rk3588.sh -b Release -l INFO
scp build/tennis \
  orangepi@<board-ip>:/home/orangepi/robot/aka-rk3588/build/tennis
```

交叉编译产物仍要求 rootfs 中存在兼容的 `librknnrt.so`、`libuvc`、`libusb-1.0`
和 `libturbojpeg`。不确定运行库是否匹配时，优先在开发板 Linux 原生编译。

## 5. 让文件进入 StarryOS 可见的 rootfs

在开发板 Linux 中执行：

```bash
cd /home/orangepi/robot/aka-rk3588
test -x build/tennis
test -f models/tennis.rknn
test -f config/lekiwi_calibration.json
sync
```

执行 `sync`，退出 Linux 串口并释放租约，再按照 2.2 节的命令启动 StarryOS。
StarryOS 启动后检查：

```sh
cd /home/orangepi/robot/aka-rk3588
ls -l build/tennis models/tennis.rknn config/lekiwi_calibration.json
```

如果 StarryOS 找不到新文件，先回 Linux 检查文件是否写入正确的物理 rootfs，以及
重启前是否执行了 `sync`。

## 6. 推荐调试顺序

每次修改机械臂代码或参数后按以下顺序验证。前一步失败时不要运行完整闭环。

### 6.1 扫描电机

Linux 和 StarryOS 通用命令：

```bash
cd /home/orangepi/robot/aka-rk3588
./build/tennis test-feetech auto scan
```

正常现象：

```text
found 9 motor(s): 1 2 3 4 5 6 7 8 9
```

`auto` 优先使用 userspace libusb CDC；Linux 下 claim 失败时回退到
`/dev/ttyACM0`。缺少某个 ID 时先检查电源、线缆、舵机 ID 和总线连接。

读取状态：

```bash
./build/tennis test-feetech auto read
```

重点观察位置、速度、电压和温度。

### 6.2 检查校准

```bash
./build/tennis test-new-arm auto calib-check
```

正常现象：

```text
calibration ok
```

只有在机械结构变化、更换舵机或确认原校准失效时才重新校准：

```bash
./build/tennis test-new-arm auto calibrate
```

### 6.3 测试待机姿态和夹爪

```bash
./build/tennis test-new-arm auto pos
./build/tennis test-new-arm auto set arm_gripper 60
./build/tennis test-new-arm auto set arm_gripper 0
```

当前约定：

```text
arm_gripper 60：打开
arm_gripper 0：完全关闭
```

正常现象是机械臂安全到达待机姿态、夹爪方向正确、到达机械限位后不继续强顶。

### 6.4 单独测试 IK 抓取动作

机械臂流畅度和角度的主要调试命令：

```bash
./build/tennis test-new-arm auto ik-pick
```

动作大致为：

```text
HOME
-> 肩部转向
-> 打开夹爪并调整腕部
-> PRE_GRAB
-> GRAB
-> 闭合夹爪
-> CLEAR（沿下降路径回到 PRE_GRAB 高度）
-> CARRY（约2秒五次 S 曲线收臂）
-> 保持约0.5秒，确认静止后允许车轮启动
```

正常现象：

- step 持续前进，最终显示 `done=1 failed=0`。
- 肩、肘和腕部连续移动，没有长时间停顿后突然跳动。
- 夹爪在接近球前打开，到达抓取点后闭合。
- 抬升时夹爪不碰地面、底盘或相机支架。

常见现象：

| 现象 | 优先检查 |
| --- | --- |
| 动作一顿一顿 | `tick()` 是否被视觉帧率限制 |
| 某阶段停几秒 | `GAP` 是否按视觉帧计数 |
| 夹爪伸过球 | 减小 `grab_x`，同步减小 `pre_grab_x` |
| 夹爪够不到球 | 增大 `grab_x` 和 `pre_grab_x` |
| 夹爪过高或过低 | 每次调整 `grab_y` 约 `0.005` 米 |
| 夹爪不垂直 | 每次调整 `wrist_pick_pitch` 约 `3-5` |
| 抬升姿态突变 | 检查 `lift_x/lift_y` 和 `wrist_lift_pitch` |
| USB 超时或校验错误 | 降低更新频率，检查供电、USB和线缆 |

参数位于：

```text
config/lekiwi_pick_config.txt
```

修改后不需要重新编译，重新运行 `ik-pick` 即可。每次只调整一个参数。

### 6.5 测试三轮底盘

必须先架起机器人：

```bash
./build/tennis test-base auto forward 0
./build/tennis test-base auto backward 0
./build/tennis test-base auto left 0
./build/tennis test-base auto right 0
./build/tennis test-base auto rotate-left 0
./build/tennis test-base auto rotate-right 0
./build/tennis test-base auto stop
```

正常现象是三个轮子按全向底盘映射协调运行，测试结束后自动停车。

### 6.6 视觉与安全追球

```bash
./run_vision_once.sh
```

正常情况下生成 `capture.jpg` 和 `result.jpg`。

架空运行安全追球：

```bash
./run_lekiwi_test.sh
```

正常现象是出现 `LEKIWI_CHASE`，确认控制输出后打印 `STOP_AFTER_CHASE` 并停车，
不会进入抓球和找桶。

StarryOS 下脚本自动使用 `RKNN_CORE_MASK=0`。当前不要用三核 NPU 执行真实闭环，
因为三核模式曾出现错误 bbox。

### 6.7 完整流程

只有前述测试全部通过、机器人架起且周围安全时才执行：

```bash
./run_lekiwi_full.sh
```

关键状态：

```text
LEKIWI_CHASE
-> PICK_BALL
PICK_BALL done
-> FIND_BUCKET
-> PUT_BALL
PUT_BALL done
-> LEKIWI_CHASE
```

出现以下情况应立即停止：底盘在机械臂动作期间仍移动、机械臂撞限位、同一步长时间
不前进、舵机抖动或过热，以及连续 USB timeout/checksum/status ID 错误。

## 7. 第一阶段流畅度调试方法

当前动作算法参考 Desktop-Wanderer。机械臂动作已固定为约20 Hz，GAP约300 ms，
HOME使用关节空间缓启动，夹球后先CLEAR，再以关节空间五次 S 曲线进入CARRY。

1. 运行 `test-new-arm auto ik-pick`，录像并记录总时长。
2. 在完整流程中录像同一机械臂阶段。
3. 若完整流程与独立测试节奏不同，检查 `PICK_BALL/PUT_BALL` 是否仍走固定周期循环。
4. 当前保持20 Hz；第一阶段不继续提高频率。
5. `GAP` 应保持约300 ms，不能乘上视觉帧间隔。

建议每次记录：

```text
日期：
系统：Linux / StarryOS
提交：
动作更新周期：
参数修改：
ik-pick 总时长：
是否流畅：
是否成功夹球：
异常日志：
视频文件：
```

## 8. 常用恢复命令

```bash
./build/tennis test-base auto stop
./build/tennis test-feetech auto torque-off
./build/tennis test-new-arm auto pose-list
./build/tennis test-new-arm auto pos
```
