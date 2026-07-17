# LeKiwi 三轮机器人机械臂调试手册

本文面向当前三轮 LeKiwi/Feetech 机器人，说明如何编译、部署和运行
`aka-rk3588` 用户态程序，以及如何判断机械臂动作是否正常。本文不适用于旧的
两轮 ESP32/ZP10D 方案。

当前硬件约定：

```text
ID1：肩部水平旋转        ID6：夹爪
ID2：肩部抬升            ID7：左轮
ID3：肘部弯曲            ID8：后轮
ID4：腕部俯仰            ID9：右轮
ID5：腕部旋转
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
如果运行时报 `GLIBC_x.xx not found`，不要替换开发板系统库，直接回到4.1节在开发板
Linux中重新编译。

## 5. 让文件进入 StarryOS 可见的 rootfs

在开发板 Linux 中执行：

```bash
cd /home/orangepi/robot/aka-rk3588
test -x build/tennis
test -f models/tennis.rknn
test -f config/lekiwi_calibration.json
test -f config/lekiwi_pick_config.txt
sync
```

执行 `sync`，退出 Linux 串口并释放租约，再按照 2.2 节的命令启动 StarryOS。
StarryOS 启动后检查：

```sh
cd /home/orangepi/robot/aka-rk3588
ls -l build/tennis models/tennis.rknn \
  config/lekiwi_calibration.json config/lekiwi_pick_config.txt
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

首次校准时程序会确认ID1～ID6存在并关闭扭矩。按终端提示，把除连续旋转腕部外的
各关节和夹爪分别缓慢移动到两个安全端点，确保每个关节都覆盖完整可用范围，然后按
回车。程序自动计算中点和零位并写入：

```text
config/lekiwi_calibration.json
```

校准文件保存原始编码范围和零点，不要用抓球动作参数替代它。校准完成后重新执行
`calib-check`；失败时不要继续运行机械臂动作。

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
-> 按grab_id1～grab_id5确定基础夹球姿态
-> 打开夹爪，自动计算安全接近轨迹
-> 应用前后、左右、上下和俯仰偏移
-> 到达GRAB
-> 闭合夹爪
-> CLEAR（沿原下降路径抬离地面）
-> CARRY（约2秒五次S曲线进入carry_id1～carry_id5）
-> 保持约0.5秒，确认静止后允许车轮启动
```

正常现象：

- step 持续前进，最终显示 `done=1 failed=0`。
- 肩、肘和腕部连续移动，没有长时间停顿后突然跳动。
- 夹爪在接近球前打开，到达抓取点后闭合。
- 抬升时夹爪不碰地面、底盘或相机支架。
- 有球时最终应看到 `holding=yes`；无球动作测试出现 `holding=no` 属于正常。

每次开始抓取都会打印解析结果：

```text
[LeKiwiArmController] grab ids=(-12.0,37.7,42.1,0.2,0.0) \
offset_cm=(forward=0.0,lateral=0.0,height=0.0) pitch_offset=0.0 \
resolved=(pan=-12.0,x=0.1199,y=-0.0600,pre_y=0.1211,pitch=80.0)
```

修改配置后先检查这行，确认 `ids` 和 `offset_cm` 是本次输入，`resolved` 是程序实际
使用的最终目标。若日志仍是旧值，说明旧进程没有退出或修改的不是当前目录下的文件。

#### 夹球与收球基础姿态

参数位于 `config/lekiwi_pick_config.txt`，角度均为标定后的度数，不是0～4095原始值：

```bash
cd /home/orangepi/robot/aka-rk3588
vi config/lekiwi_pick_config.txt
```

```text
grab_id1_deg ～ grab_id5_deg    夹爪闭合时的ID1～ID5基础姿态
carry_id1_deg ～ carry_id5_deg  抬球后、启动车轮前的ID1～ID5收臂姿态
```

ID6夹爪不记录到两组姿态中，继续使用：

```text
gripper_open_delta_deg = 60
gripper_close_delta_deg = -60
```

当前CARRY时间参数：

```text
carry_duration_ms = 2000   # 收臂S曲线约2秒
carry_settle_ms = 500      # 到位后再稳定约0.5秒
```

优先使用下面的位置偏移解决现场误差。只有零偏移仍无法得到合理姿态时，才重新手动
记录并替换 `grab_id*`；不要用 `carry_id*` 直接夹地面上的球。

#### 快速调整夹球位置

| 现象 | 修改方法 |
| --- | --- |
| 夹爪伸过球 | 减小 `grab_forward_offset_cm`，例如 `0 → -0.5` |
| 夹爪够不到球 | 增大 `grab_forward_offset_cm`，例如 `0 → +0.5` |
| 夹爪在球左边 | 减小 `grab_lateral_offset_cm`，使夹爪向右 |
| 夹爪在球右边 | 增大 `grab_lateral_offset_cm`，使夹爪向左 |
| 夹爪比球低 | 增大 `grab_height_offset_cm`，例如 `0 → +0.5` |
| 夹爪比球高 | 减小 `grab_height_offset_cm`，例如 `0 → -0.5` |
| 夹爪俯仰不合适 | 每次调整 `grab_pitch_offset_deg` 约 `5` 度 |

三个位置偏移的单位均为厘米。程序自动把前后、上下换算为ID2/ID3，把左右换算为
ID1，并补偿ID4保持原夹爪朝向。建议每次只改一个参数、每次只改0.5厘米。

修改后不需要重新编译，必须退出旧进程并重新运行：

```bash
./build/tennis test-new-arm auto ik-pick
```

#### 常见异常

| 现象 | 优先检查 |
| --- | --- |
| 动作一顿一顿 | `tick()` 是否被视觉帧率限制 |
| 某阶段停几秒 | `GAP` 是否按视觉帧计数 |
| 修改参数但动作没变 | 检查启动日志中的 `offset_cm`，确认文件路径和旧进程 |
| 零偏移姿态不合理 | 重新记录 `grab_id1_deg～grab_id5_deg` |
| 收臂停止姿态不合理 | 检查 `carry_id1_deg～carry_id5_deg` |
| 收臂太快或太慢 | 调整 `carry_duration_ms`，不要直接提高控制频率 |
| USB 超时或校验错误 | 降低更新频率，检查供电、USB和线缆 |

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

同一命令可在 Linux 和 Starry 使用：Linux 会先编译再运行，Starry 会跳过编译并
运行共享根文件系统中的 `build/tennis`。默认使用 NPU core 0，正常情况下返回0，
检测一次并生成 `capture.jpg` 和 `result.jpg`。

架空运行安全追球：

```bash
./run_lekiwi_test.sh
```

正常现象是出现 `LEKIWI_CHASE`，确认控制输出后打印 `STOP_AFTER_CHASE` 并停车，
不会进入抓球和找桶。

停车参数也在 `config/lekiwi_pick_config.txt`：

| 参数 | 当前值与准确含义 | 增减效果 |
| --- | --- | --- |
| `ball_stop_size_px` | `155`；球检测框宽、高中的较大值 | 增大：更靠近球停车；减小：更远停车 |
| `ball_stop_tolerance_px` | `15`；允许 `155±15`，即140～170像素 | 增大：容易停车但前后误差大；减小：距离一致但可能反复调整 |
| `ball_center_tolerance_px` | `30`；球心允许偏离目标中心±30像素 | 增大：容易抓取但左右误差大；减小：对得更正但可能左右摆动 |
| `ball_stable_frames` | `2`；距离和球心条件连续满足2帧 | 增大：过滤误检但等待更久 |

停车距离不对时先调 `ball_stop_size_px`，不要立即用机械臂前后偏移补偿。停车位置已经
稳定但夹爪仍有小误差时，再调三个 `grab_*_offset_cm`。当前Starry约2.3fps，连续2帧
约需0.9秒。

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

## 7. 推荐调参和记录方法

当前动作算法参考 Desktop-Wanderer。机械臂动作已固定为约20 Hz，GAP约300 ms，
HOME使用关节空间缓启动，夹球后先CLEAR，再以关节空间五次 S 曲线进入CARRY。

1. 先固定球和车的位置，运行 `test-new-arm auto ik-pick`。
2. 按“前后→左右→上下→俯仰”顺序，每次只调整一个参数并录像。
3. 单独动作能够夹球后，再运行安全追球，确认停车尺寸和球心误差。
4. 最后测试完整流程，并对比独立动作和完整流程的节奏。
5. 当前保持20 Hz；不要用提高频率掩盖姿态或停车位置问题。
6. `GAP` 应保持约300 ms，不能乘上视觉帧间隔。

完整闭环抓取失败时会尝试多组前后/高度偏移。某一组成功后，程序会把成功的
`grab_forward_offset_cm` 和 `grab_height_offset_cm` 写回配置文件，但不会覆盖
`grab_id*` 和 `carry_id*`。测试前后可用 `git diff -- config/lekiwi_pick_config.txt`
检查自动保存结果。

建议每次记录：

```text
日期：
系统：Linux / StarryOS
提交：
动作更新周期：
参数修改：
启动日志中的 ids：
启动日志中的 offset_cm：
启动日志中的 resolved：
BALL_READY 时 size/off：
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
