# LeKiwi 三轮网球机器人使用说明书

本文说明当前 `aka-rk3588` 用户态程序的功能、运行方式、配置参数、动作流程和现场
调参方法。Linux/StarryOS 启动、源码同步、编译部署和底层故障定位见
[`lekiwi_arm_debug_guide.md`](lekiwi_arm_debug_guide.md)。

本文只适用于当前三轮全向底盘、Feetech STS3215 机械臂，不适用于旧的两轮
ESP32/ZP10D 机器人。

## 1. 当前能力

完整闭环为：

```text
UVC摄像头采集
→ RKNN YOLOv8识别网球
→ 三轮底盘追球、对正和停车
→ 六自由度机械臂抓球
→ 判断是否夹球成功并自动重试
→ HSV识别红桶、靠近并停车
→ 机械臂安全接近、放球和撤离
→ 继续寻找下一颗球
```

Linux 和 StarryOS 使用同一个 AArch64 用户态程序和同一个物理 rootfs。Feetech 总线
参数统一使用 `auto`：Linux 通常回退到 `/dev/ttyACM0`，StarryOS 使用 userspace
libusb CDC。

硬件编号：

| ID | 部件 | ID | 部件 |
| --- | --- | --- | --- |
| 1 | 肩部水平旋转 | 6 | 夹爪 |
| 2 | 肩部抬升 | 7 | 左轮 |
| 3 | 肘部弯曲 | 8 | 后轮 |
| 4 | 腕部俯仰 | 9 | 右轮 |
| 5 | 腕部旋转 | | |

关键文件：

```text
build/tennis                         主程序
models/tennis.rknn                   网球检测模型
config/lekiwi_calibration.json       ID1～ID6机械臂校准
config/lekiwi_pick_config.txt        抓球、收球、放球和视觉停车参数
run_lekiwi_full.sh                   完整捡球闭环
run_bucket_place_demo.sh             找桶和放球单次演示
run_lekiwi_test.sh                   架空追球安全测试
run_vision_once.sh                   单帧视觉测试
```

## 2. 运行前检查

1. 确认电池电量、USB线、舵机总线和摄像头连接可靠。
2. 确认机械臂线束不会被关节夹住。
3. 第一次运行新参数时取出夹爪内的球，并让三个轮子离地。
4. 保证机器人、机械臂和桶周围没有人手或障碍物。
5. 随时准备按 `Ctrl-C`；异常时先停车，再关闭机械臂扭矩。

在工程目录执行只读检查：

```bash
cd /home/orangepi/robot/aka-rk3588
./build/tennis test-new-arm auto config-check
./build/tennis test-feetech auto scan
./build/tennis test-feetech auto read
./build/tennis test-new-arm auto calib-check
```

正常结果包括：

```text
config ok: pick/place trajectories are inside configured safety limits
found 9 motor(s): 1 2 3 4 5 6 7 8 9
calibration ok
```

`config-check` 不会让机械臂动作。`scan`、`read` 只通信，不发送姿态命令。

## 3. 三种运行方式

### 3.1 单帧视觉测试

不初始化底盘和机械臂：

```bash
./run_vision_once.sh
```

正常结束后生成：

```text
capture.jpg
result.jpg
```

### 3.2 找桶和放球演示

```bash
./run_bucket_place_demo.sh
```

只执行一次：

```text
保留当前夹爪位置
→ 平滑进入CARRY
→ FIND_BUCKET
→ 靠近桶并按bucket_stop_size_px停车
→ 固定PLACE_APPROACH
→ 配置指定的PLACE_RELEASE
→ 打开夹爪
→ 原路撤离并退出
```

该模式不找球、不抓球。夹爪内有球时会实际释放；首次应空载确认停车距离和动作轨迹。

### 3.3 完整闭环

```bash
./run_lekiwi_full.sh
```

完整程序会持续寻找下一颗球，直到按 `Ctrl-C` 或发生不可恢复错误。

安全追球测试使用：

```bash
./run_lekiwi_test.sh
```

它在确认追球/停车逻辑后退出，不进入抓球和找桶阶段。

## 4. 当前状态机

主要状态：

```text
CHASE_BALL
→ PICK_BALL
→ FIND_BUCKET
→ PUT_BALL
→ CHASE_BALL
```

### 4.1 追球与停车

程序选择最合适的网球检测框，先完成大方向旋转，再进行细调。只有球框尺寸和球心
偏差连续满足要求，才进入抓球。距离过近时会后退，不会在误差范围外强行抓取。

### 4.2 抓球

```text
HOME
→ 打开夹爪
→ PRE_GRAB安全接近
→ GRAB最终夹球姿态
→ 闭合夹爪并检测接触
→ CLEAR沿原路径抬升
→ CARRY平滑收臂
```

ID6出现单独的 `0x20` 过载时，仅在夹爪闭合、持球和释放场景中作为接触信号；其他
舵机错误仍会停车。抓球完成后结合夹爪位置和视觉复核判断是否真正夹住球。失败时会
尝试小范围前后/高度偏移，成功的偏移可能写回配置文件。

### 4.3 小车持球移动

只有机械臂完成 CARRY 并稳定后才允许车轮启动。当前 `carry_id1_deg=-11.3`，因此
收球时肩部水平关节会略向左转；这不是机械臂跟踪桶，而是记录的持球姿态。

### 4.4 找桶与停车

桶使用全分辨率图像进行 HSV 检测。程序先快速转向，再以低速将桶中心精调到
`bucket_center_tolerance_px` 范围内；未对正时禁止前进。程序以桶框宽、高中的较小值
作为距离指标：达到 `bucket_stop_size_px` 后停车；接近目标的最后10%自动降速；明显
过近时后退。距离和中心连续满足 `bucket_stable_frames` 帧后才进入放球。

### 4.5 放球

接近姿态与最终放球姿态完全独立：

```text
固定PLACE_APPROACH：高位、收回、越过桶沿
配置PLACE_RELEASE：严格使用place_id1_deg～place_id5_deg
```

两者之间使用五次 S 曲线平滑插值。最终姿态不会通过 IK 重新计算，也不会反向改变
接近姿态。程序检查所有关节角度、中间轨迹以及最终姿态确实低于接近姿态。

## 5. 当前稳定配置

参数文件：

```text
config/lekiwi_pick_config.txt
```

修改后退出旧进程并重新运行即可生效，不需要重新编译。角度均为校准后的度数，不是
舵机 `0～4095` 原始值。

当前从开发板同步的稳定参数：

```text
grab_id1_deg = -18.0
grab_id2_deg = 37.7
grab_id3_deg = 20.0
grab_id4_deg = 40.0
grab_id5_deg = 0.0
grab_forward_offset_cm = -0.5
grab_lateral_offset_cm = -0.5
grab_height_offset_cm = -1.0
grab_pitch_offset_deg = 0.0

carry_id1_deg = -11.3
carry_id2_deg = -18.3
carry_id3_deg = -45.0
carry_id4_deg = 51.8
carry_id5_deg = 0.1

place_id1_deg = -11.3
place_id2_deg = 20.4
place_id3_deg = -25.7
place_id4_deg = 70.0
place_id5_deg = 0.0

bucket_stop_size_px = 360
bucket_center_tolerance_px = 20
bucket_stable_frames = 3
gripper_open_delta_deg = 60.0
gripper_close_delta_deg = -60.0
carry_duration_ms = 2000
carry_settle_ms = 500
arm_speed_scale = 0.5

ball_stop_size_px = 155
ball_stop_tolerance_px = 5
ball_center_tolerance_px = 30
ball_stable_frames = 2
```

## 6. 参数含义和调整顺序

### 6.1 夹球基础姿态

`grab_id1_deg～grab_id5_deg` 是夹爪闭合瞬间的基础姿态。通常先保持不变，使用四个
偏移修正现场误差：

| 现象 | 调整 |
| --- | --- |
| 夹爪伸过球 | 减小 `grab_forward_offset_cm` |
| 夹爪够不到球 | 增大 `grab_forward_offset_cm` |
| 夹爪在球左边 | 减小 `grab_lateral_offset_cm` |
| 夹爪在球右边 | 增大 `grab_lateral_offset_cm` |
| 夹爪太低 | 增大 `grab_height_offset_cm` |
| 夹爪太高 | 减小 `grab_height_offset_cm` |
| 夹爪俯仰不合适 | 每次调整 `grab_pitch_offset_deg` 约5度 |

位置单位为厘米，每次只改 `0.5`，并且一次只改一个方向。

### 6.2 CARRY姿态

`carry_id1_deg～carry_id5_deg` 是夹球成功、车轮启动前的机械臂姿态。

- 小车移动时机械臂偏左：检查 `carry_id1_deg`。
- 收臂太慢或太快：调整 `carry_duration_ms`。
- 到位后仍晃动：适当增加 `carry_settle_ms`。

### 6.3 最终放球姿态

`place_id1_deg～place_id5_deg` 是最终严格执行的关节角度。ID2、ID3共同决定伸展和
高度，ID4决定夹爪俯仰。它们不影响固定的接近姿态。

调整顺序：

1. 先用 `bucket_stop_size_px` 调整小车与桶的距离，每次改10～20。
2. 使用 `task place-approach` 确认固定接近姿态能越过桶沿。
3. 小角度调整 ID2、ID3，使用 `task place-release` 检查最终位置。
4. 调整 ID4，使夹爪以合适角度释放。
5. 最后运行 `task place-cycle` 或桶演示。

ID3变得更负不一定让夹爪更低；机械臂是二连杆结构，必须以实际位置和
`config-check` 为准。

### 6.4 桶停车和对齐

`bucket_stop_size_px` 使用桶框较短边：

- 增大：小车更靠近桶才停车。
- 减小：小车离桶更远就停车。

不同尺寸的桶需要重新确认该值。机械臂前后位置误差较大时优先调整停车距离，不要
首先改变放球姿态。

`bucket_center_tolerance_px` 是桶中心与画面中心允许的左右偏差：减小会对得更正，
但过小会因检测抖动反复旋转。`bucket_stable_frames` 是距离和中心同时满足要求的连续
帧数。当前推荐值为 `±20 px`、连续3帧。

### 6.5 网球停车参数

| 参数 | 含义 |
| --- | --- |
| `ball_stop_size_px` | 球框宽、高较大值的目标尺寸；增大表示更靠近球 |
| `ball_stop_tolerance_px` | 目标尺寸允许误差；当前 `155±5` |
| `ball_center_tolerance_px` | 球心与画面目标中心允许的左右误差 |
| `ball_stable_frames` | 距离和左右条件连续满足多少帧才抓球 |

## 7. 机械臂分阶段命令

以下命令会产生实际动作，只有 `config-check` 例外：

```bash
./build/tennis test-new-arm auto config-check       # 不动作
./build/tennis test-new-arm auto task home          # 回HOME
./build/tennis test-new-arm auto task carry         # 到CARRY
./build/tennis test-new-arm auto task place-approach # 固定接近姿态
./build/tennis test-new-arm auto task place-release  # 接近后到最终姿态，不开夹爪
./build/tennis test-new-arm auto task place-cycle    # 完整放球，会打开夹爪
./build/tennis test-new-arm auto ik-pick             # 完整机械臂抓球
./build/tennis test-new-arm auto ik-put              # 完整机械臂放球
```

这些动作从舵机实际位置开始，不会为了演示而先强制回 HOME。重复执行同一目标时可能
几乎看不到动作，这是正常的幂等行为。

## 8. 日志判断

正常完成：

```text
done=1 failed=0
```

常见状态：

| 日志 | 含义 |
| --- | --- |
| `BALL_FORWARD/BACKWARD` | 根据球框尺寸前进或后退 |
| `BALL_READY` | 球距离和左右位置满足要求 |
| `BUCKET_FORWARD/BACKWARD` | 调整与桶的距离 |
| `BUCKET_FINE_LEFT/RIGHT` | 停止前进，以低速精细对正桶中心 |
| `BUCKET_READY ... off=... tol=20 target=360` | 距离和中心满足要求 |
| `place_approach` | 固定安全接近姿态 |
| `place_release` | 配置指定的最终放球姿态 |
| `gripper contact` | 夹爪过载或稳定残差判断为接触球 |

上层某些日志仍使用 `communication failure`，但后面的具体原因可能是配置或轨迹安全
校验失败，应以冒号后的错误为准。

## 9. 安全和异常处理

- 控制器以50ms绝对周期运行，通信耗时计入周期，不再在通信后额外等待50ms。
- S曲线按单调时钟的真实经过时间计算；普通关节命令限制为15度/秒，启动和 HOME
  保持25度/秒，夹爪限制为40度/秒。
- 调度或通信延迟时，每周期角度增量仍受限制：动作可以延长，但不会突然追赶。
- 放球下降、释放和撤离使用不同的低速 S 曲线。
- 配置越界、最终放球点没有低于接近点、轨迹扫出安全走廊时会在动作前拒绝。
- CLEAR负载过大且达不到安全高度时，程序保持底盘停止，释放球并恢复 HOME。
- `Ctrl-C` 后应确认轮子停止；需要手动移动机械臂时执行：

```bash
./build/tennis test-base auto stop
./build/tennis test-feetech auto torque-off
```

底层 USB、编译、rootfs、StarryOS 或校准故障见调试手册。

## 10. 推荐现场调试流程

```text
1. config-check、scan、read、calib-check
2. 空载测试HOME和CARRY
3. 单独运行ik-pick固定球和车的位置
4. 架空运行run_lekiwi_test.sh
5. 空载分阶段测试place-approach和place-release
6. 空载运行run_bucket_place_demo.sh
7. 有球运行桶演示
8. 最后运行run_lekiwi_full.sh
```

每次只修改一组参数并记录提交、系统、参数、日志和视频。完整闭环自动写回抓球成功
偏移后，应将开发板的稳定配置同步回仓库。
