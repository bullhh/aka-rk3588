# 用户态程序使用、测试与调参

本文只介绍 `aka-rk3588` 用户态程序本身：如何编译，在 Linux 和 StarryOS 中如何
运行，如何从只读检查逐步测试摄像头、NPU、底盘和机械臂，以及如何调整抓球和放球
参数。本文从已经进入目标操作系统并具备项目文件的状态开始。

## 1. 编译用户态程序

程序输出是 AArch64 可执行文件 `build/tennis`。项目提供统一脚本
`build_rk3588.sh`，可在 x86_64 Linux 主机交叉编译，也可在开发板 Linux 原生编译。
StarryOS 直接运行已经生成的 AArch64 文件，不在 StarryOS 中编译。

### 1.1 在 x86_64 Linux 主机交叉编译

准备 CMake、`aarch64-linux-gnu-gcc` 和 `aarch64-linux-gnu-g++`，在项目根目录执行：

```bash
./build_rk3588.sh -b Release -l WARN
```

脚本自动选择交叉工具链，成功后得到：

```text
build/tennis
```

### 1.2 在开发板 Linux 原生编译

在 AArch64 Linux 中执行相同命令：

```bash
./build_rk3588.sh -b Release -l WARN
```

脚本检测到 `aarch64` 后自动使用本机 `gcc/g++`。

日志级别在编译时确定：

| 构建方式 | 用途 |
| --- | --- |
| `Release + WARN` | 正式演示和性能测试，减少日志对 StarryOS 调度的影响 |
| `Release + INFO` | 功能调试，显示状态切换和动作步骤 |
| `Debug` | 源码级调试，不用于性能数据 |

修改日志级别后必须重新编译。程序运行还需要 `models/tennis.rknn`、两个 `config/`
配置文件以及目标系统中的 `libusb-1.0`、`libuvc`、`libturbojpeg` 和 RKNN Runtime。

## 2. 在 Linux 和 StarryOS 中运行

### 2.1 Linux

进入项目目录后，可以直接执行测试命令。完整闭环入口为：

```bash
./run_lekiwi_full.sh
```

Linux 下该脚本会先调用 `build_rk3588.sh -b Release -l INFO` 完成一次原生编译，再启动
程序。若已经专门构建了 `WARN` 版本并希望保持该日志级别，应直接执行等价主命令：

```bash
AKA_STATE_LOG_INTERVAL_MS=3000 RKNN_CORE_MASK=0 \
  ./build/tennis models/tennis.rknn auto 0 auto lekiwi
```

Feetech 参数使用 `auto`。程序先尝试 userspace libusb CDC；Linux 没有权限直接 claim
CDC 接口时会回退到 `/dev/ttyACM0`。出现以下连续日志表示回退成功，不是故障：

```text
libusb ... LIBUSB_ERROR_ACCESS
falling back to TTY /dev/ttyACM0
opened TTY /dev/ttyACM0 baud=1000000
```

若摄像头报 `uvc_open failed: Access denied`，则是进程无权访问对应 usbfs 设备节点。
应修复 Linux 的 udev 权限规则；长期使用时不要用 `sudo` 掩盖权限问题。

### 2.2 StarryOS

StarryOS 启动后进入同一项目目录，直接使用已有的 `build/tennis`：

```bash
./run_lekiwi_full.sh
```

脚本不会在 StarryOS 中重新编译，并默认设置 `RKNN_CORE_MASK=0`。Feetech `auto` 后端
应选择 userspace libusb CDC，因为 StarryOS 不依赖 `/dev/ttyACM0`。

StarryOS 运行完整程序前，应确认摄像头和 Feetech CDC 都已经枚举。正常日志包含：

```text
[FeetechBus] auto backend selected: libusb CDC ...
[UvcCapture] streaming 640x480 @ 30 fps
[INFO] Model input 640x480
[INFO] Warming up camera (skip 10 frames)...
```

以下 UVC 描述符提示通常不影响运行，以随后能否出现 `streaming` 为准：

```text
unsupported descriptor subtype VS_STILL_IMAGE_FRAME
unsupported descriptor subtype VS_COLORFORMAT
attempt to claim already-claimed interface 1
```

StarryOS 性能测试不要把程序输出重定向到板端文件。高频文件写入会引入存储和调度
负载，使帧率、启动等待和机械臂周期不再代表正常运行。应在串口前台观察并由外部终端
保存必要片段。

## 3. 从只读检查逐步测试完整功能

完整程序同时使用摄像头、NPU、底盘和机械臂。首次运行新二进制或新配置时，应从不会
动作的检查开始，再逐步开放实际动作。

### 3.1 配置、通信和校准检查

```bash
./build/tennis test-new-arm auto config-check
./build/tennis test-feetech auto scan
./build/tennis test-feetech auto read
./build/tennis test-new-arm auto calib-check
```

| 命令 | 是否动作 | 通过标准 |
| --- | --- | --- |
| `config-check` | 否 | 抓球、放球配置和规划轨迹通过安全校验 |
| `scan` | 否 | 找到 ID1～ID9 |
| `read` | 否 | 九个舵机的位置、电压和温度可以读取 |
| `calib-check` | 否 | ID1～ID6 校准数据完整且格式正确 |

首次建立 CDC 通信时，第一条命令可能偶发一次 `rx timeout` 或校验错误。立即重试一次；
第二次成功通常是设备刚建立后的瞬态。连续失败才检查供电、USB 线和接口占用。

### 3.2 摄像头和 NPU

```bash
./run_vision_once.sh
```

该命令不进入底盘和机械臂闭环。正常时完成一帧采集和推理，并生成 `capture.jpg` 与
`result.jpg`。

### 3.3 三轮底盘

先架起三个车轮，再分别执行：

```bash
./build/tennis test-base auto forward 0
./build/tennis test-base auto backward 0
./build/tennis test-base auto left 0
./build/tennis test-base auto right 0
./build/tennis test-base auto rotate-left 0
./build/tennis test-base auto rotate-right 0
./build/tennis test-base auto stop
```

每次只运行一个方向并现场观察 ID7、ID8、ID9。方向或轮序不正确时不要落地运行。

### 3.4 机械臂分阶段动作

确认工作范围内无人和障碍物后执行：

```bash
./build/tennis test-new-arm auto task home
./build/tennis test-new-arm auto task carry
./build/tennis test-new-arm auto task place-approach
./build/tennis test-new-arm auto task place-release
```

这些命令都会实际动作。`place-release` 会先到高位接近姿态，再下降到最终姿态，但不会
打开夹爪。需要测试抓球和完整放球时再执行：

```bash
./build/tennis test-new-arm auto ik-pick
./build/tennis test-new-arm auto task place-cycle
```

成功结果应包含 `done=1 failed=0`。

### 3.5 状态机演示

按风险从低到高依次使用：

```bash
./run_lekiwi_test.sh          # 架空追球和停车测试
./run_bucket_place_demo.sh    # 找桶、靠近和放球一次
./run_lekiwi_full.sh          # 持续执行完整闭环
```

首次测试必须架起底盘或空载机械臂，确认方向、停车和轨迹后再落地运行完整闭环。

## 4. 配置文件与校准文件的分工

机器人的配置分为两类文件，职责不同，修改频率也不同：

| 文件 | 职责 | 什么时候改 |
| --- | --- | --- |
| `config/lekiwi_calibration.json` | 硬件零位和机械范围 | 更换舵机、重新安装花键、机械端点变化 |
| `config/lekiwi_pick_config.txt` | 抓球/放球姿态、停车距离、速度等级 | 球夹不准、桶位置不合适、动作速度调整 |

两类配置的分工原则：如果问题是"机械臂不知道关节的物理边界"，需要校准；如果问题
是"机械臂知道边界但抓不到球"，只需调配置。不要一遇到抓取失败就重做硬件校准。

配置文件在程序启动时读取，修改后退出旧进程并重新运行即可生效，不需要重新编译。
所有姿态单位均为校准后的角度（度），不是 0～4095 的原始位置值。

## 5. 首次校准：建立零位和安全范围

校准的目的是让程序知道每个关节的安全机械范围。只在首次装配、更换舵机或
`calib-check` 失败时才需要做。

### 5.1 执行校准

先托住机械臂并清空工作区域：

```bash
./build/tennis test-new-arm auto calibrate
```

程序会关闭 ID1～ID6 扭矩并持续读取位置。操作步骤：

1. 缓慢移动一个非连续关节到两端的安全机械位置（不要撞击或通电硬顶限位）
2. ID5 为全转关节，使用完整 360° 范围
3. 全部关节记录完成后，按提示写入 `config/lekiwi_calibration.json`

校准结束时扭矩仍处于关闭状态。

### 5.2 验证校准

```bash
./build/tennis test-new-arm auto calib-check   # 检查校准文件格式和完整性
./build/tennis test-new-arm auto config-check   # 检查配置和轨迹安全
./build/tennis test-new-arm auto task home      # 回 HOME 姿态（会动作！）
```

执行 `task home` 前应先架起底盘并托住机械臂——这是校准后第一次实际动作。

## 6. 日常调参：先停车，后落点

夹不到球可能来自两种完全不同的误差：小车停错了位置，或者机械臂相对于车体的落点
不对。**必须先让停车位置可重复，再调整机械臂落点**。同时修改两者会相互掩盖问题。

### 6.1 调整球停车参数

观察进入 `BALL_READY` 前的 `size`、`off` 和 `ready` 日志输出：

| 参数 | 含义 | 调整方法 |
| --- | --- | --- |
| `ball_stop_size_px` | 球框宽高较大值的目标 | 停太远则增大，停太近则减小，每次 5～10 |
| `ball_stop_tolerance_px` | 允许的尺寸误差范围 | 越小停车越一致；太小会前后反复，每次改 1～2 |
| `ball_center_tolerance_px` | 球心左右允许偏差（像素） | 偏差过大则减小，每次 5 |
| `ball_stable_frames` | 距离和中心需连续满足的帧数 | 偶发误触发时增加 1 |

程序会根据球框增长速度预测制动，并在停车后低速前进/后退修正。因此高速档和低速档
可以使用同一个目标尺寸。不要靠放大 tolerance 来掩盖制动过冲——那会把停车误差传递
给机械臂。

### 6.2 调整机械臂抓球落点

停车连续 3～5 次稳定后，固定小车和球的位置，单独测试抓取：

```bash
./build/tennis test-new-arm auto ik-pick
```

优先使用四个小范围偏移参数来修正落点：

| 现象 | 参数 | 修改方向 |
| --- | --- | --- |
| 夹爪伸过球 | `grab_forward_offset_cm` | 减小 0.5 |
| 夹爪够不到球 | `grab_forward_offset_cm` | 增大 0.5 |
| 夹爪落在球左侧 | `grab_lateral_offset_cm` | 减小 0.5 |
| 夹爪落在球右侧 | `grab_lateral_offset_cm` | 增大 0.5 |
| 夹爪太低 | `grab_height_offset_cm` | 增大 0.5 |
| 夹爪太高 | `grab_height_offset_cm` | 减小 0.5 |
| 位置正确但俯仰不合适 | `grab_pitch_offset_deg` | 每次 ±5 度试一个方向 |

> 只有小范围偏移无法得到自然姿态时，才直接修改 `grab_id1_deg` ～ `grab_id5_deg`
> 的绝对角度值。ID6 夹爪力度由 `gripper_open_delta_deg` / `gripper_close_delta_deg`
> 控制，调试落点时保持不变。

### 6.3 调整放球参数

放球参数的分工：

- `bucket_stop_size_px`：增大则更靠近桶，减小则离桶更远，每次改 10～20
- `bucket_center_tolerance_px`：桶中心允许偏差，过小会左右反复
- `bucket_stable_frames`：距离和中心需连续满足多少帧才触发放球
- `place_id1_deg` ～ `place_id5_deg`：最终下降和开爪释放姿态（不影响高位接近姿态）

调试顺序：先调桶停车距离 → 用 `place-approach` 确认能越过桶沿 → 用 `place-release`
检查最终释放位置 → 最后才用会实际开夹爪的 `place-cycle`。

### 6.4 调整收臂姿态和速度

`carry_id1_deg` ～ `carry_id5_deg` 是夹住球并抬离地面后、车轮启动前的收臂姿态。
如果移动中机械臂持续偏左或偏右，首先检查 `carry_id1_deg`（肩部水平旋转），而不是
修改视觉中心。

`motion_speed_level` 是统一的速度入口：

| 等级 | 用途 | 机械臂上限 | 夹爪上限 | 接近球远/近速 | 接近桶远/近速 |
| --- | --- | --- | --- | --- | --- |
| 1 | 首次调试，最保守 | 15°/s | 30°/s | 20 / 8 | 20 / 8 |
| 2 | 稳定运行 | 22.5°/s | 40°/s | 30 / 10 | 30 / 10 |
| 3 | 快速运行 | 30°/s | 45°/s | 40 / 15 | 40 / 15 |
| 4 | 极速演示 | 50°/s | 60°/s | 65 / 20 | 70 / 25 |

HOME 速度在 1～4 档中分别为 15、20、25、25°/s；极速档不会继续提高 HOME 速度，
保证从未知上电位置回位时仍使用安全上限。

## 7. 机械臂调试命令参考

以下是常用的机械臂调试命令。带 "会动作" 标记的命令会实际驱动舵机：

| 命令 | 说明 | 是否动作 |
| --- | --- | --- |
| `./build/tennis test-new-arm auto config-check` | 检查配置和轨迹安全 | 否 |
| `./build/tennis test-new-arm auto task home` | 回到 HOME 姿态 | 会动作 |
| `./build/tennis test-new-arm auto task carry` | 进入收臂姿态 | 会动作 |
| `./build/tennis test-new-arm auto task place-approach` | 高位越过桶沿的姿态 | 会动作 |
| `./build/tennis test-new-arm auto task place-release` | 下降并释放姿态 | 会动作 |
| `./build/tennis test-new-arm auto task place-cycle` | 完整放球循环（会开爪） | 会动作 |
| `./build/tennis test-new-arm auto ik-pick` | 逆运动学抓取序列 | 会动作 |

## 8. 使用日志定位问题

### 8.1 性能日志

程序每 10 秒输出一次性能摘要：

```text
[PERF] ... camera=... effective=... busy=...
[PERF] stage_ms wait=... jpeg_decode=...
[PERF] stage_ms input=... run=... control=...
```

| 指标 | 含义 |
| --- | --- |
| `effective` | 完整处理并产生控制结果的真实帧率——这是最重要的指标 |
| `camera` | 主循环成功取得帧的速率，不等于传感器曝光率 |
| `busy` | 扣除取帧等待后的理论处理能力 |
| `wait` | 等待 libuvc 交付下一张完整图片的耗时 |
| `run` | `rknn_run()` NPU 推理阶段耗时 |
| `control` | 本轮底盘/状态控制耗时 |

位置状态日志默认限频输出。需要瞬时逐帧日志时可以设置环境变量
`AKA_STATE_LOG_INTERVAL_MS=0`，但这会显著扰动 StarryOS 性能，仅用于短时定位。

### 8.2 常见现象判断

| 现象 | 判断和处理 |
| --- | --- |
| Linux libusb `ACCESS` 后回退 `/dev/ttyACM0` | Feetech 正常回退，不是故障 |
| `uvc_open failed: Access denied` | Linux usbfs 权限/udev 规则缺失 |
| StarryOS 找不到 CDC 且无 `/dev/ttyACM0` | USB 未直通/枚举完成，或客户机设备配置错误 |
| 首次一条 `rx timeout`，重试成功 | CDC 刚建立的瞬态；连续失败才查供电和线缆 |
| `LIBUSB_ERROR_BUSY` | 旧进程仍占用接口，确认进程退出和接口释放 |
| ID6 `0x20` 出现在闭合/持球阶段 | 可作为夹爪接触信号；其他关节同类错误仍是故障 |
| `unsafe place trajectory` | 配置或轨迹校验失败，不是通信错误 |
| 重复执行姿态但看不到动作 | 已接近目标角度，属于正常幂等行为 |

问题报告至少应包含：系统环境、程序提交版本、模型文件、配置文件、完整运行命令、
失败前后的日志以及现场视频。
