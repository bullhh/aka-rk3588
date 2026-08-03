# 三足捡球机器人

`aka-rk3588` 是运行在 RK3588 / Orange Pi 5 Plus 上的用户态控制程序。它面向一台由
三轮全向底盘、UVC 摄像头和 Feetech 六关节机械臂组成的捡球机器人，将视觉感知、
运动控制和任务规划串联为完整的自主捡球流程：寻找网球、靠近并抓取，再寻找红桶并
放入球，循环往复。

同一份 AArch64 用户态程序、模型和动作配置可用于 Linux、StarryOS，以及二者作为
AxVisor 客户机的环境。

## 1. 功能

### 1.1 任务流程

程序启动后循环执行以下过程：

```text
发现网球 → 追球和对正 → 精确停车 → 机械臂抓球
        → 抓取复核/自动重试 → 搜索红桶 → 对齐并靠近
        → 高位越过桶沿 → 下降放球 → 撤离 → 继续找球
```

**发现网球**：摄像头提供连续画面，NPU（RK3588 内置神经网络处理器）运行 YOLO
检测模型，实时识别网球在画面中的位置和大小。

**追球和对正**：根据球的位置和距离，底盘进行差速转向和前进/后退，使球保持在画面
中心且距离合适。

**精确停车**：当球的位置和大小连续多帧满足条件后，程序控制底盘制动并确认停车。

**机械臂抓球**：底盘停止后，六关节机械臂依次执行打开夹爪、下降抓取、闭合夹爪、
抬升收臂的动作序列。

**抓取复核**：夹爪闭合后，程序结合夹爪反馈和摄像头画面判断是否真的抓到了球。失败
时会进行小范围重试；多次失败则安全释放并重新找球。

**搜索红桶**：抓球成功后，程序切换到 HSV 颜色检测模式，寻找红色桶的位置。

**对齐放球**：控制机器人靠近并对齐桶口，机械臂先抬到安全高位越过桶沿，再下降到
配置指定的释放位置，打开夹爪放球。

**撤离并继续**：放球完成后机械臂收回，状态机回到找球阶段，开始下一轮。

整个过程完全自主运行，直到按下 `Ctrl-C` 或出现不可恢复的错误。

### 1.2 硬件组成

| 组件 | 型号/规格 | 作用 |
| --- | --- | --- |
| 主控 | RK3588 / Orange Pi 5 Plus | 运行用户态程序，调度 NPU 推理和所有外设 |
| 摄像头 | UVC，640×480 MJPEG，30 FPS | 提供实时画面 |
| 底盘 | ID7、ID8、ID9 三轮全向底盘 | 驱动机器人移动和转向 |
| 机械臂 | ID1～ID6 Feetech 舵机 | 执行抓球和放球动作 |
| 通信 | TTY 或 userspace libusb CDC | 与舵机通信 |

舵机 ID 定义：

| ID | 部件 | ID | 部件 |
| --- | --- | --- | --- |
| 1 | 肩部水平旋转 | 6 | 夹爪 |
| 2 | 肩部抬升 | 7 | 左轮 |
| 3 | 肘部弯曲 | 8 | 后轮 |
| 4 | 腕部俯仰 | 9 | 右轮 |
| 5 | 腕部旋转 | | |

### 1.3 软件模块

程序的核心是一条"图像 → 决策 → 动作"的处理链路：

- **视觉采集**：通过 libuvc 从 UVC 摄像头获取 640×480 MJPEG 帧，使用 libjpeg-turbo
  解码为 RGB。
- **网球检测**：将解码后的图像送入 NPU，运行 YOLOv8 RKNN 模型（640×480、INT8
  量化），输出球的位置和边界框大小。
- **桶检测**：对图像做 HSV 颜色空间转换，按红色区域定位桶的位置。
- **底盘控制**：根据目标位置和距离计算三轮速度，实现前进、后退、差速转向和制动。
- **机械臂控制**：管理 ID1～ID6 舵机的轨迹规划，使用限速 S 曲线保证平滑安全运动。
- **任务状态机**：协调上述所有模块，在找球、追球、抓球、找桶、放球等状态之间切换。

### 1.4 运行环境

同一份 AArch64 用户态程序可运行在四种环境中：

- 原生 Linux（Orange Pi 5 Plus 直接启动 Linux）
- 原生 StarryOS
- AxVisor Linux 客户机（单核 vCPU）
- AxVisor StarryOS 客户机（单核 vCPU）

摄像头、NPU、底盘、机械臂和完整任务状态机已在上述四种环境中完成验证。

### 1.5 关键文件

```text
build/tennis                         主程序
models/tennis.rknn                   网球检测模型
config/lekiwi_calibration.json       机械臂关节校准文件
config/lekiwi_pick_config.txt        抓球、放球、速度和停车配置
run_lekiwi_full.sh                   完整闭环入口脚本
run_lekiwi_test.sh                   架空追球测试脚本
run_bucket_place_demo.sh             找桶和放球单次演示脚本
run_vision_once.sh                   单帧视觉测试脚本
```

## 2. 原理

### 2.1 图像处理链路

```text
UVC 摄像头（640×480 MJPEG，30 FPS）
  ↓
libusb / libuvc 后台传输与 USB 事件处理
  ↓ 交付下一张完整图片
前台主循环同步取最新帧
  ↓
libjpeg-turbo 解码为 RGB
  ↓
构造 640×480 INT8 RKNN 输入
  ↓
RK3588 NPU 推理 + YOLO 后处理
  ↓
得到球/桶的位置与尺寸
  ↓
计算三轮底盘命令 或 推进机械臂任务状态
```

数据流起点在 `capture/uvc_capture.cpp`，经 `detect/detect.cpp` 推理，由
`robot/omni_base.cpp`（底盘）或 `robot/lekiwi_task_controller.cpp`（机械臂）执行。
顶层状态机位于 `tennis.cpp`，决定当前帧的结果用于追球、抓球还是放球。

模型输入尺寸为 640×480，与摄像头输出完全匹配。相比通用的 640×640 正方形输入，
消除了上下各 80 像素的无效补边：输入元素从 1,228,800 减少到 921,600（减少
25%），候选点从 8,400 减少到 6,300。NPU 推理时间在 Linux 上从约 42 ms 降至约
16 ms。

### 2.2 摄像头与主线程的配合

程序使用 libuvc 的同步流接口。`uvc_stream_get_frame()` 阻塞等待下一张完整图片，
但摄像头并不等待主线程——它持续按 30 FPS 曝光，USB 控制器和 libusb 在后台持续
接收数据。实际结构是：

```text
后台：摄像头、USB 控制器和 libusb 持续接收数据
前台：主循环等待完整帧 → 串行执行解码、推理和控制 → 再取下一帧
```

假设前台处理一帧需要约 70 ms，摄像头仍每 33 ms 产生一帧：

```text
  0 ms：摄像头产生第 1 帧，程序取走并开始处理
 33 ms：摄像头产生第 2 帧，程序仍在处理第 1 帧
 66 ms：摄像头产生第 3 帧
 70 ms：程序处理完成，下一轮取到当前最新的完整帧（第 3 帧）
```

程序跳过已过时的第 2 帧直接取第 3 帧，避免检测结果因帧排队而越来越滞后。

早期方案使用 libuvc 异步回调 + 独立分发线程。在 StarryOS 调度压力较大时，分发
线程可能来不及交付已完成帧。改为前台直接同步等待后，减少了线程切换开销，同时
关闭了自动曝光优先导致的动态降帧，摄像头请求稳定在 30 FPS。

### 2.3 串行处理

每轮处理顺序固定：

1. `getFrame()` 等待并复制一张 MJPEG
2. 读取 JPEG 头并使用 libjpeg-turbo 解码为 RGB
3. 将 640×480 RGB 送入 RKNN
4. `rknn_run()` 等待 NPU 完成推理
5. 取得输出并执行 YOLO 后处理
6. 根据当前状态计算并发送底盘控制
7. 进入下一轮

程序不是"采集线程、推理线程、控制线程"组成的流水线。多核操作系统可以把 USB
事件、内核任务和用户进程调度到不同 CPU，但应用本身不会把一张图的解码和下一张图
的推理并行化。

串行模型的优势是状态、图像和控制结果天然一一对应，不会出现帧间错位。代价是
StarryOS 中 NPU 等待期间的 CPU 忙轮询会挤压 USB 事件处理，导致下一轮仍需等待
完整帧交付——这是 StarryOS 约 15 FPS 与 Linux 约 30 FPS 之间差距的主要来源。

### 2.4 状态机

```text
CHASE_BALL（追球）
  ├─ 未发现球：旋转搜索
  ├─ 偏左/偏右：差速转向修正
  ├─ 太远/太近：前进或后退
  └─ 距离和中心连续稳定 → 进入 PICK_BALL

PICK_BALL（抓球）
  ├─ 暂停 UVC 传输
  ├─ 打开夹爪 → PRE_GRAB → GRAB → 闭合
  ├─ CLEAR 抬球 → CARRY 收臂
  ├─ 恢复 UVC 并视觉复核
  ├─ 成功 → 进入 FIND_BUCKET
  └─ 失败 → 小范围重试或回到 CHASE_BALL

FIND_BUCKET / APPROACH_BUCKET（找桶和靠近）
  ├─ HSV 颜色空间寻找红色桶
  ├─ 左右对齐、预测减速和停车
  └─ 距离与中心连续稳定 → 进入 PUT_BALL

PUT_BALL（放球）
  ├─ 暂停 UVC 传输
  ├─ PLACE_APPROACH：抬到安全高位越过桶沿
  ├─ PLACE_RELEASE：下降到释放位置
  ├─ 打开夹爪并撤离
  └─ 恢复 UVC → 回到 CHASE_BALL
```

#### 视觉伺服

追球阶段使用连续视觉反馈控制底盘：

- **距离判断**：球框大小反映距离——框越大说明越近。
- **方向修正**：球心偏离画面中心的像素数决定转向方向和幅度。
- **平滑停车**：程序根据球框增长速度预测制动，进入减速区后从远速连续降到近速。
  停车后用低速前进/后退微调，配合稳定帧确认（连续 N 帧满足条件才认为就绪），
  减少"前进—停车—再前进"的抖动。
- **左右微调**：接近目标时，左右偏差进入微调区后保持向前并通过左右轮差速修正；
  中心误差退出更小的滞回窗口后才恢复直行，避免检测值在阈值附近波动时反复出现
  "前进、停下原地转、再前进"。

#### 抓取复核与失败恢复

夹爪闭合后，程序通过两步确认是否抓取成功：

1. **夹爪反馈**：ID6 在闭合阶段的接触/过载（`0x20` 错误码）可作为"碰到球"的信号。
2. **视觉复核**：恢复摄像头后重新检测球是否还在地面上可见。

只有夹爪反馈为"持有"且视觉确认球已从画面消失时，才判定抓取成功。

失败后的恢复策略：保持底盘停止，使用预设的小范围偏移参数组依次重试。达到重试
上限后安全释放夹爪并回到追球状态。如果启动时夹爪中已有球，程序会先主动释放，
避免带负载强行回 HOME。

#### 安全放球

放球由两个互相独立的姿态组成：

- **`PLACE_APPROACH`**：固定高位姿态，用于安全越过桶沿。
- **`place_id1_deg` ～ `place_id5_deg`**：配置文件指定的最终下降和释放姿态（不影响
  高位接近姿态）。

程序在动作前检查关节范围是否合法、插值轨迹是否安全、最终释放位置是否低于接近
位置，防止从高位下降时撞桶。

## 3. 使用

### 3.1 准备工作

目标文件系统需要以下内容：

- 运行库：`libusb-1.0`、`libuvc`、`libturbojpeg` 和 RKNN Runtime
- 可执行文件：`build/tennis`
- 模型文件：`models/tennis.rknn`
- 配置文件：`config/lekiwi_calibration.json` 和 `config/lekiwi_pick_config.txt`
- 本仓库中的运行脚本

部署前检查机器人供电、摄像头、USB 集线器和 Feetech 总线。

### 3.2 编译

项目提供统一的编译脚本 `build_rk3588.sh`。日志级别是编译选项（`WARN`、`INFO`、
`Debug`），修改后需要重新编译。

**在 x86_64 Linux 上交叉编译**（安装 CMake 和 AArch64 GNU 工具链后）：

```bash
./build_rk3588.sh -b Release -l WARN
```

脚本自动选择 `aarch64-linux-gnu-gcc/g++`，输出 `build/tennis`。日常演示推荐
`Release + WARN`：保留警告和错误，同时减少串口格式化输出对 StarryOS 调度的
影响。定位功能逻辑时可改为 `-l INFO`，只有源码级调试才使用 `Debug`。

**在开发板上原生编译**：同一脚本在 AArch64 Linux 上自动使用本机 `gcc/g++`。StarryOS
不进行编译，直接使用已编译好的二进制。采用共享根文件系统时，可先启动 Linux 完成
原生编译，再启动 StarryOS 使用同一份工程目录。

### 3.3 部署

推荐同步整个仓库而非只复制 `tennis` 文件，因为程序运行依赖模型、配置和脚本：

```bash
rsync -a --delete \
  --exclude build/ \
  ./ <user>@<board-host>:~/robot/aka-rk3588/
```

然后在开发板 Linux 上编译。若使用交叉编译，也可以连同 `build/tennis` 一起复制：

```bash
scp build/tennis <user>@<board-host>:~/robot/aka-rk3588/build/tennis
```

> **注意**：`--delete` 会删除目标目录中仓库不包含的文件。开发板上存在实机专用配置
> 时应去掉该选项并先备份配置。

部署后确认文件就绪：

```bash
test -x build/tennis
test -s models/tennis.rknn
./build/tennis test-new-arm auto config-check
```

### 3.4 首次校准

机械臂首次使用、更换舵机或重新安装花键后，必须校准。校准的目的是让程序知道每个
关节的安全机械范围——没有有效校准文件，机械臂命令和完整闭环会拒绝运行。

先托住机械臂并清空工作区域：

```bash
./build/tennis test-new-arm auto calibrate
```

程序关闭 ID1～ID6 扭矩并持续读取位置。缓慢移动每个非连续关节到两端的安全机械
位置（不要撞击或通电硬顶限位）；ID5 为全转关节，使用完整 360° 范围。全部记录
完成后按提示写入 `config/lekiwi_calibration.json`。校准结束时扭矩仍处于关闭状态。

随后验证校准文件并检查配置安全：

```bash
./build/tennis test-new-arm auto calib-check   # 检查校准文件格式和完整性
./build/tennis test-new-arm auto config-check   # 检查配置和轨迹安全
./build/tennis test-new-arm auto task home      # 回 HOME 姿态（会动作）
```

执行 `task home` 前应先架起底盘并托住机械臂——这是校准后第一次实际动作。

> 配置文件中所有姿态单位均为校准后的角度（度），不是 0～4095 的原始位置值。
> 修改配置文件后退出旧进程并重新运行即可生效，不需要重新编译。

### 3.5 静态检查

完整闭环同时使用摄像头、NPU、底盘和机械臂，不适合直接用来定位初始化问题。校准
完成后，按下列顺序逐层执行，前一步失败就停下来排查：

| 顺序 | 命令 | 是否动作 | 通过标准 |
| --- | --- | --- | --- |
| 1 | `./build/tennis test-new-arm auto config-check` | 否 | 抓球和放球配置、轨迹安全校验通过 |
| 2 | `./build/tennis test-feetech auto scan` | 否 | 找到 ID1～ID9 |
| 3 | `./build/tennis test-feetech auto read` | 否 | 九个舵机的位置、电压和温度可读 |
| 4 | `./build/tennis test-new-arm auto calib-check` | 否 | 校准文件格式正确且 ID1～ID6 完整 |

这四步都不涉及实际动作，可在任何时候安全执行。

首次建立 CDC 通信时，第一条命令可能偶发 `rx timeout` 或校验错误。立即重试一次；
第二次成功通常只是设备刚建立后的瞬态。连续失败才需要检查供电、线缆、USB 占用
和权限。

### 3.6 单项演示到完整闭环

静态检查全部通过后，按以下顺序逐步进入实际动作：

**单帧视觉**：

```bash
./run_vision_once.sh
```

确认摄像头、JPEG 解码、RKNN 模型和后处理链路正常。执行后生成 `capture.jpg` 和
`result.jpg`，不驱动底盘或机械臂。

**架空追球**：

```bash
./run_lekiwi_test.sh
```

必须先架起三个车轮。验证找球、方向判断、远近速度、减速和停车逻辑。首次底盘方向
测试不应在地面进行。

**找桶和放球演示**：

```bash
./run_bucket_place_demo.sh
```

执行一次找桶、靠近、放球和退出，不经过抓球阶段。首次应空载（不夹球）验证桶停车
距离和机械臂是否能越过桶沿。

**完整闭环**：

```bash
./run_lekiwi_full.sh
```

程序持续循环"追球—抓球—找桶—放球"，直到 `Ctrl-C` 或不可恢复错误。Linux 下该脚本
默认执行一次 `Release + INFO` 原生构建；StarryOS 直接使用已有二进制。正式性能
测试如需 `WARN` 级别，应先显式构建，然后直接运行：

```bash
AKA_STATE_LOG_INTERVAL_MS=3000 RKNN_CORE_MASK=0 \
  ./build/tennis models/tennis.rknn auto 0 auto lekiwi
```

### 3.7 判断正常启动

启动阶段的关键日志：

```text
[FeetechArm] torque enabled at current positions; no startup jump
[UvcCapture] streaming 640x480 @ 30 fps
[INFO] Model input 640x480
[INFO] Warming up camera (skip 10 frames)...
```

以下 libuvc 提示通常不是故障，以随后是否成功开始 streaming 为准：

```text
unsupported descriptor subtype VS_STILL_IMAGE_FRAME
unsupported descriptor subtype VS_COLORFORMAT
attempt to claim already-claimed interface 1
```

运行中的主要状态会交替出现 `BALL_FORWARD/BACKWARD/READY`、`PICK_BALL`、
`BUCKET_FORWARD/BACKWARD/READY` 和 `PUT_BALL`。机械臂步骤完成时应看到
`done=1 failed=0`。

### 3.8 安全停止

先按 `Ctrl-C`。若需单独确保底盘停止或手动释放机械臂扭矩：

```bash
./build/tennis test-base auto stop
./build/tennis test-feetech auto torque-off
```

Linux 下出现 `uvc_open failed: Access denied` 表示进程无权读写 USB 设备节点，
不是摄像头损坏。应为摄像头 VID/PID 配置持久 udev 权限规则。`sudo` 只适合临时
确认，不应作为长期运行方案。

### 3.9 调参

校准文件只在硬件变化时修改。日常抓取不准、停车位置偏差、动作速度不合适，全部
通过 `config/lekiwi_pick_config.txt` 调整。

#### 球停车参数

观察进入 `BALL_READY` 前的 `size`、`off` 和 `ready` 日志输出：

| 参数 | 含义 | 调整方法 |
| --- | --- | --- |
| `ball_stop_size_px` | 球框宽高较大值的目标 | 停太远则增大，停太近则减小，每次 5～10 |
| `ball_stop_tolerance_px` | 允许的尺寸误差范围 | 越小停车越一致；太小会前后反复，每次改 1～2 |
| `ball_center_tolerance_px` | 球心左右允许偏差（像素） | 偏差过大则减小，每次 5 |
| `ball_stable_frames` | 距离和中心需连续满足的帧数 | 偶发误触发时增加 1 |

程序根据球框增长速度预测制动，并在停车后低速修正。不要靠放大 tolerance 来掩盖
制动过冲——那样会把停车误差传递给机械臂。

#### 抓球落点

**先让停车位置可重复，再调整机械臂落点。** 同时修改两者会相互掩盖问题。

停车连续 3～5 次稳定后，固定小车和球，用 `ik-pick` 单独测试抓取。优先使用小范围
偏移参数（单位 cm，每次改 0.5）：

| 现象 | 参数 | 修改方向 |
| --- | --- | --- |
| 夹爪伸过球 | `grab_forward_offset_cm` | 减小 0.5 |
| 夹爪够不到球 | `grab_forward_offset_cm` | 增大 0.5 |
| 夹爪偏左/偏右 | `grab_lateral_offset_cm` | 减小/增大 0.5 |
| 夹爪太低/太高 | `grab_height_offset_cm` | 增大/减小 0.5 |
| 俯仰不合适 | `grab_pitch_offset_deg` | 每次 ±5 度 |

小范围偏移无法得到自然姿态时，才直接修改 `grab_id1_deg` ～ `grab_id5_deg`。

#### 放球参数

调试顺序：先调桶停车距离 → 用 `place-approach` 确认越过桶沿 → 用 `place-release`
检查释放位置 → 最后用会开爪的 `place-cycle`。

| 参数 | 含义 | 调整方法 |
| --- | --- | --- |
| `bucket_stop_size_px` | 桶靠近目标 | 增大更靠近，减小更远，每次 10～20 |
| `bucket_center_tolerance_px` | 桶中心允许偏差 | 过小会左右反复 |
| `bucket_stable_frames` | 连续满足帧数 | — |
| `place_id1_deg` ～ `place_id5_deg` | 最终释放姿态 | 不影响高位接近姿态 |

#### 速度和收臂

`carry_id1_deg` ～ `carry_id5_deg` 是夹球抬离地面后的收臂姿态。移动中机械臂
持续偏左或偏右，首先检查 `carry_id1_deg`（肩部水平旋转）。

`motion_speed_level` 统一控制机械臂、夹爪和底盘速度：1 首次调试（最保守），2
稳定运行，3 快速运行，4 极速演示。等级 4 对应机械臂 50°/s、夹爪 60°/s，
接近球远/近速 65/20，接近桶远/近速 70/25。从 HOME 回初始姿态始终限制为 25°/s。

#### 常用机械臂命令

| 命令 | 说明 | 是否动作 |
| --- | --- | --- |
| `./build/tennis test-new-arm auto config-check` | 检查配置和轨迹安全 | 否 |
| `./build/tennis test-new-arm auto task home` | 回到 HOME 姿态 | 会动作 |
| `./build/tennis test-new-arm auto task carry` | 进入收臂姿态 | 会动作 |
| `./build/tennis test-new-arm auto task place-approach` | 高位越过桶沿的姿态 | 会动作 |
| `./build/tennis test-new-arm auto task place-release` | 下降并释放姿态 | 会动作 |
| `./build/tennis test-new-arm auto task place-cycle` | 完整放球循环（会开爪） | 会动作 |
| `./build/tennis test-new-arm auto ik-pick` | 逆运动学抓取序列 | 会动作 |

### 3.10 日志与故障排查

#### 性能日志

程序每 10 秒输出一次性能摘要：

```text
[PERF] ... camera=... effective=... busy=...
[PERF] stage_ms wait=... jpeg_decode=...
[PERF] stage_ms input=... run=... control=...
```

| 指标 | 含义 |
| --- | --- |
| `effective` | 完整处理并产生控制结果的真实帧率 |
| `camera` | 主循环成功取得帧的速率，不等于传感器曝光率 |
| `busy` | 扣除取帧等待后的理论处理能力 |
| `wait` | 等待 libuvc 交付下一张完整图片的耗时 |
| `run` | `rknn_run()` NPU 推理阶段耗时 |
| `control` | 本轮底盘/状态控制耗时 |

位置状态日志默认限频。需要瞬时逐帧日志时可设 `AKA_STATE_LOG_INTERVAL_MS=0`，
但这会显著扰动 StarryOS 性能，仅用于短时定位。

#### StarryOS 日志采集

正式测量时应在串口或启动工具前台观察输出，由主机终端保存。不要使用板端文件
重定向（`> run.log 2>&1`）——高频日志写入板端文件系统会增加存储中断和调度压力，
与 UVC、NPU 和舵机控制竞争，得到的启动时间和帧率不再代表正常运行状态。

#### 常见现象

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

## 4. 架构

### 4.1 三层安全保护

机械臂控制从硬件边界到软件轨迹逐层约束：

1. **`lekiwi_calibration.json`（硬件层）**：将舵机原始位置（0～4095）换算为关节
   角度，并限定每个关节的机械安全范围。超出范围的角度命令会被拒绝。
2. **`lekiwi_pick_config.txt`（姿态层）**：定义目标姿态、视觉停车参数和速度等级。
   配置中的角度值必须在安全范围内。
3. **任务控制器（轨迹层）**：执行前检查轨迹端点和完整插值路径。不安全的轨迹在
   `config-check` 阶段就会被拒绝。

### 4.2 上电安全

上电时程序先读取 ID1～ID6 的当前位置，在当前位置开启扭矩，再限速移动到目标姿态。
这避免了上电瞬间关节突然跳回预设角度。

### 4.3 S 曲线速度控制

控制周期目标为 50 ms（20 Hz），使用单调时钟维持绝对周期。S 曲线按真实经过时间
计算速度，而不是"每循环固定前进一步"。如果通信或调度变慢，动作会延长但不会在
下一周期突然追赶并产生大角度跳变。速度等级 1～4 统一控制机械臂、夹爪和底盘的
远近速度，从 HOME 回初始姿态仍限制为 25°/s。

### 4.4 机械臂动作期间暂停 UVC

抓球和放球是阻塞式任务，这段时间状态机不使用视觉帧。摄像头与 Feetech CDC 共享
USB 路径，继续传输只会增加 USB 事件和调度压力。程序在进入 `PICK_BALL` 和
`PUT_BALL` 前暂停 UVC stream，动作结束后使用原设备句柄重新建立 stream handle。
这不是重新打开摄像头，不会重新枚举设备或再次跳过 10 帧预热。

暂停/恢复各需数百毫秒，但换来的收益明显：StarryOS 机械臂最坏控制周期从约
213 ms 降至约 50 ms，接近 20 Hz 目标。

### 4.5 跨平台共用代码

Feetech 通信使用 `auto` 后端时，程序自动选择：

1. 优先尝试 **userspace libusb CDC**：通过 libusb 枚举 CDC ACM 设备，claim 控制
   和数据接口，用 bulk IN/OUT 传输 Feetech 协议包。
2. 回退到 **`/dev/ttyACM0` TTY**：直接打开串口设备。

StarryOS 不依赖 `/dev/ttyACM0` 设备节点，可直接操作 CDC ACM 接口。CDC 实现会
消费单播写入 ACK、清理过期状态包并在退出时释放接口，因此 `scan`、`read` 和完整
程序可以跨进程连续执行。

### 4.6 多核环境

用户程序没有创建固定的"采集核"和"推理核"，也未默认绑核。前台视觉链保持串行，
多核主要帮助系统并行处理 USB 中断、文件系统和其他后台任务。

曾经尝试过 CPU 绑核实验：部分视觉样本帧率有所提高，但机械臂最坏控制周期明显变长，
因此没有作为默认方案保留。后续若引入真正的生产者/消费者流水线，必须同时验证帧龄、
控制延迟、USB 公平性和机械臂 20 Hz 控制周期——不能只看 FPS。

## 5. 性能

### 5.1 测量方法

以下数据在同一台 RK3588 三轮机器人上复测。四种环境共用：

- Release + Warn 编译的 `build/tennis`
- 同一个 640×480 ReLU INT8 RKNN 模型
- UVC 640×480 MJPEG @ 30 FPS
- `RKNN_CORE_MASK=0`（NPU 单核模式）
- 状态日志间隔 3000 ms，性能统计窗口 10 s
- 底盘架起，摄像头视野中有球，执行正式追球路径

每组先排除初始化/预热影响的首个窗口，再取完整 10 s 稳态窗口。Linux 各取 3 个窗口，
StarryOS 各取 2 个窗口并求平均。日志通过串口或终端前台观察，未在板端实时重定向。

这些是追球稳态吞吐数据。测试包含真实摄像头、JPEG、NPU、后处理、底盘控制和舵机
初始化，但因底盘架起且不要求真实抓球，不能用来计算抓取成功率或单轮耗时。

### 5.2 结果

四种环境的追球稳态吞吐如下。`effective` 是一秒内完整完成"取帧→解码→推理→控制"
的帧数，是衡量实际处理能力的关键指标。

| 环境 | 处理帧率 | 单帧耗时 | 取帧等待 | JPEG解码 | NPU推理 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 原生 Linux | 30.0 FPS | 33.4 ms | 16.1 ms | 4.5 ms | 12.0 ms |
| AxVisor Linux 客户机（单核） | 30.0 FPS | 33.4 ms | 15.7 ms | 4.8 ms | 12.1 ms |
| 原生 StarryOS | 15.1 FPS | 62.9 ms | 30.5 ms | 8.5 ms | 16.4 ms |
| AxVisor StarryOS 客户机（单核） | 15.1 FPS | 62.9 ms | 30.5 ms | 8.5 ms | 16.4 ms |

Linux 环境下（原生和客户机），处理帧率已达到摄像头 30 FPS 的上限，NPU 不再是
瓶颈。单核 AxVisor 客户机未引入可测量的性能开销。

StarryOS 环境下（原生和客户机），处理帧率约为 15～16 FPS，约为 Linux 的一半。
各处理环节均慢于 Linux，但差异最大的环节是取帧等待（约 29～30 ms，Linux 约
16 ms），占总耗时差距的一半以上。这是因为 StarryOS 的 `rknn_run()` 路径含 CPU
忙轮询，NPU 工作期间没有充分让出 CPU 给 USB 事件处理，导致摄像头产生的帧未能
及时交付。

StarryOS 客户机样本略高于原生样本，但受窗口数和 USB 调度抖动影响，不能据此判断
虚拟化优于原生。可靠结论是单核客户机未破坏视觉路径，StarryOS 稳定在 15～16 FPS
区间。

### 5.3 优化记录

**640×480 模型替代 640×640**：消除上下各 80 像素的无效补边，输入
元素和候选点均减少 25%。Linux NPU 时间从 42 ms 降至 16 ms，StarryOS 从 94 ms
降至 20 ms。

**同步 UVC 取帧**：用前台同步等待替代异步回调 + 分发线程，关闭曝光优先导致的
动态降帧。StarryOS 纯摄像头路径从约 11 FPS 提升至 27.6 FPS，证明 USB 链路本身
接近 30 FPS 能力。

**机械臂期间暂停 UVC**：动作期间暂停摄像头传输，StarryOS 机械臂最坏控制周期从
213 ms 降至 50 ms。

**模型先于摄像头初始化**：先分配 NPU 内存再启动摄像头，StarryOS 启动到视觉就绪
从最长 27 s 降至约 1.5 s。

**ReLU 替代 SiLU**：StarryOS 帧率从 13.7 FPS 提升至 14.1 FPS（+3.4%）。

### 5.4 注意事项

- 使用 Release 构建，StarryOS 推荐 Warn 日志级别
- 至少运行到两个完整 10 s 稳态窗口，排除预热和首次分配的影响
- 只比较同一模型、场景、NPU core、日志级别和状态分支的数据
- 不要在 StarryOS 板端重定向日志到文件
- 完整程序帧率以 `effective` 为准，`camera` 和 `busy` 含义不同
- FPS 不能代表机械臂流畅度，还需检查 50 ms 控制周期、P95 延迟和动作视频

