# aka-rk3588 双客户机感知与控制实现说明

本文说明 AKA 为 Orange Pi 5 Plus/RK3588 上的 AxVisor+StarryOS+Zephyr 双客户机捡球方案
增加的源码、运行方式和验证方法。

当前架构把机器人业务代码统一放在 AKA：StarryOS 运行感知用户程序，Zephyr 运行实时控制
application，两端共用同一个 48 字节 IVC 协议。TGOSImages 只提供 Zephyr 构建环境，
TGOSKits/AxVisor 负责双客户机、IVC 和设备资源集成。

文档依据 2026-08-24 的 `dual` 分支本地未提交工作树编写。文中会区分源码可构建、host
测试、板端启动、设备动作和完整捡球，避免把其中一个结果扩大为全部完成。

---

## 1. 目标和总体结构

### 1.1 功能拆分

AxVisor 同时运行两台客户机：

| 客户机 | AKA 程序 | 职责 | 独占资源 |
| --- | --- | --- | --- |
| VM 1：StarryOS | `tennis-perception` | UVC 采集、RKNN 网球识别、球桶识别、发布结果 | 摄像头、RKNPU、根文件系统 |
| VM 2：Zephyr | `orangepi_robot_control` | 接收结果、底盘决策、机械臂轨迹、安全停车 | UART6、Feetech 控制总线、虚拟定时器 |

数据路径为：

```text
UVC 摄像头
    ↓
StarryOS：tennis-perception
    ├── MJPEG 解码
    ├── RKNN YOLOv8 网球识别
    └── 红色球桶识别
    ↓ struct perception_result_v2（48 字节）
/dev/axivc 发布端
    ↓
AxVisor IVC v2 共享环形队列
    ↓
Zephyr：orangepi_robot_control
    ├── 校验并选择最新结果
    ├── 底盘/抓取/放球状态机
    ├── 350 ms 输入看门狗
    └── 50 ms 机械臂插值
    ↓
UART6 1 Mbps
    ↓
Feetech 车轮 ID 7～9、机械臂 ID 1～6
```

### 1.2 源码统一与运行隔离并不冲突

“感知和控制放在同一个 AKA 仓库”只表示代码所有权统一，不表示它们重新运行在同一个
进程或客户机中：

- Starry 只编译并运行 `tennis-perception`；
- Zephyr 控制代码静态链接到另一台客户机镜像；
- Starry 不打开执行器控制总线；
- Zephyr 不访问摄像头、RKNN 模型或根文件系统；
- 两端只通过版本化 IVC 消息耦合。

因此可以在一个 PR 中同步修改识别结果和控制策略，又能继续验证虚拟化隔离和实时任务
拆分。

### 1.3 与原有单系统程序的关系

原有程序 `tennis` 保持不变，它在一个 Linux/Starry 环境中完成感知和控制：

```text
tennis
├── 摄像头和 RKNN
├── 底盘控制
├── Feetech 机械臂
├── 校准和逆运动学
└── 完整捡球状态机
```

双客户机场景新增：

```text
tennis-perception                  # Starry 用户程序，只负责感知和发送
zephyr/orangepi_robot_control      # Zephyr application，只负责控制
```

不要在双客户机场景中同时运行原 `tennis` 和 Zephyr 控制应用，否则两边可能竞争同一套
执行器。

### 1.4 与 TGOSImages、TGOSKits 的边界

AKA 负责：

- 机器人感知和控制业务源码；
- Starry/Zephyr 公共协议；
- 机器人状态机、舵机 ID 和时间参数；
- AKA 侧的一键构建入口；
- 感知和控制单元测试；
- Starry 侧运行脚本。

TGOSImages 负责：

- Zephyr 源码版本和补丁；
- Python 环境和交叉工具链；
- Orange Pi Zephyr board；
- 将 AKA application 编译为 BIN、ELF 和 DTB。

TGOSKits/AxVisor 负责：

- 两台客户机的 vCPU 和内存；
- Starry 客户机设备；
- Zephyr UART6 直通；
- IVC channel；
- 镜像打包、板端启动和控制台多路复用。

---

## 2. 当前修改结构

与双客户机机器人功能直接相关的源码为：

```text
CMakeLists.txt
README.md

perception/
├── perception_main.cpp
├── perception_result.hpp
├── bucket_detector.cpp
└── bucket_detector.hpp

protocol/
└── perception_result_v2.h

zephyr/orangepi_robot_control/
├── README.md
├── CMakeLists.txt
├── prj.conf
├── app.overlay
├── src/
│   ├── main.c
│   ├── ivc_transport.c
│   ├── ivc_transport.h
│   ├── feetech_bus.c
│   ├── feetech_bus.h
│   ├── robot_controller.c
│   ├── robot_controller.h
│   ├── resettable_watchdog.c
│   └── resettable_watchdog.h
└── tests/
    ├── CMakeLists.txt
    ├── prj.conf
    ├── testcase.yaml
    └── src/main.c

scripts/
└── build_zephyr_control.sh

tests/
└── bucket_detector_test.cpp

run_dual_pick.sh
run_dual_pick_ci_once.sh
AKA_RK3588_MODIFICATIONS.md
```

`build/`、`build-ivc-cross/` 等目录是本地生成物，不属于源码修改。

---

## 3. Starry 感知端修改

### 3.1 `CMakeLists.txt`：独立感知目标

#### 原因

原 `tennis` 同时链接感知和控制模块。如果把它直接放进 Starry 客户机，Starry 仍会控制
舵机，无法让 Zephyr 成为执行器的唯一所有者。

#### 实现

新增 `PERCEPTION_SOURCES`，复用原工程的：

- UVC 同步采集；
- TurboJPEG 解码；
- RKNN runtime；
- YOLOv8 后处理；
- image/file 公共工具。

在 native 和 cross-compile 两条 CMake 分支中都增加：

```cmake
add_executable(tennis-perception ${PERCEPTION_SOURCES})
```

这里的两条分支表示构建环境不同：

- native：在 AArch64 Linux/Orange Pi 上使用本机库编译；
- cross-compile：在 x86 主机用 AArch64 交叉编译器生成目标程序。

它们不是两份业务源码。两条路径构建同一个 `tennis-perception`，但链接库搜索方式不同。

感知目标不链接原有：

```text
feetech/
robot/
motor/
arm/
```

因此 Starry 端无法通过这个可执行文件直接控制车轮和机械臂。

### 3.2 `perception_main.cpp`：感知主循环

主要功能：

1. 解析模型、摄像头和 transport 参数；
2. 根据模式打开 `/dev/axivc` 或标准输出；
3. 初始化 RKNN 模型；
4. 打开并预热 UVC 摄像头；
5. 每帧执行解码、网球识别和球桶识别；
6. 组装公共协议消息；
7. 尝试写入 IVC；
8. 按独立频率打印结果和 10 秒窗口统计。

生产模式默认长期运行，直到收到信号或遇到不可恢复错误。

### 3.3 网球和球桶结果

网球结果复用原 RKNN/YOLOv8 路径，输出：

- 是否可见；
- 置信度；
- 目标中心；
- bounding box 大小；
- 原始帧宽高。

球桶使用独立的红色区域检测，输出球桶中心和边界框。Starry 只报告图像空间结果，不在
感知程序中生成轮速或机械臂目标位置。

### 3.4 IVC 发布端

`--transport ivc` 时打开：

```text
/dev/axivc
```

每条消息是一个完整的 `struct perception_result_v2`。写入使用有限重试；如果 ring 暂时满，
当前旧结果可以丢弃，摄像头和 NPU 主循环继续运行。这样不会因 Zephyr 短暂消费不及时而
永久停止感知。

持续出现 dropped 并不是正常稳定状态，通常表示：

- Zephyr 尚未订阅；
- Zephyr 消费停止；
- IVC 通道状态异常；
- 生产速率长期超过消费能力。

### 3.5 发送频率与日志频率分离

两个参数语义不同：

| 参数 | 作用 |
| --- | --- |
| `--report-every N` | 每 N 次有效推理生成并尝试发送结果 |
| `--status-every N` | 每 N 条生成结果打印一次窗口帧率和最新识别结果，0 表示关闭 |
| `--pipeline-heartbeat 0\|1` | 是否每秒打印流水线卡点诊断心跳 |

生产默认值为：

```text
--report-every 1
--status-every 60
--pipeline-heartbeat 0
```

即每帧发送，但只按 60 条结果低频打印统一状态，且默认不运行心跳打印线程。降低日志量
不会降低 Zephyr 的输入频率。需要定位感知流水线停滞时，可临时设置
`PIPELINE_HEARTBEAT=1` 启用每秒心跳。

### 3.6 Starry 感知测试

`tests/bucket_detector_test.cpp` 覆盖球桶检测的典型情况。host 构建还保留原有运动策略测试，
用于确认新增感知目标没有破坏单系统代码。

---

## 4. 公共 IVC 协议

### 4.1 唯一权威定义

公共结构位于：

```text
protocol/perception_result_v2.h
```

该头文件同时兼容 C 和 C++：

- Starry C++ 的 `perception_result.hpp` 包含它并建立类型别名；
- Zephyr C application 直接包含它；
- C++ 下保留字段默认初始化；
- C 下保持普通 wire struct。

两端不再各自保存一份重复结构体。

### 4.2 消息布局

协议版本为 2，大小固定为 48 字节：

```text
magic / version / flags
sequence / monotonic_ms
frame_width / frame_height
confidence_milli
ball center / box
bucket center / box
```

主要常量：

| 字段 | 值或含义 |
| --- | --- |
| `magic` | `0x31524350` |
| `version` | `2` |
| `PERCEPTION_TARGET_VISIBLE` | bit 0 |
| `PERCEPTION_BUCKET_VISIBLE` | bit 1 |
| `PERCEPTION_ROBOT_CI` | bit 2，仅架空确定性验收 |

两端都有 48 字节编译期检查。若以后需要新增字段，应新增协议版本或兼容布局，不能只在某一
端修改结构。

### 4.3 字段使用边界

- `sequence` 用于观察处理是否持续以及是否跳帧；
- `monotonic_ms` 是 Starry 内部时间，不是跨 VM 端到端延迟；
- 当前两台客户机没有完成严格时钟同步；
- 文本形式 `STARRY_PERCEPTION_STATUS` 只是低频运行状态，真正通信的是二进制结构。

---

## 5. Zephyr 控制端修改

### 5.1 应用形态

`zephyr/orangepi_robot_control` 是 Zephyr application。它不是写入根文件系统、再从 Shell
启动的 ELF。构建时它与 Zephyr 内核、驱动和 Shell 静态链接，客户机启动后自动进入
`main()`。

应用源码属于 AKA；Zephyr 内核版本、工具链和 board 属于 TGOSImages。

### 5.2 `CMakeLists.txt`、`prj.conf` 和 `app.overlay`

`CMakeLists.txt` 编入：

- 主循环；
- IVC transport；
- Feetech 总线；
- 控制状态机；
- 可重置单次看门狗。

同时把 AKA 的 `protocol/` 加入 include path，确保 Starry 和 Zephyr 使用同一协议。

`prj.conf` 启用：

- ARM Generic Timer 和系统时钟；
- NS16550 串口驱动；
- deferred logging；
- 虚拟控制台 Shell；
- 适合控制任务的 main/workqueue/log stack。

控制台和执行器串口彼此独立：

- AxVisor 虚拟 UART：Zephyr Shell 和日志；
- UART6：1 Mbps Feetech 设备总线。

`app.overlay` 定义 UART6：

```text
MMIO: 0xfeb90000，长度 0x100
IRQ:  GIC SPI 337
reg-shift: 2
clock-frequency: 16 MHz
current-speed: 1 Mbps
```

Zephyr 应用自己的 SRAM 视图缩小为 512 KiB；AxVisor VM 配置仍可能为隔离槽预留更大的
宿主物理内存，两者不是同一个概念。

### 5.7 可重置输入看门狗

每条有效感知结果都会重置一次 350 ms 单次定时器。若超过 350 ms 没有新输入：

- 停止底盘；
- 保留应用和 Zephyr 客户机运行；
- 新的有效输入到达后可以继续控制。

看门狗独立于机械臂 50 ms 插值周期，也不依赖主循环每 50 ms 才检查一次。

这解决的是“感知/通信停止后不能继续执行旧轮速”的业务安全问题，不替代 AxVisor、控制板
或电源级硬件急停。

### 5.8 Shell 和日志

Zephyr Shell 使用 AxVisor 虚拟 UART，不占用 UART6。可使用：

```text
help
kernel uptime
kernel thread list
device list
```

控制 application 自身自动运行，不需要、也不能在 Shell 中再次启动一份。

### 5.9 Zephyr 控制测试

`zephyr/orangepi_robot_control/tests` 通过 `native_sim` 覆盖 4 个行为：

- 感知立即驱动底盘，不等待 arm tick；
- arm tick 只推进机械臂插值；
- 看门狗可重置且 one-shot；
- 超时停车后新输入恢复。

测试不覆盖真实 IVC HVC、UART6、电机供电、机械方向和真实夹球。

---

## 6. 构建方法

### 6.1 构建 Starry 感知程序

已有 build 目录时：

```bash
cd /path/axvisor_two/aka-rk3588
cmake --build build --target tennis-perception -j"$(nproc)"
```

已有交叉构建目录时：

```bash
cmake --build build-ivc-cross --target tennis-perception -j"$(nproc)"
```

完整仓库构建仍可使用：

```bash
./build_rk3588.sh -b Release -l INFO
```

交叉编译器的 glibc/sysroot 必须与 Starry 根文件系统兼容。主机构建成功不能替代板端：

```bash
file <tennis-perception>
ldd <tennis-perception>
```

检查不应出现缺失动态库或目标 glibc 版本高于根文件系统。

### 6.2 从 AKA 构建 Zephyr 控制镜像

推荐两个仓库同级放置，然后执行：

```bash
cd /path/axvisor_two/aka-rk3588
./scripts/build_zephyr_control.sh
```

TGOSImages 查找顺序：

1. `--tgosimages-dir <路径>`；
2. `TGOSIMAGES_DIR`；
3. 同级 `../tgosimages`；
4. 不存在时 clone 到同级目录。

显式指定：

```bash
./scripts/build_zephyr_control.sh \
  --tgosimages-dir /path/to/tgosimages
```

已有 TGOSImages 工作树不会被 pull、checkout、reset 或 clean。

### 6.3 从 TGOSImages 反向构建

```bash
cd /path/axvisor_two/tgosimages
./scripts/apps/aka-rk3588-zephyr.sh
```

非同级目录：

```bash
./scripts/apps/aka-rk3588-zephyr.sh \
  --aka-dir /path/to/aka-rk3588
```

两个入口最终构建的都是 AKA 中的同一目录：

```text
zephyr/orangepi_robot_control
```

### 6.4 Zephyr 默认产物

产物保存在 TGOSImages：

```text
IMAGES/orangepi/zephyr/orangepi-5-plus
IMAGES/orangepi/zephyr/orangepi-5-plus.elf
IMAGES/orangepi/zephyr/orangepi-5-plus.dtb
```

构建成功后可以确认正式入口：

```bash
strings ../tgosimages/IMAGES/orangepi/zephyr/orangepi-5-plus.elf \
  | rg ZEPHYR_ROBOT_CONTROL_START
```

### 6.5 运行单元测试

AKA host 测试使用独立 build 目录：

```bash
cmake -S . -B build-tests \
  -DBUILD_TESTING=ON \
  -DNATIVE_BUILD=OFF \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build-tests \
  --target feetech_motion_policy_test bucket_detector_test \
  -j"$(nproc)"

ctest --test-dir build-tests --output-on-failure
```

Zephyr 控制测试使用 TGOSImages 已准备的 Zephyr 环境：

```bash
cd ../tgosimages

ZEPHYR_BASE="$PWD/build/zephyr" \
ZEPHYR_TOOLCHAIN_VARIANT=host \
/tmp/zephyr-pyenv/bin/python build/zephyr/scripts/twister \
  -T ../aka-rk3588/zephyr/orangepi_robot_control/tests \
  -p native_sim/native/64
```

---

## 7. 部署和运行

### 7.1 部署边界

AKA/TGOSImages 构建完成后，还需要 TGOSKits：

1. 使用 Zephyr BIN 和 DTB；
2. 构建 StarryOS 客户机；
3. 配置两台 VM、IVC 和 UART6；
4. 组装 AxVisor 启动镜像；
5. 上传并启动 Orange Pi。

仅看到 TGOSImages `build succeeded` 不能证明板端已经使用新镜像。部署时应对比产物、
TGOSKits 打包文件和板端加载文件的 SHA-256。

### 7.2 Starry 根文件系统内容

至少需要：

```text
/home/orangepi/robot/aka-rk3588-dual/
├── build-ivc-native-linux/tennis-perception
├── models/tennis.rknn
├── run_dual_pick.sh
└── run_dual_pick_ci_once.sh
```

还需要兼容的 UVC、TurboJPEG、RKNN runtime 和 `/dev/axivc`。

Zephyr 不需要访问这些文件；它是无盘、静态链接客户机。

### 7.3 启动顺序

1. 启动 AxVisor 和两台客户机；
2. Zephyr 自动进入控制 `main()`；
3. Zephyr 初始化 timer、UART6、车轮和机械臂；
4. Zephyr 等待 Starry 发布端；
5. 进入 Starry Shell；
6. 运行感知脚本；
7. Zephyr 自动接收结果并控制。

Starry 生产命令：

```bash
cd /home/orangepi/robot/aka-rk3588-dual
./run_dual_pick.sh
```

默认等价于：

```bash
RKNN_CORE_MASK=0 ./build-ivc-native-linux/tennis-perception \
  models/tennis.rknn 0 \
  --transport ivc \
  --report-every 1 \
  --status-every 60 \
  --pipeline-heartbeat 0
```

### 7.4 只测试感知

不发送 IVC、不控制机器人：

```bash
RKNN_CORE_MASK=0 ./build-ivc-native-linux/tennis-perception \
  models/tennis.rknn 0 \
  --transport stdout \
  --max-results 100 \
  --report-every 1 \
  --status-every 20 \
  --pipeline-heartbeat 0
```

stdout 模式只能验证摄像头、模型和结果格式。

### 7.5 架空完整流程

确认车轮架空、机械臂范围安全后：

```bash
./run_dual_pick_ci_once.sh
```

该模式使用真实摄像头和 RKNN 统计性能，再发送确定性控制场景。通过表示：

- 感知程序运行；
- IVC 数据能到达 Zephyr；
- 底盘各方向命令已发出；
- 机械臂动作序列已执行。

它不证明车辆落地移动、真实夹住网球或真实投入球桶。

### 7.6 控制台切换

快捷键是连续操作：先按 `Ctrl+X`，完全松开，再按第二个键。

| 操作 | 功能 |
| --- | --- |
| `Ctrl+X`，松开后按 `h` | 返回 AxVisor Shell |
| `Ctrl+X`，松开后按 `]` | 下一台客户机 |
| `Ctrl+X`，松开后按 `[` | 上一台客户机 |
| `vm console 1` | 从 AxVisor Shell 进入 Starry |
| `vm console 2` | 从 AxVisor Shell 进入 Zephyr |

控制台切换不会暂停后台客户机。

---

## 8. 日志判读

### 8.1 Starry 启动

```text
STARRY_PERCEPTION_READY protocol=2 transport=ivc robot_ci=0
```

表示模型、摄像头和 transport 已完成初始化，但不能单独证明 Zephyr 已收到第一条结果。

### 8.2 Starry 结果与帧率

默认每 60 条生成结果把实际耗时、平均帧率和窗口末尾的最新识别结果统一打印为一行：

```text
STARRY_PERCEPTION_STATUS results=60 window_s=... inference_fps=... ivc_fps=... sent=... dropped=... seq=... frame=640x480 ball_visible=... ball_confidence_milli=... ball_center=... ball_box=... bucket_visible=... bucket_center=... bucket_box=...
```

含义：

- `results`：本统计窗口内生成并尝试发送的结果数；
- `window_s`：生成这些结果实际经历的秒数，不是固定打印周期；
- `inference_fps`：成功完成的感知频率；
- `ivc_fps`：成功写入 IVC 的频率；
- `sent`：窗口内成功写入 IVC 的结果数；
- `dropped`：窗口内没有成功写入的结果数。
- `seq` 和 `frame`：最新结果的序号和输入画面尺寸；
- `ball_*`：最新一帧的网球可见性、千分制置信度、中心坐标和检测框；
- `bucket_*`：最新一帧的球桶可见性、中心坐标和检测框。

文本状态不重复打印协议 `magic` 和 `version`；二进制 IVC 消息仍保留这两个字段供 Zephyr
校验。通过 `STATUS_EVERY` 可以调整结果条数，设置为 0 可关闭状态打印。

正常稳定通信时，`inference_fps` 与 `ivc_fps` 应接近，`dropped` 应保持 0。

### 8.3 Zephyr 启动

```text
ZEPHYR_ROBOT_CONTROL_START protocol=2 source=axvisor-ivc uart=uart6
ZEPHYR_TIMER_READY ...
ZEPHYR_IVC_READY ...
ZEPHYR_ROBOT_CONTROL_READY ...
```

若出现 `ZEPHYR_ROBOT_CONTROL_FAILED`，根据 `reason` 检查 UART6、舵机初始化或 IVC。

### 8.4 Zephyr 处理频率

```text
ZEPHYR_CONTROL_STATUS messages=60 window_s=... rx_fps=... control_fps=... coalesced=... seq=... received=... processed=... invalid=... state=... arm_cycles=... max_arm_timer_late_ms=... frame=... ball_visible=... ball_confidence_milli=... ball_center=... ball_box=... bucket_visible=... bucket_center=... bucket_box=...
```

Zephyr 默认每收到 60 条有效 IVC 消息打印一次。`rx_fps` 是有效接收频率，`control_fps`
是送入控制状态机的频率，`coalesced` 是 latest-wins 合并掉的旧结果数。稳定且无积压时：

```text
Starry inference_fps ≈ Starry ivc_fps ≈ Zephyr rx_fps ≈ Zephyr control_fps
coalesced = 0
```

设置 `CONFIG_ROBOT_STATUS_EVERY_MESSAGES=0` 可关闭该状态。Zephyr 使用 latest-wins，
短时存在 sequence gap 不一定是错误；持续 `coalesced>0` 则需要检查 ring 积压、Zephyr
主循环或 UART/锁等待。

### 8.5 心跳和看门狗

Starry 的流水线诊断心跳默认关闭。排查摄像头、解码、RKNN 或 IVC 卡点时执行：

```bash
PIPELINE_HEARTBEAT=1 ./run_dual_pick.sh
```

此时每秒打印：

```text
STARRY_PIPELINE_HEARTBEAT stage=... attempt=... inference=... progress=... stalled_s=...
```

正常运行保持 `PIPELINE_HEARTBEAT=0`。

`ZEPHYR_ROBOT_CONTROL_ALIVE` 默认通过 `CONFIG_ROBOT_CONTROL_ALIVE_LOG=n` 关闭。需要在
完全没有 IVC 输入时单独观察控制客户机存活，改为 `y` 后重新构建 Zephyr 镜像。

输入停止约 350 ms 后：

```text
ZEPHYR_INPUT_WATCHDOG state=... elapsed_ms=... timeout_ms=350
```

该日志表示底盘已因输入超时停车，Zephyr 客户机本身仍应继续运行。

### 8.6 判断端到端链路

不能只看某一端的一条成功字符串。完整观察至少包括：

```text
Starry: STARRY_PERCEPTION_READY transport=ivc
Starry: STARRY_PERCEPTION_STATUS 中 inference_fps≈ivc_fps、dropped=0
Zephyr: ZEPHYR_IVC_READY
Zephyr: ZEPHYR_CONTROL_STATUS 中 rx_fps≈control_fps、coalesced=0、invalid=0
```

---

## 9. 当前验证状态和边界

### 9.1 拆分后软件验证

- 公共协议按 C11 编译通过；
- 公共协议按 C++17 编译通过；
- native `tennis-perception` 编译通过；
- cross-compile `tennis-perception` 编译通过；
- 从 AKA 入口构建 Zephyr 成功；
- 从 TGOSImages 入口构建 Zephyr 成功；
- 两个入口产物哈希一致；
- Zephyr ELF 包含正式控制入口，不含临时死锁复现入口；
- Zephyr control native_sim 测试 4/4 通过；
- 两仓库 `git diff --check` 通过。

### 9.2 已有板端记录

- 双客户机能够启动并并行运行；
- Starry UVC 和 RKNN 能持续产生结果；
- Starry→Zephyr IVC 有发送、接收和 `invalid=0` 记录；
- ARM Generic Timer、`k_msleep()` 和 WFI 唤醒有板端记录；
- UART6 1 Mbps 能与控制板通信；
- 车轮 ID 7～9 有实际转动记录；
- 机械臂有动作和反馈记录；
- 350 ms 输入看门狗和 50 ms 机械臂周期有板端记录；
- 架空确定性捡球动作链有完成记录。

这些是此前实现的板端证据；本次“仓库重新拆分”只重新完成了软件构建和 host 测试，没有
重新部署板卡。

### 9.3 尚未完成或不能扩大表述

- 新目录布局产物尚未重新完成板端部署验收；
- 未完成 24 小时双客户机压力测试；
- 未测量感知→IVC→决策→UART 的 P99/最大时延；
- IVC 当前仍约 1 ms polling，没有通知 IRQ/event；
- publisher 退出和重启后的自动重连仍需完善；
- UART6 pinmux/时钟仍依赖板级预配置；
- 架空转轮不等于车辆地面运动正确；
- 确定性 holding 模拟不等于真实夹球；
- 软件看门狗不等于硬件急停；
- 当前修改尚未 commit、push 或形成 PR。

---

## 10. 后续优化方向

### 10.1 通信

- 用 AxVisor 通知 IRQ/event 替代 1 ms polling；
- 明确区分 NotReady、ring full 和永久错误；
- 支持 publisher 退出和重新创建后的自动重连；
- 增加 sequence gap、ring occupancy 和重连测试；
- 完善双向状态确认和健康监测。

### 10.2 实时性和安全

- 将 Feetech 控制迁到专用高优先级线程/workqueue；
- 使用 UART 中断或 DMA，避免同步反馈长时间阻塞；
- 增加控制板/硬件级超时停车；
- 明确通信中断时机械臂保持、回安全位或 torque-off 策略；
- 统计接收、决策、串口发送和反馈的 P99/P99.9/最大时延。

### 10.3 构建和部署

- 固定 AKA、TGOSImages、TGOSKits 三仓库兼容 commit；
- 记录程序、模型、Zephyr BIN/DTB 和最终 FIT 的 SHA-256；
- 为 Starry 根文件系统提供可复现安装包；
- 在部署流程中自动校验实际加载镜像不是旧产物。

---

## 11. 建议提交拆分

AKA 建议按评审边界拆成三个提交。

### 提交一：Starry 感知端和公共协议

```text
CMakeLists.txt
perception/
protocol/perception_result_v2.h
tests/bucket_detector_test.cpp
run_dual_pick.sh
run_dual_pick_ci_once.sh
```

### 提交二：Zephyr 机器人控制应用

```text
zephyr/orangepi_robot_control/CMakeLists.txt
zephyr/orangepi_robot_control/prj.conf
zephyr/orangepi_robot_control/app.overlay
zephyr/orangepi_robot_control/src/
zephyr/orangepi_robot_control/tests/
zephyr/orangepi_robot_control/README.md
```

### 提交三：构建入口和总体文档

```text
scripts/build_zephyr_control.sh
README.md
AKA_RK3588_MODIFICATIONS.md
.gitignore（若仅包含相关生成目录规则）
```

不应提交：

```text
build/
build-ivc-cross/
build-tests/
生成的 BIN/ELF/DTB
模型或根文件系统的临时复制品
TGOSImages build/、IMAGES/、twister-out/
```

TGOSImages 的外部 application 构建能力应在 TGOSImages 单独提交；TGOSKits 的 VM、IVC 和
UART6 集成应在 TGOSKits 单独提交。

---

## 12. 总结

当前 AKA 已从“只保存 Starry 感知修改”调整为机器人双客户机业务代码的统一仓库：

- 原单系统 `tennis` 保持可用；
- Starry 新增独立 `tennis-perception`；
- 感知结果通过唯一公共 48 字节协议发送；
- Zephyr 控制 application、状态机、UART6 和看门狗迁入 AKA；
- 感知立即驱动底盘，机械臂保持独立 50 ms 插值；
- 350 ms 无输入时安全停车；
- AKA 和 TGOSImages 两边都能构建同一份 Zephyr 控制源码；
- TGOSImages 继续独立管理 Zephyr 系统环境；
- TGOSKits 继续独立管理虚拟化和板端资源。

这种布局同时满足“机器人业务代码集中维护”和“感知/实时控制在不同客户机隔离运行”。
当前软件拆分和构建已经闭环，下一阶段仍需完成新布局产物的板端重新部署、长期稳定性、
严格时延和真实落地捡球验收。
