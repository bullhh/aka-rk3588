# aka-rk3588 virtual 分支：UVC / RKNN / FT232R 回环 CI

本文说明 aka-rk3588 的 `virtual` 分支 CI。该分支面向**没有机械臂和车轮**的
OrangePi 5 Plus：用真实 UVC 摄像头、真实 RKNN YOLO 推理和 FT232R TXD/RXD
物理回环，替代原机器人 CI 中的执行器部分。文档描述长期有效的实现、接口和判定
契约，不记录某次调试过程或现场流水账。

## 1. 这是测什么

`virtual` CI 验证以下两件事能在同一块板、同一时间段内真实并发运行：

1. 摄像头到 NPU 的完整视觉链路：UVC MJPEG 采集、JPEG 解码、letterbox、RKNN
   YOLO `detect_run` 推理，以及持续吞吐。
2. USB 到 FT232R 的物理回环链路：FT232R 自身 TXD/RXD 短接后，按固定节拍发送
   测试帧并逐字节比较回显。

CI 不进入 LeKiwi 状态机，不初始化或驱动机械臂和车轮，也不打开机械臂串口
`/dev/ttyS6`。涉及的三条检查如下：

| CI 文件 | check id | 板卡配置 | 环境 |
| --- | --- | --- | --- |
| `.github/ci/checks/starry.toml` | `test-orangepi-5-plus-robot-native-starryos` | `test-suit/starryos/board-orangepi-5-plus/robot-flow/board-orangepi-5-plus-robot.toml` | 原生 StarryOS |
| `.github/ci/checks/axvisor.toml` | `test-orangepi-5-plus-robot-axvisor-starryos-guest` | `test-suit/axvisor/normal/board-orangepi-5-plus/robot-starry/smoke/board-orangepi-5-plus-robot-starry.toml` | AxVisor + StarryOS guest |
| `.github/ci/checks/axvisor.toml` | `test-orangepi-5-plus-robot-axvisor-linux-guest` | `test-suit/axvisor/normal/board-orangepi-5-plus/robot-linux/smoke/board-orangepi-5-plus-robot-linux.toml` | AxVisor + Linux guest |

上表中的 `.github/ci/...` 与 `test-suit/...` 都属于相邻的 **tgoskits** 仓库，
不是本 aka-rk3588 仓库内的文件；本仓库提供板卡上运行的 `aka-rk3588-virtual`
应用和启动器，board/VM 配置在 tgoskits 侧维护。

三条检查共用固定运行目录 `/home/orangepi/robot-ci/aka-rk3588-virtual` 和同一个应用
入口，但各自有独立的 board/VM 配置。

## 2. virtual 分支边界

本分支不含真实机械控制代码、命令或后端：`build/tennis` 只保留
`vision-usb-ci` 子命令，源码中不包含机械臂/车轮驱动、Feetech 总线、LeKiwi
状态机、真实控制配置或旧机器人 CLI。因此本分支无法驱动机械臂和车轮，也不为
真实控制提供兼容或回退路径。

| 维度 | virtual CI |
| --- | --- |
| 视觉 / NPU / 性能 | 覆盖 UVC MJPEG、JPEG 解码、RKNN YOLO 推理和吞吐 |
| 机械臂 / 车轮 | 不初始化、不驱动、无真实控制代码 |
| `/dev/ttyS6` | 不打开 |
| 并发 USB 负载 | FT232R TXD/RXD 物理回环，只验证 USB/串口收发和字节完整性 |
| 重试 | 无重试，首个回环错误立即失败 |

FT232R 回环不是控制器模拟，也不是舵机应答或机械动作验证。回环帧即使原样返回，
也只证明发送、接收和字节比较通过，不能证明控制器理解命令、伺服器回复、轨迹或
机械臂/车轮最终动作正确。

真实机器人后续必须使用独立的真实分支或发布包；本 virtual 分支不保留真实控制
入口、旧启动脚本或机械操作文档。

## 3. 硬件与运行前置

- OrangePi 5 Plus 板卡，运行目标系统（原生 StarryOS 或 AxVisor guest）。
- 真实 UVC 摄像头，应用只按 `UVC_INDEX` 选择设备；当前 `UvcCapture` 不读取
  VID/PID，日志和文档不声明具体相机型号或 USB 身份。
- 一个 0403:6001 FT232R 适配器，TXD 与 RXD 短接构成物理回环；多适配器时用 `FT232_SERIAL` 或 `--ftdi-serial` 指定序列号，代码不硬编码现有序列号。
- `models/tennis.rknn` 模型文件。
- 可运行的 `build/tennis`，以及 `libuvc`、`libusb`、`libturbojpeg`、`librknnrt` 等运行时库；TTY transport 使用内核 `ftdi_sio` 和 termios，不依赖 libftdi。
- CI 固定应用目录：`/home/orangepi/robot-ci/aka-rk3588-virtual`。

完整部署、版本目录和板卡资产由人工发布/板卡资产管理负责；CI 不自动部署。本文
只给出最少构建和运行约定，不展开发布流水线。

## 4. 执行链路

```text
run_vision_usb_ci_once.sh 28.0
  -> build/tennis vision-usb-ci models/tennis.rknn 28.0 0
     -> FT232R select/open
     -> RKNN model init
     -> UVC open + 15-frame warmup
     -> 同一个回环 worker 与视觉并发执行两个 >= 10s 窗口
        -> UVC MJPEG -> JPEG 解码/letterbox -> detect_run
        -> FT232R 测试帧发送 + 回显逐字节比较
     -> worker stop，读取两个窗口的最终统计
     -> UVC pause/resume，并确认能拿到新帧
     -> cleanup
     -> [VISION_USB_CI] APPLICATION_PASS
  -> 启动器复核字段和进程 exit 0
  -> [VISION_USB_CI] RESULT=PASS attempts=1
```

启动顺序是 FT232R 设备选择/打开、RKNN 模型初始化、UVC 打开和 15 帧预热。两个
窗口依次运行：窗口 1 结束不会停止或重启回环线程，窗口 2 继续使用同一个 worker；
只有两窗都结束或提前失败退出后，才停止一次 worker 并读取两窗最终统计。

## 5. FT232R 的两种 transport

同一个 `build/tennis` 支持两种传输，实际使用哪一种以 `DEVICE` 日志中的
`transport=` 为准：

| transport | 使用位置 | 实现路径 | 覆盖内容 |
| --- | --- | --- | --- |
| `usb` | StarryOS root shell、Linux root | libusb 厂商控制请求 + bulk 端点 | USB host/usbfs、内核驱动 detach/claim/release |
| `tty` | Linux guest | 通过 sysfs 精确映射到该 0403:6001 设备的 `/dev/ttyUSBx`，使用 termios | Linux `ftdi_sio` TTY 路径 |

默认 `auto`：先尝试 `usb`；如果设备存在但 raw libusb 打开被拒、transport 不
支持、发生其他可回退错误，或者 libusb 因权限无法读取 serial 而无法完成序列号
选择，才尝试 `tty`。`tty` 后端会重新从 sysfs 读取 serial 并再次做严格选择，
因此“读不到 serial”不会被误判成多设备歧义。真正的多设备无选择器、可读 serial
重复、可读 serial 匹配不到，或 sysfs serial 不可读时直接失败，不会猜测设备。
显式 `FTDI_TRANSPORT=usb` 永不回退，失败即失败。两种 transport 都使用 1 Mbps、
8N1、无流控。

Linux CI 场景中，UVC 的 libusb 节点通常需要更高权限，由 board 配置以足够权限启动，
并强制 `FTDI_TRANSPORT=tty` 来实际覆盖 `ftdi_sio` 路径。Starry 后续必须在相邻
tgoskits 仓库的 board 配置中显式设置 `FTDI_TRANSPORT=usb`，当前尚未落实；本仓库
只保证显式 `usb` 严格不回退。本文不记录密码或某台板的权限处理细节。

`tty` 路径只接受 `/dev/ttyUSB` 加数字的节点，拒绝 `/dev/ttyS6`、`/dev/ttyACM0`
和其它非 FT232R 串口。

## 6. 回环帧与通过含义

回环负载使用 Feetech 命令帧形状，包含帧头、ID、长度、指令、自增序号、变化
nonce、payload 长度、payload 和校验和。指令固定为无写入副作用的 PING（`0x01`），
绝不使用 WRITE（`0x03`），也不构造 broadcast WRITE。每帧发送后读取回显并要求
逐字节完全一致；短包、长包、内容不一致、序号回退或超前都会被分类为错误。首个
回环错误会立即终止工作线程和本次运行，不做隐式重试。

如果适配器误接到真实 Feetech 总线，PING 最多让真实舵机返回非回显响应，严格逐字节
比较会失败并终止本次运行；它不会改寄存器或触发动作。

在 `transport=usb` 时，应用会剥离 FTDI 每个 USB IN 包前面的 2 个状态字节；只含
状态字节的包不产生数据。`transport=tty` 时这些状态字节由 Linux `ftdi_sio` 处理，
应用只比较 TTY 数据流。

该机制只验证通信链路和字节完整性；即使某帧回显成功，也不能把它当成舵机成功或
运动闭环成功。

## 7. PASS / FAIL 契约

应用使用两个窗口和一个并发回环 worker。每个窗口必须：

- 有效时长 `>= 10s`；
- `processed > 0`；
- 允许最多 3 个 MJPEG 解码失败帧：单帧解码失败只跳过该帧继续，不增加 `processed`，也不重置回环；第 4 个解码失败帧立即以 `reason=jpeg_decode_unstable` 结束本次运行；
- 成功回环次数 `>= 160`；
- 回环错误数为 0。

两窗合计还必须满足：

- 聚合有效 FPS `>= min_fps`；三条 CI 固定使用 `28.0`；
- 两个窗口的 MJPEG 解码失败总数在 `0..6`，且必须等于两窗 `jpeg_errors` 之和；
- 成功回环总数 `>= 320`；
- 回环错误总数为 0；
- UVC pause/resume 成功，并且 resume 后拿到新帧；
- RKNN 模型释放和设备清理结果满足要求；
- 应用进程退出码为 0。

应用只有在全部条件满足后才输出唯一的
`[VISION_USB_CI] APPLICATION_PASS ...`。启动器只在看到恰好一条带合法 `transport`
的 `DEVICE`、恰好两个 `PERF_WINDOW` 和两个 `LOOPBACK_WINDOW` 统计、恰好一条自洽的
`APPLICATION_PASS`、且进程 exit 0 时，才输出
`[VISION_USB_CI] RESULT=PASS attempts=1`。否则输出
`[VISION_USB_CI] RESULT=FAIL attempts=1` 并返回非零。

容错字段：`jpeg_errors` 出现在两个 `PERF_WINDOW` 行、`PERF_SUMMARY` 行和
`APPLICATION_PASS` 行。窗口行的 `jpeg_errors` 是该窗口跳过的坏 MJPEG 帧数，必须
`<= 3`；`PERF_SUMMARY` 和 `APPLICATION_PASS` 的 `jpeg_errors` 是两窗之和，必须
`<= 6` 且与两窗相加一致。坏帧说明 USB 仍有帧到达（不触发 `frame_acquisition_stalled`），
但该帧不参与 `processed`。启动器要求三处字段都存在且自洽：缺少 `jpeg_errors` 的旧程序
永远不能通过。该容错只用于吸收单帧硬件噪声，持续损坏仍会失败。

当前不做“必须检测到球”或固定图片语义断言：`detect_run` 返回负数才算推理失败，
返回 0 个检测框可以通过。本 CI 验证推理执行和吞吐，不验证模型准确率。

## 8. 三种测试环境

三条 board 配置默认使用普通 `OrangePi-5-Plus`；下面的 xtask 命令直接使用配置中的普通板标识，不需要额外板型参数。

| 环境 | 客户机 / 内核 | 说明 |
| --- | --- | --- |
| 原生 Starry | 直接运行 StarryOS | root shell 下使用 `transport=usb`，执行同一入口和契约 |
| AxVisor + Starry | 当前工作区构建的 SMP1 StarryOS guest | guest 配置为 `test-suit/axvisor/normal/board-orangepi-5-plus/robot-starry/guest.toml`，`image_location = "memory"` |
| AxVisor + Linux | `test-suit/axvisor/normal/board-orangepi-5-plus/robot-linux/linux-smp1-emmc.toml` | `image_location = "fs"`，`kernel_path = "/guest/linux/orangepi-5-plus-6.1.99"`，guest 根由 cmdline `root=/dev/mmcblk0p2` 指定，ramdisk 为 `/guest/linux/initramfs.cpio` |

上表中的 `/guest/linux/orangepi-5-plus-6.1.99` 是 AxVisor **宿主文件系统**上的
内核资产，不是 guest 根分区；`root=/dev/mmcblk0p2` 选择的是 guest 的 eMMC 根分区。
运行 AxVisor + Linux 用例前，必须由统一资产流程部署该镜像，并校验哈希、真实版本
和内核模块 ABI。README 只描述这个前置契约，不把该路径视为已经验证通过。

## 9. 最小运行方式

板卡上使用固定应用目录和启动器：

```sh
cd /home/orangepi/robot-ci/aka-rk3588-virtual
./run_vision_usb_ci_once.sh 28.0
```

Linux guest 手工复现应使用对应 board 配置规定的权限，并强制 `FTDI_TRANSPORT=tty`
以覆盖 `ftdi_sio` 路径，不能用默认 `auto` 替代该覆盖路径；原生 Starry 后续必须
在相邻 tgoskits 仓库的 board 配置中显式设置 `FTDI_TRANSPORT=usb`，当前尚未落实；
本仓库的显式 `usb` 模式只保证不回退。

直接调用应用：

```sh
./build/tennis vision-usb-ci models/tennis.rknn 28.0 0
./build/tennis vision-usb-ci models/tennis.rknn 28.0 0 \
  --ftdi-serial your_adapter_serial --ftdi-transport tty
```

`run_vision_usb_ci_once.sh` 只接受一个最小 FPS 参数，其它输入通过环境变量传入：

| 变量 | 作用 | 默认 |
| --- | --- | --- |
| `MODEL_PATH` | RKNN 模型路径 | `models/tennis.rknn` |
| `UVC_INDEX` | UVC 设备索引 | `0` |
| `FT232_SERIAL` | 多适配器时的 FT232R 序列号选择器 | 空 |
| `FTDI_TRANSPORT` | FT232R transport：`usb`、`tty`、`auto` | `auto` |
| `RKNN_CORE_MASK` | NPU 核掩码 | `0` |
| `VISION_USB_CI_CADENCE_HZ` | 回环 worker 节拍 | `20` |
| `VISION_USB_CI_MIN_TX` | 每窗最少成功回环数，只能提高到固定下限 160 之上 | `160` |

仓库三条 CI 的 xtask 命令如下。AxVisor + StarryOS 必须先按当前配置构建 SMP1
guest，再运行板卡用例：

```sh
# 原生 Starry
cargo xtask starry test board --board orangepi-5-plus-robot

# AxVisor + StarryOS：先构建当前工作区的 SMP1 guest
cargo xtask starry build \
  --config test-suit/starryos/board-orangepi-5-plus/robot-flow/build-aarch64-unknown-none-softfloat.toml \
  --smp 1
cargo xtask axvisor test board --board orangepi-5-plus-robot-starry

# AxVisor + Linux
cargo xtask axvisor test board --board orangepi-5-plus-robot-linux
```

## 10. 构建与宿主回归

### 10.1 板卡原生构建

板卡具备原生构建依赖时，可以运行：

```sh
./build_rk3588.sh -b Release -l WARN
```

aarch64 原生构建通过系统 `pkg-config` 解析 `libuvc`、`libusb-1.0`、
`libturbojpeg`，不需要交叉运行库目录。

如果目标板不具备构建依赖，运行包应由外部构建/资产流程部署，本文不展开逐文件
搬运或系统库 workaround。

### 10.2 Jammy 交叉构建（PC → AArch64）

在 x86/非 aarch64 宿主上交叉构建 `tennis` 时，必须提供来自目标 rootfs 的真实
AArch64 运行库目录，其中至少包含：

```text
libuvc.so  libusb-1.0.so  libturbojpeg.so  libudev.so.1
```

这些是 Ubuntu 22.04（Jammy）aarch64 的库文件：`libuvc0`、`libusb-1.0-0`、
`libturbojpeg0`、`libudev1` 提供运行库，对应 `-dev` 包提供 `.so` 名字（`libudev.so.1`
本身就带 SONAME，来自 `libudev1`）。构建示例：

```sh
AKA_RK3588_CROSS_LIB_DIR=/path/to/jammy-aarch64-libs \
  ./build_rk3588.sh -b Release -l WARN

# 等价写法
./build_rk3588.sh -b Release -l WARN -L /path/to/jammy-aarch64-libs
```

脚本在任何 `cmake` 调用之前校验该目录和所需文件；缺少目录或文件时直接失败并说明
原因，不会退回“生成空 stub”的方案。CMake 侧同样有构建期 guard：交叉配置缺少
`TARGET_RUNTIME_LIB_DIR` 时仍可配置并单独构建三个宿主测试目标，但只要真正构建
`tennis` 就会给出明确错误，绝不产出未解析符号的可执行文件。

链接使用 `libuvc`/`libusb-1.0`/`libturbojpeg` 这三个真实库的绝对路径，最终 `DT_NEEDED` 是库自身的 SONAME
（`libuvc.so.0`、`libusb-1.0.so.0`、`libturbojpeg.so.0`），由板卡上随包的
Jammy 运行库满足；不会链接 x86 宿主库，架构不匹配会在链接期以
`file in wrong format` 报错。

`libudev.so.1` 是 `libusb-1.0.so` 的传递依赖：链接器通过 `-rpath-link` 指向同一个
库目录来解析它，因此它必须存在于该目录，但不作为 `tennis` 的显式 `DT_NEEDED`。
缺失它时链接器会正确报错，这也是该目录的强制校验项之一。

安全门禁（禁止回退）：

- 交叉路径禁止生成空 `libuvc`/`libusb`/`libturbojpeg` stub，禁止 `stub_libs` 目标。
- 禁止 `--allow-shlib-undefined` 和 `--unresolved-symbols=ignore-all`；发布构建不允许
  任何未解析符号。
- 空 stub 方案会使 `uvc_*` 调用没有动态重定位，`UvcCapture::open` 经未解析 PLT 跳到
  UDF 触发 SIGILL，因此已彻底移除，不得以任何形式恢复。

### 10.3 x86 Linux 宿主测试

x86 Linux 宿主只应显式构建 virtual 测试目标。不要直接构建默认目标，因为默认目标
包含 AArch64 的 `tennis`，会尝试链接仓库中的 AArch64 `librknnrt.so` 并出现
`file in wrong format`。构建这三个宿主测试目标不需要交叉运行库目录：

```sh
cmake -S . -B /tmp/aka-vision-usb-tests -DBUILD_TESTING=ON
cmake --build /tmp/aka-vision-usb-tests \
  --target ftdi_protocol_test ft232_tty_selection_test
ctest --test-dir /tmp/aka-vision-usb-tests \
  -R '^(vision_usb_ci_launcher|ftdi_protocol|ft232_tty_selection|cross_link_policy)$' \
  --output-on-failure
```

`vision_usb_ci_launcher` 和 `cross_link_policy` 是直接运行的 Python 契约测试，不需要
单独构建可执行目标；`ftdi_protocol_test` 和 `ft232_tty_selection_test` 是宿主可执行
测试。`ft232_tty_selection_test` 只在 Linux 主机上配置。宿主测试只验证协议、选择、
判定、启动器契约和交叉链接策略，不能替代板卡上的真实 UVC、NPU、FT232R 和三种客户机
验收。`ftdi_protocol_test` 还会断言回环指令为 PING 且绝不等于 WRITE，并覆盖“raw
libusb serial 不可读时允许进入 TTY 严格选择”的回退策略；`cross_link_policy` 断言
CMake 交叉路径不再包含空 stub 或 unresolved 选项，并要求交叉库目录缺失时构建失败。

## 11. 覆盖边界

- 不覆盖模型准确率、固定图片语义、捡球/放球、机械臂/车轮动作、舵机反馈、急停或长时间稳定性。
- `usb` 和 `tty` 是互不替代的覆盖路径：`usb` 不证明 Linux `ftdi_sio`，`tty` 不证明 raw libusb 路径。
- FT232R 缺失、TXD/RXD 未短接、线缆断开、设备歧义或非 FT232R 串口都不能产生有效 PASS。
- AxVisor + Linux 的 `/guest/linux/orangepi-5-plus-6.1.99` 必须由统一资产流程部署并
  校验文件、版本和模块 ABI 后才有意义。
- 真实机器人闭环必须使用独立的真实分支/发布包，在已校准且安全停机的实机上另行
  测试，不属于本 virtual 分支或这三条 CI。

## 12. 关键文件

| 文件 | 作用 |
| --- | --- |
| `vision_usb_ci.cpp` / `vision_usb_ci.hpp` | 双窗口视觉 + FT232R 回环工作负载和最终判定 |
| `run_vision_usb_ci_once.sh` | 板卡 CI 启动器和 `APPLICATION_PASS`/`RESULT` 契约 |
| `tennis.cpp` | 最小 CLI，只分发 `vision-usb-ci` |
| `usb/ftdi_protocol.cpp` | Feetech/回环帧、状态字节剥离、设备选择、回环判定 |
| `usb/ft232_loopback.cpp` | libusb 厂商控制/bulk transport 和回环 worker |
| `usb/ft232_tty.cpp` | sysfs 精确映射到 `/dev/ttyUSBx` 的 termios 回退 |
| `usb/loopback_transport.hpp` | 两种 FT232R transport 的统一接口 |
| `CMakeLists.txt` | virtual `tennis` 目标与三个宿主测试目标/测试 |
| `cmake/require_target_runtime_libs.cmake` | 交叉构建真实 AArch64 运行库构建期 guard |
| `tests/cross_link_policy_test.py` | 交叉链接策略回归（禁止空 stub / unresolved 选项） |
