# Orange Pi 机器人 Zephyr 控制应用

本目录保存 RK3588 双客户机捡球方案中的 Zephyr 控制端。它接收 StarryOS 感知程序通过
AxVisor IVC 发送的结果，执行底盘决策、机械臂轨迹和安全停车，并通过 UART6 控制
Feetech 车轮和机械臂。

感知端、公共协议、控制端及验证边界的完整修改说明见仓库根目录
`AKA_RK3588_MODIFICATIONS.md`。

该应用不是 Zephyr Shell 中动态启动的用户程序。构建时它会与 Zephyr 内核、驱动和 Shell
静态链接为一个客户机镜像；AxVisor 启动 Zephyr 客户机后，应用的 `main()` 会自动运行。

## 功能

- 订阅 AxVisor IVC，接收 48 字节 `PerceptionResultV2`；
- 只处理当前队列中的最新感知结果，避免旧画面继续驱动机器人；
- 根据网球和球桶位置执行搜索、接近、抓取和放球状态机；
- 通过 UART6、1 Mbps Feetech 总线控制底盘 ID 7～9 和机械臂 ID 1～6；
- 机械臂使用独立的 50 ms 插值任务；
- 连续约 350 ms 没有有效输入时停止底盘；
- 提供 Zephyr Shell、心跳、控制帧率和诊断日志。

StarryOS 和 Zephyr 共用的通信结构定义在：

```text
../../protocol/perception_result_v2.h
```

修改消息布局时不要在两端分别复制结构体，应修改公共头文件并升级协议版本。

## 推荐目录布局

默认脚本假设 AKA 和 TGOSImages 是同级仓库：

```text
axvisor_two/
├── aka-rk3588/
├── ivc-sdk/
└── tgosimages/
```

其中：

- AKA 保存机器人感知、通信协议、控制状态机、参数和测试；
- TGOSImages 保存 Zephyr 版本、补丁、SDK、工具链、Orange Pi board 支持和镜像打包流程。
- ivc-sdk 保存 Starry/Zephyr 共用的 AXIVC v2 API、ring 实现和平台适配。

## 从 AKA 构建

在 AKA 根目录执行：

```bash
cd /path/axvisor_two/aka-rk3588
./scripts/build_zephyr_control.sh
```

脚本会优先使用同级 `../ivc-sdk`，不存在时获取最新默认分支。也可以显式指定：

```bash
./scripts/build_zephyr_control.sh \
  --tgosimages-dir ../tgosimages \
  --ivc-sdk-dir ../ivc-sdk \
  --image-name orangepi-robot-control-sdk
```

脚本按以下顺序查找 TGOSImages：

1. `--tgosimages-dir <路径>`；
2. 环境变量 `TGOSIMAGES_DIR`；
3. 同级目录 `../tgosimages`；
4. 以上都不存在时，clone TGOSImages 默认分支到 `../tgosimages`。

使用非同级目录：

```bash
./scripts/build_zephyr_control.sh \
  --tgosimages-dir /path/to/tgosimages
```

指定独立产物名称：

```bash
./scripts/build_zephyr_control.sh \
  --image-name orangepi-robot-control
```

已有 TGOSImages 工作树不会被脚本 pull、切换分支、reset 或清理；脚本会直接使用其中的
本地代码和构建缓存。首次构建可能需要下载 Zephyr 源码、Python 依赖和交叉工具链。

## 从 TGOSImages 构建

也可以从 TGOSImages 侧进入同一构建流程：

```bash
cd /path/axvisor_two/tgosimages
./scripts/apps/aka-rk3588-zephyr.sh
```

非同级目录可显式指定 AKA：

```bash
./scripts/apps/aka-rk3588-zephyr.sh \
  --aka-dir /path/to/aka-rk3588
```

两个入口最终构建的都是本目录，不存在两份 Zephyr 控制源码。

## 构建产物

默认产物位于 TGOSImages：

```text
IMAGES/orangepi/zephyr/orangepi-5-plus
IMAGES/orangepi/zephyr/orangepi-5-plus.elf
IMAGES/orangepi/zephyr/orangepi-5-plus.dtb
```

含义：

| 文件 | 用途 |
| --- | --- |
| `orangepi-5-plus` | Zephyr 客户机 BIN，供 AxVisor 加载 |
| `orangepi-5-plus.elf` | 带符号 ELF，用于反汇编和调试 |
| `orangepi-5-plus.dtb` | 本次 Zephyr 构建生成的设备树 |

确认构建的是正式控制应用：

```bash
strings ../tgosimages/IMAGES/orangepi/zephyr/orangepi-5-plus.elf \
  | rg ZEPHYR_ROBOT_CONTROL_START
```

## 运行

构建完成不等于已经部署。还需要由 TGOSKits/AxVisor 的 Orange Pi 双客户机配置把 BIN、
DTB、内存、vCPU、IVC 和 UART6 资源组装进板端启动镜像。

Zephyr 客户机启动后无需在 Shell 中再次执行控制程序。正常启动过程应包含：

```text
ZEPHYR_ROBOT_CONTROL_START protocol=2 source=axvisor-ivc uart=uart6
ZEPHYR_TIMER_READY ...
ZEPHYR_IVC_READY ...
```

StarryOS 侧随后运行：

```bash
./run_dual_pick.sh
```

Zephyr 默认每收到 60 条有效感知结果后，将接收频率、控制频率、latest-wins 统计和最新
识别结果合并输出，例如：

```text
ZEPHYR_CONTROL_STATUS messages=60 window_s=2.83 rx_fps=21.20 control_fps=21.20 coalesced=0 seq=... received=... processed=... invalid=0 state=search-ball ...
```

`window_s` 是收到这批消息实际经历的时间，不是固定 60 秒。通过
`CONFIG_ROBOT_STATUS_EVERY_MESSAGES` 修改每次统计的消息数，设置为 0 可关闭。独立的
`ZEPHYR_ROBOT_CONTROL_ALIVE` 默认关闭；需要无输入存活心跳时，将
`CONFIG_ROBOT_CONTROL_ALIVE_LOG` 改为 `y` 后重新构建。

主要日志：

| 日志 | 含义 |
| --- | --- |
| `ZEPHYR_IVC_READY` | IVC 订阅和共享区初始化完成 |
| `ZEPHYR_CONTROL_STATUS` | 每批有效消息的接收、控制、合并情况和最新感知结果 |
| `ZEPHYR_INPUT_WATCHDOG` | 感知输入超时，底盘已执行安全停车 |
| `ZEPHYR_ROBOT_CONTROL_FAILED` | UART、舵机或 IVC 初始化失败 |

## 运行控制测试

先至少成功运行一次 TGOSImages Zephyr 构建，以准备 Zephyr 源码和 Python 环境，然后执行：

```bash
cd /path/axvisor_two/tgosimages

ZEPHYR_BASE="$PWD/build/zephyr" \
ZEPHYR_TOOLCHAIN_VARIANT=host \
/tmp/zephyr-pyenv/bin/python build/zephyr/scripts/twister \
  -T ../aka-rk3588/zephyr/orangepi_robot_control/tests \
  -p native_sim/native/64
```

当前测试覆盖：

- 感知结果不等待机械臂周期即可驱动底盘决策；
- 机械臂插值只在独立周期内推进；
- 输入看门狗可重置且只触发一次；
- 输入超时停车后，新输入能够恢复控制。

这些 host 测试不覆盖真实 IVC、UART6、舵机供电、车轮方向或机械结构。

## 安全要求

首次部署新镜像或修改控制参数后，应先架空车轮，并确保机械臂活动范围内没有人员、线缆
和障碍物。只有看到 IVC 输入持续推进、控制帧率正常且输入中断后能够自动停车，才能继续
扩大测试范围。
