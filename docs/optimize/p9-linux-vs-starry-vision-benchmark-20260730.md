# Linux 与 StarryOS 真实视觉链路对比（2026-07-30）

## 测试条件

- 同一台 Orange Pi 5 Plus 三足机器人；
- 同一份 ARM64 用户态二进制：`93b8b49d73b3614ec62dbcb0c5d0ea3e82e23571f223e8087ab03b22b99f8e88`；
- 同一份 640×480 ReLU RKNN 模型：`2eee3c422fb9f7fdeea5b9db5df047ac4f6d9b8047f9b654f75c3cb012db0081`；
- 摄像头 640×480 MJPEG 30 FPS，固定使用 NPU core 0；
- 每个系统先预热摄像头 10 帧和完整推理 10 帧，再统计 4 个约 10 秒的稳态窗口；
- 测试链路为 UVC 取帧、MJPEG 解码、RKNN 输入、`rknn_run`、输出和后处理；不初始化车轮和机械臂。

测试使用的临时命令和统计代码已经删除，机器人已重新编译为正式程序。

## 原始稳态窗口

Linux：

```text
frames=301 drops=0/0 effective=30.00fps wait=15.13ms jpeg=5.73ms run=12.04ms total=33.33ms
frames=300 drops=0/0 effective=29.99fps wait=15.45ms jpeg=5.42ms run=12.05ms total=33.35ms
frames=300 drops=0/0 effective=29.99fps wait=15.67ms jpeg=5.22ms run=12.03ms total=33.34ms
frames=301 drops=0/0 effective=30.02fps wait=15.78ms jpeg=5.13ms run=12.00ms total=33.32ms
```

StarryOS：

```text
frames=158 drops=0/16 effective=15.78fps wait=30.41ms copy=2.369ms jpeg=9.30ms run=16.52ms total=59.50ms
frames=162 drops=0/10 effective=16.16fps wait=30.05ms copy=4.157ms jpeg=9.31ms run=14.96ms total=59.39ms
frames=172 drops=0/9  effective=17.20fps wait=29.42ms copy=1.447ms jpeg=9.30ms run=15.47ms total=56.54ms
frames=149 drops=0/12 effective=14.89fps wait=31.78ms copy=6.112ms jpeg=9.30ms run=16.96ms total=65.06ms
```

`drops=取帧失败数/MJPEG 解码失败数`。正式程序同样会跳过损坏帧并继续运行。

## 加权结果

| 指标 | Linux | StarryOS | 差异 |
| --- | ---: | ---: | ---: |
| 有效处理帧率 | 30.01 FPS | 16.01 FPS | StarryOS 低 46.6% |
| `rknn_run` | 12.03 ms | 15.95 ms | StarryOS 高 32.6% |
| 取帧等待 | 15.51 ms | 30.37 ms | StarryOS 高 95.8% |
| MJPEG 解码 | 5.38 ms | 9.30 ms | StarryOS 高 73.1% |
| 整帧时间 | 33.34 ms | 59.97 ms | StarryOS 高 79.9% |
| MJPEG 损坏帧 | 0/1202 | 47/688 | StarryOS 约 6.8% |

## 结论

Linux 当前真实摄像头链路中的 `rknn_run` 是约 **12.03 ms**；StarryOS 是约
**15.95 ms**，并不是固定的 26.41 ms。固定输入测试曾得到 Linux 11.98 ms、
StarryOS 13.79 ms，因此接入真实 UVC 后 Linux 几乎不变，而 StarryOS 增加约
2.16 ms。`rknn_run` 统计的是系统调用的墙钟时间，不是 NPU 硬件内部的纯执行时间，
会包含 StarryOS NPU 驱动等待、忙轮询期间的调度延迟，以及与 USB 后台任务竞争造成的
暂停。

此前正式程序的 26.41 ms 是完整机器人程序特定运行窗口的观测值，不能代表 NPU 固定
性能。本次同模型、同摄像头链路没有复现该数值，只能说明移除底盘、机械臂和正式状态机
负载后，视觉组件本身可以达到上述性能。

随后用串口前台方式复测正式流程。Linux 三次均约 30 FPS；StarryOS 冷启动首个
窗口约 7～9 FPS，随后稳定在约 16.05 FPS，稳定窗口的 `rknn_run` 约 15.7 ms，
与本页组件隔离基准基本一致。此前明显更低的结果来自把高频日志重定向到 StarryOS
板端 ext4，受 UVC 与存储并发停顿污染，不能代表正常前台运行。详见
[P10 正式流程前台复测](./p10-linux-vs-starry-full-flow-20260730.md)。

在本隔离测试中，更大的系统差距不只在 NPU：StarryOS 每帧比 Linux 多约 14.86 ms
取帧等待、3.93 ms JPEG 解码，并出现 47 个 MJPEG 损坏帧。正式流程的稳态数据也
保持这一量级；不运行 NPU 的找桶阶段仍出现较高 JPEG 和调度耗时。后续应同时处理
StarryOS USB/UVC 完整帧交付、后台事件调度、JPEG CPU 执行效率和找桶重复解码，
不能只优化 RKNN 模型。
