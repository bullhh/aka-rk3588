# P0 Linux 与 StarryOS 性能基线对比

## 测试结论

Linux 和 StarryOS 均完成了一次真实的“找球、夹球、找桶、放球”闭环，功能正确。两次测试使用完全相同的用户态程序、模型和机械臂配置。

StarryOS 使用 `Warn` 日志级别测试。后续 StarryOS 性能测试也统一使用 `Warn`，不再使用 `Info` 数据作为基线。

| 指标 | Linux | StarryOS Warn | 差异 |
| --- | ---: | ---: | ---: |
| 稳态窗口总时长 | 20.03 s | 91.01 s | Starry 采样更长 |
| 摄像头帧率 | 29.87 FPS | 7.83 FPS | Starry 为 Linux 的 26% |
| 有效处理帧率 | 26.31 FPS | 6.32 FPS | Starry 为 Linux 的 24% |
| 覆盖帧比例 | 11.7% | 19.2% | Starry 更高 |
| 单帧平均耗时 | 37.99 ms | 157.79 ms | Starry 约 4.15 倍 |
| 单帧 P50 | 约 37.2 ms | 约 134.4 ms | Starry 约 3.61 倍 |
| 单帧 P95 | 37.3～41.4 ms | 平均约 261.6 ms | Starry 抖动明显 |
| JPEG 解码 | 3.47 ms | 8.30 ms | Starry 约 2.39 倍 |
| RKNN 推理 | 33.56 ms | 88.24 ms | Starry 约 2.63 倍 |
| UVC 回调复制 | 约 0.005 ms | 3.58 ms | Starry 回调存在明显调度延迟 |
| 摄像头等待 | 约 0.20 ms | 47.91 ms | Starry 采集供帧不足 |

## 相同测试条件

```text
程序 SHA-256: fc6b2bba684640c36d14bbc9a4d7a0052895cb308f91df878f629ae771aee2e9
模型 SHA-256: 99534a0883312ef06115675a3dbf6dd2504d56fb0e1466a354081300f1370e1a
配置 SHA-256: b8da2a9ff762ac187e1cca06937d7e5d71aab815e4df24596905c99bdc23e994
摄像头: 640×480 MJPEG @ 30 FPS
模型: 640×640 INT8
NPU core mask: RKNN_NPU_CORE_0_1_2
可见 CPU: 8
状态日志间隔: 1000 ms
性能窗口: 10 s
```

StarryOS 构建信息：

```text
tgoskits commit: 9993bc1eacf101d4320dfb8c6e1d8ef94677090b
StarryOS bin SHA-256: 20f4b1c8fe0fe5a948cdcc0cab4b0c79c663b74a47aadf85744d12204f31614e
AX_LOG: warn
SMP: 8
```

## StarryOS 稳态数据

StarryOS 在放球完成后连续采集了 9 个稳态窗口：

```text
总时长       91.01 s
摄像头帧数   713
处理帧数     575
覆盖帧数     137
摄像头帧率   7.83 FPS
有效帧率     6.32 FPS
单帧平均     157.79 ms
RKNN 推理    88.24 ms
JPEG 解码    8.30 ms
```

完整数值见：[StarryOS Warn 关键日志](./p0-baseline-starry-warn-640x640-20260728.log)。Linux 数据见：[Linux 性能基线](./p0-performance-baseline.md)。

## 当前瓶颈判断

StarryOS 的差距不是单一环节造成的：

1. 摄像头实际回调只有约 7.83 FPS，远低于请求的 30 FPS。主循环平均等待新帧约 47.91 ms，UVC 回调复制本身也出现毫秒级调度延迟。
2. 同一 RKNN 模型在 StarryOS 下平均推理约 88.24 ms，Linux 为 33.56 ms，说明 StarryOS 的 NPU 驱动、时钟、内存同步或调度路径仍有明显开销。
3. JPEG 解码从 Linux 的 3.47 ms 增至 8.30 ms，说明 CPU 执行效率或调度也存在差距。
4. 将内核日志降为 `Warn` 后，高频逐次 RKNPU ioctl 日志已经消失，但摄像头和 NPU 两个主要瓶颈依然存在，不能只靠减少日志解决。

因此下一轮应先分别测量 UVC-only、RKNN-only 和完整流程，不应立即把所有差距归因于用户态模型尺寸。

## 后续测试约定

- StarryOS 一律使用 `Warn` 构建。
- 性能日志直接写入持久目录，不先写 `/tmp`。
- 程序退出后执行 `sync`，确认文件大小和 SHA-256 后再重启。
- 每种优化至少采集 6 个无机械臂动作的 10 秒窗口。
- 机械臂动作窗口只用于功能和卡顿观察，不与视觉稳态窗口求平均。
