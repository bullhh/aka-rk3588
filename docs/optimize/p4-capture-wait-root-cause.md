# StarryOS `capture_wait` 约 24 ms 的原因

## 结论

`capture_wait` 不是 JPEG 解码、图像复制或摄像头曝光本身的耗时。它是用户程序在
`uvc_stream_get_frame()` 中等待 libuvc 返回一帧完整 MJPEG 数据的时间。

StarryOS 单独采集时，稳定窗口可达到 27.9～28.6 FPS；Linux 同时连续运行 NPU
推理时仍为 30.06 FPS。只有在 StarryOS 连续运行 RKNN/NPU 后，USB 完成帧的交付
被明显延迟，摄像头降到 11.73 FPS。每次推理后加入一次仅 1 微秒的 `usleep()`，
摄像头立即恢复到 29.66 FPS，同时 NPU 仍保持约 52 次/秒。

因此，当前直接原因是 StarryOS 的 RKNN/NPU 等待链持续忙等，没有给 USB 完成链
稳定的调度机会；不是 USB 带宽、摄像头标称帧率或 NPU 算力不足。

## 测试矩阵

测试均使用同一摄像头、`640×480 MJPEG @ 30 FPS` 和同一个 640×480 RKNN 模型。
StarryOS 隔离测试把采集主线程固定在 CPU1，测试 NPU 线程固定在 CPU7。

| 环境与负载 | 摄像头帧率 | NPU 频率 | NPU 平均耗时 |
| --- | ---: | ---: | ---: |
| Linux，仅 UVC | 30.00 FPS | - | - |
| Linux，UVC + 连续 NPU | 30.06 FPS | 62.8 次/秒 | 15.94 ms |
| StarryOS，仅 UVC | 稳定窗口 27.9～28.6 FPS | - | - |
| StarryOS，UVC + 连续 NPU | 11.73 FPS | 54.9 次/秒 | 18.13 ms |
| StarryOS，每次 NPU 后 `usleep(1)` | 29.66 FPS | 52.0 次/秒 | 18.13 ms |
| StarryOS，每次 NPU 后 `usleep(1 ms)` | 30.32 FPS | 30.9 次/秒 | 17.67 ms |
| StarryOS，每次 NPU 后 `usleep(2 ms)` | 30.39 FPS | 30.4 次/秒 | 17.73 ms |
| StarryOS，每次 NPU 后 `usleep(5 ms)` | 30.24 FPS | 30.2 次/秒 | 17.79 ms |
| StarryOS，每次 NPU 后 `usleep(10 ms)` | 30.19 FPS | 31.9 次/秒 | 18.98 ms |

1 微秒已经足以恢复帧率，说明起作用的是“发生一次阻塞和重新调度”，而不是需要
额外等待若干毫秒。若是 DDR、USB 或 DMA 带宽不足，1 微秒休眠不可能产生这种
幅度的恢复。

NPU 分阶段统计也表明，18.13 ms 中主要是 `rknn_run` 的 17.34 ms；输入同步约
0.28 ms，输出约 0.12 ms，后处理约 0.03 ms。性能问题不在预处理或后处理。

## 源码与运行时证据

### 1. 用户程序采用同步取帧

`capture/uvc_capture.cpp` 的 `getFrame()` 直接调用：

```cpp
uvc_stream_get_frame(strmh_, &frame, timeout_ms * 1000);
```

`capture_wait` 测量的是该调用返回前的时间。随后 `memcpy()` 单独记入
`capture_copy`，所以两者不会混在一起。

摄像头和推理在主业务循环中仍是串行的：取帧、解码、推理、控制，然后再取下一帧。
libuvc 内部虽然有 `libusb_event` 线程异步处理 USB 事件，但程序没有独立的用户态
采集队列。因此 USB 完成事件交付变慢会直接表现为下一次 `capture_wait` 增大。

### 2. StarryOS NPU 驱动同步忙轮询

`drivers/npu/rockchip-npu/src/ioctrl.rs` 的提交路径在内核中循环读取三个 NPU 核心的
完成状态；没有进展时只执行 `spin_loop()`，不会睡眠或进入等待队列：

```rust
while states.iter().any(|state| state.inflight) {
    // poll_core_completion(...)
    if !progressed {
        spin_loop();
    }
}
```

同时，`drivers/npu/rockchip-npu/src/data/mod.rs` 中 RK3588 的三个 NPU 中断处理函数
目前都返回 `None`。也就是说，当前实现没有使用“中断唤醒等待任务”的完成机制。

### 3. RKNN 运行时还会创建忙等辅助线程

实机观察一次 UVC + NPU 测试进程可见四个线程：

```text
主采集线程
libusb_event
RKNN 运行时内部线程
测试 NPU 线程
```

连续推理时后两个线程长期处于 `R` 状态，系统态 CPU 占用显著。任务虽然有 8 个
CPU 可用，但 StarryOS 当前的线程亲和性并未完整继承：主线程经 `taskset -c 1`
限制后，新建的 `libusb_event` 和 RKNN 内部线程仍显示 `0-7`。此外，当前
`sched_setaffinity` 无法用线程 ID 从外部重新绑定这些线程。这使用户态很难可靠地
把 USB 与 RKNN 辅助线程完全隔离。

普通用户态忙循环也验证了 CPU0 对当前 USB 完成链较敏感：忙循环放在 CPU0 时，
UVC 只有 13.18 FPS；放在 CPU7 时仍为 29.55 FPS。不过，即使将显式 NPU 线程和
采集线程分开，连续 NPU 仍会降低帧率，说明只调整外层线程亲和性并不足以解决问题。

## `capture_wait` 为什么约为 24 ms

摄像头标称周期约为 33.3 ms。正常情况下，USB/libuvc 在后台持续接收，主循环完成
约 18 ms 推理后，下一帧已经部分到达，所以只需等待剩余时间。

在 StarryOS 中，NPU 提交及 RKNN 辅助线程持续忙等，USB 事件和帧完成回调不能按
摄像头节拍及时运行。主线程进入 `uvc_stream_get_frame()` 后，需要继续等待积压的
USB 完成处理，最终平均形成约 24 ms 的等待。它表示“完整帧交付延迟”，不表示
摄像头传输一帧固定需要 24 ms。

## 后续修复建议

稳定方案应优先放在 StarryOS，而不是长期依赖用户程序休眠：

1. 为 RK3588 NPU 实现中断完成处理，用等待队列阻塞提交任务，替换
   `submit_ioctrl()` 中持续 `spin_loop()` 的同步轮询。
2. 修正线程 CPU 亲和性的继承与按 TID 设置能力，确保新建线程继承调用者掩码，
   并允许将 libusb 与 RKNN 辅助线程分核。
3. 并发采集/NPU测试中，推理线程一次极短的阻塞可以恢复摄像头帧率；但后续串行
   A/B 已证明当前正式流程加入休眠没有有效提升，因此当前保持0微秒。详细结果见
   `p5-starry-npu-yield-test.md`。

本轮仅定位原因，没有把休眠写入正式捡球流程，也没有修改 StarryOS 内核。

## 原始日志

- `p4-linux-uvc-only-20260729.log`
- `p4-linux-uvc-plus-npu-20260729.log`
- `p4-starry-uvc-only-20260729.log`
- `p4-starry-gap0-20260729.log`
- `p4-starry-gap1us-20260729.log`
- `p4-starry-gap1ms-20260729.log`
- `p4-starry-gap2ms-20260729.log`
- `p4-starry-gap5ms-20260729.log`
- `p4-starry-gap10ms-20260729.log`
- `p4-starry-uvc-plus-user-busy-20260729.log`
- `p4-starry-uvc-plus-user-busy-cpu7-20260729.log`
