# P3 StarryOS UVC 帧率优化结果

## 结论

2026-07-28 在 `10.3.10.60` 管理的三足机器人上完成 Linux 和
StarryOS 实机对比。低帧率不是摄像头 USB 传输或 NPU 单一环节导致，
而是两个可独立复现的问题：

1. StarryOS 下 libuvc 内部回调分发线程只向程序交付约 11 FPS，
   但 USB/UVC 底层实际每秒已接收 30 个完整 MJPEG 帧。分发线程来不及
   消费时，libuvc 的单帧缓冲会被新帧覆盖。
2. 摄像头允许自动曝光主动降低帧率。Linux 下实测开启
   `exposure_dynamic_framerate` 时只有约 10 FPS，关闭后立即恢复 30 FPS。

最终仅保留用户态有效修改：

- 使用 `uvc_stream_get_frame()` 同步取帧，绕过 StarryOS 下失速的
  libuvc 用户回调线程。
- 打开摄像头时设置 UVC AE priority 为 0，保留自动曝光，但不允许
  为延长曝光时间而降帧。
- 机械臂动作前停流并关闭旧 stream handle；动作后重建 handle，
  避免长时间暂停后复用旧 URB/端点状态。
- 连续状态日志严格按时间间隔限流，避免 `BALL_LEFT` 与
  `BALL_FINE_LEFT` 交替时退化为逐帧打印。

## 实测结果

| 场景 | 优化前 | 优化后 | 结果 |
| --- | ---: | ---: | --- |
| StarryOS，640×480 完整视觉链路 | 稳态 8.38 FPS | 稳态 13.1～13.5 FPS | 约提升 60% |
| StarryOS，UVC-only | 回调约 11 FPS | 27.6 FPS | 接近 30 FPS 设定值 |
| Linux，UVC-only | 30 FPS | 29.8 FPS | 无性能回归 |
| StarryOS，暂停 20 s 后恢复 | 复用旧 handle 曾黑窗约 35 s | 27.8 FPS | 重建 handle 后立即恢复 |

完整视觉链路稳态窗口中，JPEG 解码约 8.5～8.7 ms，RKNN 运行约
35～37 ms，摄像头等待约 22～24 ms。程序仍是串行的“取帧→解码→推理
→控制”，因此 UVC-only 提高到约 28 FPS 不代表完整链路也会达到
28 FPS；本次优化的实际闭环收益以 `effective_fps` 为准。

## 机械臂暂停/恢复验证

因车身已架起且无法真实夹球，测试时仅临时放宽停车尺寸和中心
容差，触发一次完整夹球动作，测试后已恢复稳定配置：

```text
[UvcCapture] paused before PICK_BALL in 598.7 ms
...
[UvcCapture] resumed after PICK_BALL in 709.4 ms
[GAME] PICK_BALL done ... ball_visible=yes ... post_fps=11.7
```

恢复后首帧立即完成解码和识别，随后稳态窗口为 13.50 和 13.47 FPS。
夹球结果为失败是测试条件中车身不移动且球位非真实夹取位置所致，
本次用于验证 UVC 与机械臂 USB 切换，不用于评价夹球准确率。

## 调试命令

```bash
# UVC-only：预热 10 帧后统计 10 秒
AKA_UVC_BENCH_SECONDS=10 ./build/tennis test-uvc 0

# 验证长时间暂停后恢复
AKA_UVC_BENCH_SECONDS=10 \
AKA_UVC_BENCH_RESUME=1 \
AKA_UVC_BENCH_PAUSE_SECONDS=20 \
./build/tennis test-uvc 0

# 完整链路；StarryOS 性能测试使用 Warn 构建
AKA_STATE_LOG_INTERVAL_MS=3000 ./run_lekiwi_full.sh
```

原始记录：

- [Linux UVC-only](./p3-final-linux-uvc-sync-20260728.log)
- [StarryOS UVC-only](./p3-final-starry-uvc-sync-20260728.log)
- [StarryOS 完整视觉与车轮控制](./p3-final-starry-warn-640x480-chase-20260728.log)
- [StarryOS 模拟夹球与 UVC 恢复](./p3-final-starry-simulated-pick-20260728.log)

## 已撤销的无效尝试

- 调整 xHCI 周期端点 interval 编码后，URB 完成频率变化，但完整
  MJPEG 仍为 30 帧/秒，UVC-only 和闭环帧率无收益，已撤销。
- 修改 xHCI Block Event Interrupt 策略后帧率无变化，已撤销。
- 单独采集线程原型在 Linux 下不交付帧且退出可阻塞，已完整撤销。

tgoskits 最终没有保留本次调试产生的 USB/xHCI 修改或诊断日志。
