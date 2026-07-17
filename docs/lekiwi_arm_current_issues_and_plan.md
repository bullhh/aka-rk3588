# LeKiwi 当前状态、问题与计划

## 1. 当前结论

三轮机器人基本功能已跑通。Feetech userspace libusb 单播写 ACK 残留问题已经修复，
Linux TTY、Linux libusb 和 Starry libusb 均已通过20轮实机压力测试。通信错误传播
也已修复。机械臂第一阶段轨迹已经改为固定控制周期、关节空间平滑回 HOME，并避开
GRAB 到 LIFT 之间的奇异区；下一步是现场夹球和完整闭环验证。

| 模块 | 状态 | 结论 |
| --- | --- | --- |
| 电机 ID 1-9、三轮底盘 | 已实现 | 可用 |
| Linux TTY | 已实现 | 稳定 |
| Linux/Starry libusb CDC | 写 ACK 已修复 | 20轮压力测试通过 |
| Starry USB 跨进程重开 | 已修复 | 可用 |
| 校准、待机姿态、夹爪 | 已实现 | 可用 |
| 独立 `ik-pick` | 通信与13阶段轨迹通过 | Linux/Starry动作节奏一致 |
| UVC + 单核 NPU | 已实现 | 可用 |
| 三核 NPU | 已实现驱动 | bbox 不可靠，禁用 |
| 完整追球、抓取、放球 | 待验证 | 通信和独立动作已具备条件 |

## 2. 已实现并验证

### Feetech、机械臂和底盘

- `scan` 找到 ID 1-9，`read` 能读取位置、速度、电压和温度。
- `scan -> read` 跨进程连续执行成功。
- ID 1-9 的 `RESPONSE_LEVEL` 均为 1；单播写会返回状态 ACK。
- `write_reg()` 已通过统一的 `tx_rx()` 读取并校验 ACK，不再把 ACK 留给下一个请求。
- Linux TTY、Linux 强制 libusb 和 Starry libusb 的 `ik-pick` 各连续20轮通过。
- Starry `scan -> read` 跨进程连续20轮通过；每轮找到9个电机并读取9条状态。
- 上述测试均为退出码0、`done=1 failed=0`，无 timeout、checksum mismatch、错 ID、
  short read 或 `LIBUSB_ERROR_BUSY`。
- 已删除初始化位置和夹爪验证的目标角度 fallback；轨迹写入失败会设置 controller
  failed。协议模拟验证初始读取、动作写入、夹爪验证三种故障均返回退出码1。
- 完整闭环遇到机械臂通信失败会停车、清理资源并以非零退出，不再回到追球状态继续运行。
- 读取和写入使用互逆的关节标定，目标角度、反馈角度和舵机角度处于同一坐标系。
- 任意起始姿态先以低增益缓升方式在关节空间回 HOME，不再沿危险笛卡尔直线回位。
- 夹球后先回到 PRE_GRAB 高度，再移动到 LIFT，避免经过二连杆原点附近的奇异区。
- LIFT 期间腕部从抓取角度连续插值到运输角度，不再超过范围后反向跳变。
- 抓取点已从 `(0.09,-0.08)` 调整到 `(0.12,-0.06)`，减小肩肘折叠和腕部负角度。
- MOVE_TO 和腕部阶段现在等待实际关节反馈收敛；小于6°的非夹爪残差直接发送最终目标，
  单阶段100周期仍未收敛会输出各关节残差并返回失败。
- 独立动作固定约20 Hz，GAP约300 ms；Linux TTY、Linux libusb 和 Starry libusb
  已通过；反馈收敛版本 Linux 连续3轮、Starry 1轮通过，单轮约10.7-10.9秒。
- `calib-check`、`pos`、夹爪 60°/0°、`torque-off` 成功。
- `test-base auto stop` 成功；方向运动需确认机器人架空后再测。
- 已有校准、IK、HOME/PRE_GRAB/GRAB/LIFT 和运输姿态。

### Starry USB

- 已实现 `USBDEVFS_CLEAR_HALT`，解决重开接口后的 data toggle 失配。
- 已实现 usbfs 文件关闭自动释放接口，解决 Ctrl-C/kill 后永久
  `LIBUSB_ERROR_BUSY`。
- 实机通过：`scan -> read`，以及 `claim -> kill -9 -> claim`。

### 视觉

以下底层命令在 Starry 下成功，能生成 `capture.jpg` 和 `result.jpg`：

```bash
RKNN_CORE_MASK=0 ./build/tennis test-yolo models/tennis.rknn 0
```

## 3. 未解决问题和解决方案

| 优先级 | 问题与证据 | 解决方案 |
| --- | --- | --- |
| P0 | Starry xHCI 异步 URB 取消不完整，取消的 IN URB 可能吞掉下一次回复 | 实现 Stop Endpoint、Set TR Dequeue Pointer、必要的 Reset Endpoint 和 TRB 回收；增加取消后再次收发测试 |
| P1 | `run_vision_once.sh` 在 Starry 报 `pipefail` 错误，但底层视觉正常 | 改为 POSIX `sh` 或明确 Bash；加入 `is_starry()`；Starry 只运行已有二进制 |
| P1 | 缺少 `config/lekiwi_arm_poses.txt`，`pose-list` 失败 | 提交实机确认的默认姿态，或用 `pose-save` 生成 |
| P2 | 还没有用球验证夹取点和夹爪力度 | 放置固定位置网球连续测试，必要时只微调 `grab_x/grab_y` |
| P2 | 还没有运行优化后的完整视觉闭环 | 先架空运行，再在开阔场地低速运行并录像 |
| P3 | Starry 三核 NPU bbox 不可靠 | 当前固定 `RKNN_CORE_MASK=0`，三核问题单独修复 |

## 4. 通信 P0 的稳定实现要求

### 必须做到

1. 同一时刻只有一个未完成协议请求。
2. 单播读和单播写都消费完整状态包，并校验 ID、长度、error 和 checksum。
3. 广播写明确不等待回复。
4. timeout、checksum 或错 ID 只能有限重试，超过阈值立即失败。
5. 通信失败必须传播到动作状态机和进程退出码。
6. Linux TTY、Linux libusb、Starry libusb 使用同一套协议语义和错误策略。

### 不能采用

- 恢复每条命令前 2 ms 的 `drain_usb_input()`。
- 增加 drain 次数或用更短超时碰运气。
- 无限丢弃错 ID，直到偶然收到目标 ID。
- checksum 错误后继续使用当前数据。
- 始终使用目标角度 fallback，并仍报告动作成功。
- 只给 Starry 增加应用层特判，让 Linux libusb 保留另一套行为。

## 5. 实施顺序

1. `[已完成]` 修复单播写 ACK 消费和同步事务语义。
2. `[已完成]` 三后端各20轮通信压力测试。
3. `[已完成]` 让通信错误正确传播到动作控制器和退出码。
4. `[下一步]` 完善 Starry xHCI URB 取消语义。
5. 修复视觉脚本，补充默认姿态文件。
6. `[已完成]` 固定机械臂约20 Hz控制和约300 ms GAP。
7. `[已完成]` 修正标定、HOME、CLEAR、LIFT和腕部连续轨迹。
8. `[下一步]` 用真实网球验证夹取，再运行安全追球和完整抓取。

建议分开提交：

```text
fix(feetech): consume unicast write acknowledgements
fix(lekiwi): propagate communication failures
fix(starry): complete xhci urb cancellation
fix(starry): run vision-only script without rebuilding
fix(lekiwi): run arm actions at a stable control rate
tune(lekiwi): adjust arm poses and gains
```

## 6. 验收标准

通信压力测试已经达到以下基线，日志中没有 timeout、checksum mismatch、
unexpected status ID 或 `LIBUSB_ERROR_BUSY`：

```text
[通过] Linux auto/TTY：       ik-pick 连续20次
[通过] Linux 强制 usb：       ik-pick 连续20次
[通过] Starry usb/libusb：    ik-pick 连续20次
[通过] Starry：               scan -> read 连续20轮
[通过] Starry：               Ctrl-C/kill 后立即重新 scan
```

通信通过后，机械臂第一阶段还需满足：

- 完整流程不再跟随 2-3 FPS 视觉帧率卡顿。
- GAP 稳定在 300-400 ms。
- Linux 和 Starry 动作节奏基本一致。
- 连续 20 次 `ik-pick` 无通信错误、撞限位、异常停顿或控制器失败。
- 夹爪能稳定夹球，抬升时不碰底盘和相机支架。

只有以上项目通过，才能架空运行安全追球和完整抓取流程。
