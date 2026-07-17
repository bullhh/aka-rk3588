# LeKiwi C++ 闭环迁移说明

本文记录把 Desktop-Wanderer Python 控制流程迁移到当前 C++ 工程后的设计与测试方法。

> 注：本文后半部分保留了早期 `grab_x/grab_y` 等参数名用于说明迁移历史。当前可调
> 参数已经改为 `grab_id1_deg`～`grab_id5_deg` 基础姿态和三个厘米偏移；实际调试请以
> `config/lekiwi_pick_config.txt` 内的逐项注释及 `docs/lekiwi_user_manual.md` 为准。

## 目标

当前工程需要同时运行在 Linux 和 StarryOS，因此主流程仍保留 C++ 实现。Desktop-Wanderer 只作为控制策略参考，不直接引入 Python 运行时。

本次优化重点：

- 追球距离和角度改为目标框视觉伺服。
- 机械臂抓取改为逆运动学 + P 控制。
- 抓取后读取夹爪反馈判断是否真的夹住。
- 抓取失败时回到找球，不再直接认为成功。
- 原始位置姿态录制功能保留为调试工具，不作为主抓取策略。

## 追球/找桶距离判断

Python 版本不是用真实距离，也不是单纯用检测框面积，而是定义画面中央目标框：

- 画面 640x480 时，目标框宽约 150，高约 160。
- 球中心在目标框左侧则左转。
- 球中心在目标框右侧则右转。
- 球在目标框内但尺寸偏小则前进。
- 球在目标框内但尺寸偏大则后退。
- 球中心和尺寸都稳定 10 帧后进入抓取。

C++ 中对应实现：

```text
robot/lekiwi_task_controller.hpp
robot/lekiwi_task_controller.cpp
LeKiwiMoveController
```

主循环接入点：

```text
tennis.cpp
GameState::CHASE_BALL -> GameState::PICK_BALL
GameState::FIND_BUCKET -> GameState::PUT_BALL
```

## 机械臂抓取

Python 版本的 `CATCH_ACTION` 已迁移到 C++：

```text
move_to(0.0989, 0.125)
shoulder_pan -12
gripper +60
wrist_flex 80
move_to(0.140, 0.1211)
move_to(0.140, -0.05)
gap
gripper -60
gap
shoulder_pan +12
move_to(-0.1, 0.2)
wrist_flex -20
```

C++ 通过 `inverse_kinematics(x, y)` 计算：

```text
arm_shoulder_lift
arm_elbow_flex
arm_wrist_flex = -shoulder_lift - elbow_flex + pitch
```

Linux Python 版本每帧读取当前关节角度，按 P 控制写入新的目标角度。C++ 在 StarryOS 下为降低 USB CDC 读压力，只在动作开始时读取一次真实关节位置，之后用上一帧已发送位置作为内部当前值继续做 P 控制；这样轨迹仍是连续收敛，不会每 tick 高频读取 6 个关节。

对应实现：

```text
LeKiwiArmController
FeetechArm::get_joint_deg()
FeetechArm::write_degrees()
```

## 抓取失败重试

抓取动作完成后，C++ 会读取 `arm_gripper` 当前位置，并立即重新取一帧图像跑 RKNN 复核球是否仍在视野内：

```text
gripper_hold = arm_gripper > 25
ball_visible = 抓取后这一帧仍能检测到球
holding      = gripper_hold && !ball_visible
```

如果第一次没夹住，程序不会立刻去找桶。当前策略是：

```text
夹取失败
-> 若 ball_visible 且仍为 BALL_READY，直接使用下一组偏移再次 PICK_BALL
-> 若 ball_visible 但不再满足抓取条件，回到 CHASE_BALL 重新视觉伺服对准球
-> 若视觉和夹爪都不能确认持球，回到 CHASE_BALL 继续找球
```

自动偏移顺序为：

```text
(0, 0)
(-0.005, 0)
(-0.010, 0)
(-0.015, 0)
(-0.020, 0)
(+0.010, 0)
(0, -0.010)
(0, +0.010)
(-0.015, -0.010)
(-0.015, +0.010)
```

偏移含义：

```text
第一个值：同时加到 pre_grab_x 和 grab_x
第二个值：同时加到 pre_grab_y 和 grab_y
```

如果某次偏移夹住了球，程序会把成功参数写回：

```text
config/lekiwi_pick_config.txt
```

如果夹住：

```text
PICK_BALL -> FIND_BUCKET
```

如果所有偏移都失败：

```text
PICK_BALL -> CHASE_BALL
```

也就是放弃当前偏移序列，重新从第一组参数开始视觉对准和抓取。

抓取完成瞬间会用夹爪反馈判断是否夹住。进入找桶后不再持续复检夹爪，避免夹爪受力或通信瞬时异常导致误判掉球并退出找桶。

## 投放

Python 版本的 `PUT_ACTION` 已迁移：

```text
shoulder_lift +50
gap
gripper +60
gap
move_to(-0.1, 0.2)
gripper -60
```

桶进入目标框且稳定后：

```text
FIND_BUCKET -> PUT_BALL -> CHASE_BALL
```

## 测试建议

先确认单项功能：

```bash
cd /home/orangepi/robot/aka-rk3588
./build/tennis test-new-arm /dev/ttyACM0 calib-check
./build/tennis test-feetech /dev/ttyACM0 read
./build/tennis test-yolo models/tennis.rknn 0
./build/tennis test-bucket 0
```

再运行完整闭环：

```bash
./run_lekiwi_loop.sh
```

观察日志：

```text
LEKIWI_CHASE      追球视觉伺服
-> PICK_BALL      开始 IK 抓取
PICK_BALL done    抓取完成并打印 gripper 位置
grab failed       抓取失败，回到追球
-> FIND_BUCKET    抓取成功，开始找桶
LEKIWI_BUCKET visible=0 label=BUCKET_SEARCH  视野里没有桶，旋转搜索
-> PUT_BALL       桶到位，开始投放
PUT_BALL done     投放完成，回到追球
```

## 注意

如果机械臂方向仍明显错误，优先检查校准文件和关节方向映射，而不是录制原始位置姿态。原始位置姿态现在只用于排查硬件方向、范围和夹爪开闭，不是最终自动夹取方案。

## 抓取位置调试

抓取几何参数放在：

```text
config/lekiwi_pick_config.txt
```

该文件用于在不重新编译代码的情况下调整抓取动作和停车距离。程序启动或执行 `ik-pick` 时会读取这个文件；修改文件后，重新运行命令即可生效。

### 参数作用

```text
home_x / home_y
```

机械臂抓取动作开始时的参考位置。通常不需要优先调整，除非初始姿态本身明显不合理。

```text
pre_grab_x / pre_grab_y
```

夹爪下探前的预抓取位置。它决定机械臂从初始姿态移动到球附近时的中间点。调 `grab_x` 时通常同步调整 `pre_grab_x`，避免机械臂轨迹突兀。

```text
grab_x / grab_y
```

真正闭合夹爪时的位置，是最关键的抓球参数。

- 夹爪伸过球、在球前方闭合：减小 `grab_x`。
- 夹爪还没够到球：增大 `grab_x`。
- 夹爪太高：减小 `grab_y`。
- 夹爪太低：增大 `grab_y`。

建议每次只改 `0.005` 米，避免一次改太多导致判断困难。

```text
lift_x / lift_y
```

夹住球后的抬起位置。抓住球但抬起过程容易掉球时，再调整这两个值。

```text
shoulder_pan_delta
```

抓取前肩部左右摆动量。当前值来自 Python 版本的 `CATCH_ACTION`。如果夹爪整体偏左或偏右，可以小幅调整它。

```text
gripper_open_delta / gripper_close_delta
```

夹爪打开和闭合动作量。当前夹爪测试中 `arm_gripper 60` 是打开，`arm_gripper 0` 是完全关闭。如果夹爪闭合不够，可以让 `gripper_close_delta` 更小；如果夹得太死或容易撞球，可以减小闭合幅度。

```text
wrist_pick_pitch / wrist_lift_pitch
```

抓取时和抬起后的腕部姿态。夹爪角度不贴合球时，优先调整 `wrist_pick_pitch`。

如果夹爪闭合时不是尽量垂直向下，而是明显前倾或后仰，说明当前腕部角度不是最佳抓球角度。建议每次改 `5`：

```text
wrist_pick_pitch = 80
wrist_pick_pitch = 75
wrist_pick_pitch = 85
```

观察夹爪在闭合瞬间是否更接近从球两侧夹住，而不是从斜上方推球。

```text
ball_target_size
```

视觉伺服停车尺寸。日志中的 `size=155/156` 就是球在画面里的检测框尺寸。

- 值越大，车会离球更近再停。
- 值越小，车会离球更远就停。
- 当前默认 `155`，让车比 `168~170` 像素时稍远一点停车。

机械臂位置调通之前，建议先不要调整这个值。

```text
ball_size_tolerance
```

允许球尺寸偏离目标尺寸的范围。当前默认 `5`，表示球尺寸在目标值上下约 5 像素内才认为距离合适。

```text
ball_center_tolerance
```

进入抓取前允许的球中心误差，单位是像素。之前 `off=27/28` 也会进入抓取，容易造成夹爪横向偏离。当前默认 `12`，也就是球中心必须更接近画面目标中心才会抓取。

```text
stable_frames
```

进入抓取前需要稳定满足条件的帧数。默认 `10`，约等于 1 秒左右，具体取决于实际推理帧率。

### 推荐调试流程

推荐先不要跑完整闭环，而是单独测试机械臂 IK 抓取：

```bash
cd /home/orangepi/robot/aka-rk3588
./build/tennis test-new-arm /dev/ttyACM0 ik-pick
```

调试时先让球和车保持在完整闭环停车后的相对位置，然后执行 `ik-pick`。这样可以只观察机械臂，不受视觉和底盘运动影响。

如果夹爪靠前，也就是伸过球、在球前方闭合，优先减小：

```text
grab_x
pre_grab_x
```

小幅试错时建议每次改 `0.005` 米，例如：

```text
grab_x = 0.1350
pre_grab_x = 0.1350
```

如果现场已经能明显估计偏差，例如夹爪整体靠前约 `5cm`，可以直接把 `grab_x` 和 `pre_grab_x` 同步减小 `0.05`：

```text
grab_x = 0.0900
pre_grab_x = 0.0900
```

如果夹爪够不到球，增大 `grab_x`。如果夹爪太高或太低，调整 `grab_y`，每次改 `0.005` 米。

每次调完后重新执行：

```bash
./build/tennis test-new-arm /dev/ttyACM0 ik-pick
```

如果抓住了球，日志中最后的夹爪反馈应该明显大于空夹时的值，并且抓取后视觉里不应再看到球。例如失败时可能看到：

```text
PICK_BALL done gripper=10.0 gripper_hold=no ball_visible=yes area=0.075 off=-8 size=152 label=BALL_READY holding=no
```

抓住球后通常会看到：

```text
PICK_BALL done gripper=45.4 gripper_hold=yes ball_visible=no area=0.000 off=0 size=0 label=IDLE holding=yes
```

完整闭环里如果第一次没夹住，会看到类似日志：

```text
grab failed -> PICK_BALL immediate retry next attempt=2/10 reason=ball still visible
-> PICK_BALL attempt=2/10
PICK_BALL start IK catch sequence attempt=2/10 offset=(-0.0050, 0.0000) grab=(0.1350, -0.0500)
```

如果某次偏移成功，会看到：

```text
saved successful pick config grab=(...)
-> FIND_BUCKET
```

如果肉眼看不出夹取位置变化，以日志中的 `offset=(...)` 和 `grab=(...)` 为准。早期偏移只有 0.005 米，肉眼可能不明显；现在重试范围扩大到了 0.020 米。

### 抓取后肩部回正

Python 原始抓取序列里，在夹爪闭合后有一步：

```text
shoulder_pan +12
```

它的作用是把抓取前的肩部水平转动量转回去。实际观察时，这会表现为“夹取动作结束、收机械臂时向右转一下”。

当前 C++ 已经把这个动作改成配置项，默认开启以贴近 Desktop-Wanderer 原始动作：

```text
return_shoulder_pan = 1
```

- `0`：不执行肩部回正，便于观察是否真的夹住球。
- `1`：执行肩部回正，行为更接近 Python 原始动作。

如果只是调抓球，可以临时改成 `0`。调完建议恢复为 `1`，否则夹取后的回收轨迹会比 Linux Python 版本更别扭。

底盘停车距离由检测框尺寸控制。日志中 `size=155/156` 表示当前停止时球的像素尺寸。如果整体停车距离需要调整，再改：

```text
ball_target_size
```

值越大，车会停得越近；值越小，车会停得越远。当前配置为 `155` 像素。

### 安全停止

如果机械臂动作方向明显不对、碰到结构件，或需要手动摆动机械臂，先关闭扭矩：

```bash
./build/tennis test-feetech /dev/ttyACM0 torque-off
```

关闭扭矩后可以手动移动机械臂。再次执行 `pos`、`ik-pick` 或完整闭环时，程序会重新配置并开启机械臂扭矩。
