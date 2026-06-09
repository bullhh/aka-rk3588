# LeKiwi C++ 校准方案记录

## 背景

当前 C++ 实现已经可以读取并使用已有的 LeKiwi/LeRobot 校准文件：

```text
config/lekiwi_calibration.json
```

如果这个文件不存在或内容无效，C++ 的机械臂命令和完整 LeKiwi 闭环会拒绝运行。这是有意设计的安全策略：没有校准时，机械臂虽然可以“动”，但程序无法可靠地把 `ready`、`grab`、`release` 这类语义动作映射到安全、正确的真实关节位置。

当前 C++ 已经实现了“使用校准结果”和“生成校准结果”。校准命令采用两端自动求中点方案：操作者只需要在一次校准过程中把所有机械臂关节都转到安全两端，程序同时记录 1..6 号电机范围，并自动计算中点作为零位，不再要求手动凭感觉摆到正中。

## 为什么必须校准

STS3215/Feetech 舵机暴露的是原始位置值，大致范围是：

```text
0..4095
```

单独一个 raw 值并不能说明真实机械臂姿态。它还取决于：

- 舵机安装方向
- 舵盘安装角度
- 连杆结构
- 关节零位
- 机械结构允许的安全活动范围

校准文件记录：

```text
id
drive_mode
homing_offset
range_min
range_max
```

这些信息可以让 C++ 做到：

- 把关节命令映射到舵机 raw 位置
- 对输出位置做安全限幅
- 避免在零点未知时运行机械臂
- 重启后保持一致行为

## 当前安全行为

当前行为如下：

```text
test-feetech scan/read      不需要校准，允许运行
test-base                   不依赖机械臂校准，允许运行
test-new-arm                缺校准时拒绝运行
run_lekiwi_loop.sh          缺校准时拒绝运行
```

这样可以避免系统不知道机械臂零点和限位时误动作。

## C++ 校准命令

建议增加一个显式命令：

```bash
./build/tennis test-new-arm /dev/ttyACM0 calibrate
```

该命令是交互式命令，只能在人现场看护机器人时执行。

## 建议校准流程

### 1. 总线检查

打开 Feetech 总线：

```text
/dev/ttyACM0
baudrate 1000000
```

扫描期望电机 ID：

```text
1 2 3 4 5 6 7 8 9
```

如果机械臂电机 `1..6` 有任意一个缺失，立即失败。

底盘电机 `7..9` 也应该打印出来，但机械臂校准主要依赖 `1..6`。

### 2. 关闭机械臂扭矩

关闭机械臂电机扭矩：

```text
1..6
```

底盘保持停止。

提示用户一次性操作所有机械臂关节：

```text
机械臂扭矩已关闭。
请手动缓慢转动所有机械臂关节到安全两端，全部完成后按 ENTER。
```

这一步需要操作者手动扶住或移动机械臂。

### 3. 记录两端并自动计算中点

校准开始后，程序会循环读取 1..6 号机械臂电机的 `Present_Position`，同时记录每个电机观察到的最小值和最大值：

```text
range_min = 观察到的最小值
range_max = 观察到的最大值
```

然后自动计算中心和零位偏移：

```text
center = (range_min + range_max) / 2
homing_offset = center - 2047
```

对于连续旋转关节或轮子，范围可以使用：

```text
range_min = 0
range_max = 4095
```

### 4. 一次性记录所有关节的安全范围

操作者在同一个校准窗口内，把所有有限角度关节都在安全范围内完整活动一遍。

示例提示：

```text
请把所有机械臂关节都转动到安全两端。
全部完成后按 ENTER。
```

在等待期间，C++ 循环读取 `Present_Position`，实时显示：

```text
raw / min / max
```

同时记录的关节：

```text
arm_shoulder_pan
arm_shoulder_lift
arm_elbow_flex
arm_wrist_flex
arm_wrist_roll
arm_gripper
```

夹爪也需要在同一过程中完整打开和闭合一遍，记录安全范围。

### 5. 保存校准文件

写出文件：

```text
config/lekiwi_calibration.json
```

格式示例：

```json
{
  "arm_shoulder_pan": {
    "id": 1,
    "drive_mode": 0,
    "homing_offset": 1803,
    "range_min": 803,
    "range_max": 3433
  }
}
```

文件中应包含 9 个电机：

```text
arm_shoulder_pan   1
arm_shoulder_lift  2
arm_elbow_flex     3
arm_wrist_flex     4
arm_wrist_roll     5
arm_gripper        6
base_left_wheel    7
base_back_wheel    8
base_right_wheel   9
```

底盘轮子可以使用：

```text
homing_offset = 0
range_min = 0
range_max = 4095
```

### 6. 校验保存结果

写出文件后：

1. 用 `LekiwiCalibration` 重新加载。
2. 检查必需关节是否存在。
3. 检查 ID 和范围是否合法。
4. 打印摘要。

期望输出示例：

```text
校准文件已保存: config/lekiwi_calibration.json
arm_shoulder_pan: id=1 offset=... range=[..., ...]
...
```

### 7. 校准结束后的安全状态

校准结束后：

- 底盘保持停止。
- 机械臂默认保持扭矩关闭，除非操作者明确选择开启。
- 不要自动执行 `pos`、`grab` 或任何机械臂动作。

## 安全要求

校准不能在正常启动时自动执行。

它必须是显式命令，因为校准过程需要：

- 人在现场看护
- 手动移动机械臂
- 知道机械结构的安全限位
- 在机械臂卡住或碰撞时立即停止

如果正常运行时找不到 `config/lekiwi_calibration.json`，程序应该继续拒绝机械臂和完整闭环，并打印清晰提示：

```text
missing/invalid calibration: config/lekiwi_calibration.json
请先运行 test-new-arm /dev/ttyACM0 calibrate
```

## 实现位置

建议修改或新增的文件：

```text
robot/lekiwi_calibration.hpp
robot/lekiwi_calibration.cpp
robot/feetech_arm.cpp
test_cmds.cpp
test_cmds.hpp
README.md
```

已新增或使用的方法：

```cpp
bool LekiwiCalibration::save(const std::string& path) const;
void LekiwiCalibration::set(const std::string& name, const JointCalibration& cal);
```

已新增校准辅助函数：

```cpp
static int cmd_calibrate_lekiwi_arm(const char* uart_dev);
```

当前轻量 JSON 解析器只负责读取已知校准格式。保存校准文件时，可以直接用 `std::ofstream` 生成 JSON，不需要引入外部 JSON 依赖。

## 当前状态

已经实现：

- C++ 读取 `config/lekiwi_calibration.json`。
- 缺少校准时，C++ 拒绝机械臂和完整闭环。
- `test-new-arm /dev/ttyACM0 calib-check` 可以检查校准文件是否可用。
- `test-new-arm /dev/ttyACM0 calibrate` 可以交互生成校准文件。
- 校准时一次性记录 1..6 号电机安全两端，并自动计算中点。
- C++ 自动写出 `config/lekiwi_calibration.json`。
