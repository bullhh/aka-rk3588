# LeKiwi Linux/StarryOS 开发调试手册

本文只说明开发板启动、源码同步、编译部署、底层测试、日志定位和恢复手段。机器人
功能、状态机、参数含义和现场调参见
[`lekiwi_user_manual.md`](lekiwi_user_manual.md)。


## 1. 安全规则

1. 首次测试底盘时架起三个轮子。
2. 首次测试机械臂参数时取出球，检查线束和周围障碍物。
3. `place-cycle`、`ik-put` 和桶演示会打开夹爪。
4. 手动移动机械臂前必须关闭扭矩。
5. 同一时间只保持一个 Linux 或 StarryOS 板卡租约。
6. 准备使用 `Ctrl-C`；串口终端退出使用 `Ctrl-A x`。

紧急恢复：

```bash
./build/tennis test-base auto stop
./build/tennis test-feetech auto torque-off
```

## 2. 启动开发板 Linux

在本地 `tgoskits` 仓库执行：

```bash
cd /home/szy/work/robot/tripod/tgoskits
cargo board connect \
  --board-type OrangePi-5-Plus-robot \
  --server 10.3.10.60 \
  --port 2999
```

该命令申请板卡、启动 Linux 并进入串口。登录后查看当前 IP：

```bash
hostname -I
ip -4 addr
```

另开终端连接：

```bash
ssh orangepi@<board-ip>
```

如果第一次启动出现 `Card did not respond to voltage select` 或 `fs_devread read error`，
在 U-Boot 提示符执行一次：

```text
reset
```

若连续重试仍失败，检查存储卡、电源和板卡状态，不要归因于用户态程序。

## 3. 同步源码

推荐使用仓库脚本并显式指定本次 IP：

```bash
cd /home/szy/work/robot/tripod/aka-rk3588
REMOTE=orangepi@<board-ip> \
REMOTE_PROJECT=/home/orangepi/robot/aka-rk3588 \
./scripts/sync_to_orangepi.sh
```

脚本排除 `.git`、`build/` 和运行生成的图片。脚本当前存在历史默认 IP，因此调试时
应始终显式传入 `REMOTE`。

手工同步可使用：

```bash
rsync -az --delete \
  --exclude .git \
  --exclude build \
  /home/szy/work/robot/tripod/aka-rk3588/ \
  orangepi@<board-ip>:/home/orangepi/robot/aka-rk3588/
```

注意：`--delete` 会删除远端不在本地源码中的文件。同步前先把开发板上自动学习或
手工调出的稳定 `config/lekiwi_pick_config.txt` 对比并同步回本地：

```bash
diff -u \
  config/lekiwi_pick_config.txt \
  <(ssh orangepi@<board-ip> \
    'cat /home/orangepi/robot/aka-rk3588/config/lekiwi_pick_config.txt')
```

## 4. 在 Linux 编译

推荐在开发板 Linux 原生编译，以避免 GLIBC 和动态库版本不匹配：

```bash
ssh orangepi@<board-ip>
cd /home/orangepi/robot/aka-rk3588
PKG_CONFIG_PATH=/home/orangepi/miniforge3/envs/rknn/lib/pkgconfig \
LD_LIBRARY_PATH=/home/orangepi/miniforge3/envs/rknn/lib:${LD_LIBRARY_PATH:-} \
./build_rk3588.sh -b Release -l INFO
```

增量编译：

```bash
cd /home/orangepi/robot/aka-rk3588
cmake --build build --parallel "$(nproc)"
```

成功现象：

```text
[100%] Built target tennis
```

检查产物和依赖：

```bash
file build/tennis
ls -lh build/tennis models/tennis.rknn
ldd build/tennis
```

`build/tennis` 应为 AArch64 ELF。若出现 `GLIBC_x.xx not found`，不要替换系统库，
回到开发板 Linux 重新原生编译。

本地交叉编译仅在工具链和目标 rootfs 动态库完全匹配时使用：

```bash
cd /home/szy/work/robot/tripod/aka-rk3588
./build_rk3588.sh -b Release -l INFO
scp build/tennis \
  orangepi@<board-ip>:/home/orangepi/robot/aka-rk3588/build/tennis
```

## 5. 让 StarryOS 使用最新文件

Linux 和 StarryOS 共用物理 rootfs。必须先在 Linux 写入并同步：

```bash
cd /home/orangepi/robot/aka-rk3588
test -x build/tennis
test -f models/tennis.rknn
test -f config/lekiwi_calibration.json
test -f config/lekiwi_pick_config.txt
sync
```

退出 Linux 串口：

```text
Ctrl-A x
```

然后在本地启动 StarryOS：

```bash
cd /home/szy/work/robot/tripod/tgoskits
cargo xtask starry board \
  -c os/StarryOS/configs/board/orangepi-5-plus.toml \
  --board-config apps/starry/orangepi-5-plus-uvc/board-orangepi-5-plus.toml \
  --board-type OrangePi-5-Plus-robot \
  --server 10.3.10.60 \
  --port 2999
```

进入 `root@starry` 后：

```sh
cd /home/orangepi/robot/aka-rk3588
ls -l build/tennis models/tennis.rknn \
  config/lekiwi_calibration.json config/lekiwi_pick_config.txt
```

StarryOS 不编译 C++ 程序，直接运行 Linux 已经构建的二进制。如果看不到新文件，回
Linux 检查写入目录、时间戳以及是否执行 `sync`。

## 6. 最小验证顺序

每次更改控制代码或配置后按顺序执行。前一步失败时不要继续完整动作。

推荐把初始化分成四层，不要直接用完整闭环同时验证所有硬件：

| 层级 | 命令 | 是否动作 | 通过标准 |
| --- | --- | --- | --- |
| 配置 | `config-check` | 否 | 参数和规划轨迹在安全范围内 |
| 通信 | `scan`、`read` | 否 | 连续找到并读取ID1～ID9 |
| 校准 | `calib-check` | 否 | 校准文件存在、格式和关节完整 |
| 执行 | `task home`、`task carry` | 是 | 从当前位置限速动作且`done=1 failed=0` |

完整初始化检查命令：

```bash
cd /home/orangepi/robot/aka-rk3588
./build/tennis test-new-arm auto config-check
./build/tennis test-feetech auto scan
./build/tennis test-feetech auto read
./build/tennis test-new-arm auto calib-check
```

第一次上电若只有第一条舵机命令出现一次`rx timeout waiting for header/params`，立即重试
同一条命令。第二次完整成功通常是USB CDC刚建立后的瞬态；连续两次以上失败才检查供电、
USB线、总线占用和CDC实现。

### 6.1 配置静态校验

```bash
cd /home/orangepi/robot/aka-rk3588
./build/tennis test-new-arm auto config-check
```

该命令不打开总线、不让机械臂动作。它检查抓球/放球关节范围和轨迹走廊。

### 6.2 Feetech总线

```bash
./build/tennis test-feetech auto scan
./build/tennis test-feetech auto read
```

正常结果：

```text
found 9 motor(s): 1 2 3 4 5 6 7 8 9
```

Linux 常见日志：

```text
libusb_open ... LIBUSB_ERROR_ACCESS
falling back to TTY /dev/ttyACM0
opened TTY /dev/ttyACM0 baud=1000000
```

这是正常回退。StarryOS 应选择 libusb CDC。`scan` 后必须能在下一进程继续 `read`；
若出现 `BUSY`，检查是否仍有旧进程占用接口。

偶发一次 `rx timeout waiting for header/params` 时，先单独重试命令。若重试成功且
`scan/read` 完整，不要把它误判为轨迹错误；若连续复现，检查供电、USB线和舵机总线。

### 6.3 校准

```bash
./build/tennis test-new-arm auto calib-check
```

只有更换舵机、机械结构改变或确认校准失效时执行：

```bash
./build/tennis test-new-arm auto calibrate
```

校准会关闭 ID1～ID6 扭矩并要求手动移动到安全端点，结果写入
`config/lekiwi_calibration.json`。不要把动作姿态写进校准文件。

首次校准的正确操作：

1. 托住机械臂并清空周围空间，执行`calibrate`后ID1～ID6会关闭扭矩。
2. 在程序持续显示位置时，分别缓慢转动所有非连续关节到两个安全机械端点；不要撞击
   限位，也不要依靠舵机通电硬顶端点。
3. ID5是全转关节，程序使用完整`0～4095`范围，不需要寻找机械端点。
4. 所有关节范围都记录后按回车，程序以端点中点计算零位并写入JSON。
5. 校准结束时扭矩仍关闭；先执行`calib-check`，再空载执行`task home`。

以下情况才重新校准：更换舵机、拆装花键导致零位变化、机械端点变化，或者
`calib-check`明确失败。单纯夹球位置不准时不要校准，应调整
`config/lekiwi_pick_config.txt`。

### 6.4 不开夹爪的机械臂测试

```bash
./build/tennis test-new-arm auto task home
./build/tennis test-new-arm auto task carry
./build/tennis test-new-arm auto task place-approach
./build/tennis test-new-arm auto task place-release
```

`place-release` 会先到固定接近姿态，再到配置的最终放球姿态，但不会打开夹爪。命令
从舵机实际位置起步；目标已经到位时几乎不动作是正常现象。

初始化时正常日志应包含：

```text
[FeetechArm] torque enabled at current positions; no startup jump
```

它表示程序先保持上电时的实际角度，再限速执行目标，不会先瞬间跳到预设姿态。

### 6.5 会开夹爪的机械臂测试

```bash
./build/tennis test-new-arm auto ik-pick
./build/tennis test-new-arm auto task place-cycle
./build/tennis test-new-arm auto ik-put
```

执行前取出球或准备接球。成功结果为：

```text
done=1 failed=0
```

### 6.6 三轮底盘

架起机器人后执行：

```bash
./build/tennis test-base auto forward 0
./build/tennis test-base auto backward 0
./build/tennis test-base auto left 0
./build/tennis test-base auto right 0
./build/tennis test-base auto rotate-left 0
./build/tennis test-base auto rotate-right 0
./build/tennis test-base auto stop
```

### 6.7 摄像头和NPU

```bash
./run_vision_once.sh
```

Linux 会先构建，StarryOS 直接使用已有二进制。StarryOS 脚本默认设置
`RKNN_CORE_MASK=0`。真实闭环暂不建议强制三核 NPU，因为历史上出现过异常框。

以下 UVC 描述符日志通常不影响采集：

```text
unsupported descriptor subtype VS_STILL_IMAGE_FRAME
unsupported descriptor subtype VS_COLORFORMAT
attempt to claim already-claimed interface 1
```

是否正常应以随后出现 `UvcCapture streaming 640x480 @ 30 fps` 为准。

#### Linux 报 `uvc_open failed: Access denied`

典型日志：

```text
[UvcCapture] uvc_open failed: Access denied
[ERROR] Failed to open UVC device 0
```

这不是编译失败，也不是摄像头损坏。当前程序通过 libuvc 直接访问
`/dev/bus/usb/<bus>/<device>`，需要对 usbfs 设备节点具有读写权限；即使
`/dev/video0` 属于 `video` 组也不能代替该权限。

旧机器人系统在 `/etc/udev/rules.d/99-dw-uvc-camera.rules` 中保存过摄像头专用
规则。更换或重做 rootfs 后，该系统配置不会随 aka 源码自动恢复，因此可能重新出现
权限错误。这是 Linux rootfs 配置，不需要修改用户态程序。

在开发板原生 Linux 中恢复持久规则：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0ac8", ATTR{idProduct}=="0346", MODE="0660", GROUP="plugdev"' \
  | sudo tee /etc/udev/rules.d/99-dw-uvc-camera.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --action=add
sudo udevadm settle
```

拔插一次摄像头最可靠。重新执行 `lsusb` 获取可能变化的 bus/device 编号：

```bash
id
lsusb | grep 0ac8:0346
ls -l /dev/bus/usb/<bus>/<device>
```

通过标准：`orangepi` 属于 `plugdev`，对应节点权限类似：

```text
crw-rw---- root plugdev ... /dev/bus/usb/<bus>/<device>
```

然后重新运行 `./run_lekiwi_full.sh`，应出现 UVC streaming 日志。不要把
`sudo ./run_lekiwi_full.sh` 当作长期方案。

Linux 下 Feetech 的 `LIBUSB_ERROR_ACCESS` 后若继续打印：

```text
falling back to TTY /dev/ttyACM0
opened TTY /dev/ttyACM0 baud=1000000
```

表示已正常使用 TTY 后端，与摄像头的致命 `uvc_open` 失败不是同一个问题。

### 6.8 状态机测试

```bash
cd /home/orangepi/robot/aka-rk3588
./run_lekiwi_test.sh          # 架空追球，确认后退出
./run_bucket_place_demo.sh    # 找桶、靠近、放球一次后退出
./run_lekiwi_full.sh          # 完整持续闭环
```

桶演示和完整闭环会驱动小车，必须现场观察，不应在无法看到障碍物时通过 SSH 盲跑。

### 6.9 夹球准确度调整

夹球误差由“摄像头决定的小车停车位置”和“机械臂相对车体的落点”两部分组成。必须
分开调试，否则同一次修改可能掩盖另一处误差。

#### 第一步：固定机械臂参数，先调停车位置

架起轮子观察追球日志，随后落地做低速实测。重点记录进入`BALL_READY`前后的：

```text
size=<球框尺寸> off=<球心左右偏差> ready=<连续稳定结果>
```

| 现象 | 优先调整 | 修改方向 |
| --- | --- | --- |
| 小车停得太远、机械臂够不到 | `ball_stop_size_px` | 增大，每次5～10 |
| 小车停得太近、机械臂伸过球 | `ball_stop_size_px` | 减小，每次5～10 |
| 前进时经常越过合适位置再后退 | `ball_stop_tolerance_px` | 适当减小，每次1～2 |
| 停车后球明显偏左或偏右 | `ball_center_tolerance_px` | 适当减小，每次5 |
| 偶发一帧满足就开始抓球 | `ball_stable_frames` | 增加1帧 |

不要用增大`tolerance`来掩盖夹不到球；容差越大，停车点的前后离散越大。先让同一位置
连续停车3～5次基本一致，再调整机械臂。

#### 第二步：固定小车和球，只调机械臂落点

```bash
./build/tennis test-new-arm auto config-check
./build/tennis test-new-arm auto ik-pick
```

`ik-pick`会产生完整抓球动作，应先空载运行，再放置固定网球。调整顺序如下：

| 现象 | 参数 | 修改方向 |
| --- | --- | --- |
| 夹爪伸过球 | `grab_forward_offset_cm` | 减小0.5 |
| 夹爪够不到球 | `grab_forward_offset_cm` | 增大0.5 |
| 夹爪落在球左侧 | `grab_lateral_offset_cm` | 减小0.5 |
| 夹爪落在球右侧 | `grab_lateral_offset_cm` | 增大0.5 |
| 夹爪太低 | `grab_height_offset_cm` | 增大0.5 |
| 夹爪太高 | `grab_height_offset_cm` | 减小0.5 |
| 位置正确但夹爪角度倾斜 | `grab_pitch_offset_deg` | 每次正/负5度试一个方向 |

上述偏移仍无法得到自然姿态时，再小角度修改`grab_id1_deg～grab_id5_deg`。一次只改一个
关节，并在每次修改后先执行`config-check`。ID6夹爪力度由
`gripper_open_delta_deg/gripper_close_delta_deg`控制，位置调试时保持原值。

#### 第三步：回到完整闭环验证

1. 恢复小车正常落地，以同一球位连续测试至少5次。
2. 区分“没有到球的位置”和“到位但没有夹紧”：前者调停车/位置，后者检查ID6接触日志。
3. 抓球失败后程序会尝试小范围前后和高度偏移；成功偏移可能自动写回配置。
4. 测试结束后对比开发板与仓库配置，只保留多次稳定成功的值。

推荐记录表：

```text
系统/提交 | size/off | 四项grab偏移 | 是否接触 | 是否CLEAR成功 | 是否进入CARRY
```

## 7. 日志定位

### 7.1 `communication failure` 不一定是通信错误

上层状态机的历史文案可能打印：

```text
PUT_BALL communication failure, stopping: <具体原因>
```

应以冒号后的具体原因判断：

- `unsafe place trajectory`：配置或轨迹安全校验失败。
- `rx timeout`：Feetech通信超时。
- `LIBUSB_ERROR_BUSY`：USB接口占用或未释放。
- `motor id=N returned error status`：对应舵机状态错误。

### 7.2 放球配置校验

当前固定接近姿态与 `place_id1～place_id5` 最终姿态独立。若提示最终姿态没有比接近
姿态低至少0.5cm，说明 ID2/ID3 组合并未产生下降；ID3更负不一定代表末端更低。

修改后先运行：

```bash
./build/tennis test-new-arm auto config-check
```

### 7.3 修改配置但动作没变

```bash
pwd
stat config/lekiwi_pick_config.txt build/tennis
pgrep -af tennis
```

确认修改的是当前目录、旧进程已经退出、Linux/Starry 使用同一个 rootfs。程序可能在
抓球成功后自动写回偏移，因此调试结束要对比开发板配置和仓库配置。

### 7.4 动作完成但肉眼看不到

S曲线命令是绝对目标，不会先回 HOME。重复执行同一姿态时可能只调整少量关节。需要
演示明显动作时先执行 `task home` 或另一个安全姿态，再执行目标命令。

### 7.5 机械臂动作期间摄像头暂停

完整闭环进入抓球或放球时，以下日志是当前正常设计：

```text
[UvcCapture] paused before PICK_BALL in ... ms
[UvcCapture] resumed after PICK_BALL in ... ms
```

机械臂动作是阻塞序列，期间不使用视觉帧；暂停UVC可以避免摄像头与Feetech CDC在共享
USB路径上竞争。StarryOS暂停/恢复各等待数百毫秒是允许的，恢复使用原设备句柄，不会
重新枚举或重复20帧预热。判断是否异常应看恢复后能否在1秒内取得新帧并继续状态机。

仅用于对比诊断时可关闭该功能：

```bash
LEKIWI_PAUSE_UVC_DURING_ARM=0 ./run_lekiwi_full.sh
```

不要把它作为StarryOS日常运行方式；实测不停流时机械臂控制周期最坏超过200ms，而停流
后保持在约50ms。

## 8. 恢复和采集信息

停止底盘和关闭扭矩：

```bash
./build/tennis test-base auto stop
./build/tennis test-feetech auto torque-off
```

确认占用：

```bash
pgrep -af tennis
ls -l /dev/ttyACM0
```

建议每次问题报告保存：

```text
系统：Linux / StarryOS
aka提交：git rev-parse --short HEAD
tgoskits提交：git -C ../tgoskits rev-parse --short HEAD
配置：config/lekiwi_pick_config.txt
命令：完整命令行
日志：从程序启动到失败的完整输出
现象：机械臂、轮子、球和桶的实际位置
供电：电压和是否发生重启
```

开发板稳定参数同步回仓库后再提交，避免下一次部署把实机调参覆盖。
