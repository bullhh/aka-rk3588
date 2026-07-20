# LeKiwi Linux/StarryOS 开发调试手册

本文只说明开发板启动、源码同步、编译部署、底层测试、日志定位和恢复手段。机器人
功能、状态机、参数含义和现场调参见
[`lekiwi_user_manual.md`](lekiwi_user_manual.md)。

适用环境：

```text
本地源码：/home/szy/work/robot/tripod/aka-rk3588
tgoskits：/home/szy/work/robot/tripod/tgoskits
共享rootfs：/home/orangepi/robot/aka-rk3588
板卡服务：10.3.10.60:2999
板卡类型：OrangePi-5-Plus-robot
Linux账号：orangepi
Feetech参数：auto
```

开发板 IP 由 DHCP 分配，不能把文档或同步脚本中的示例地址当成固定地址。

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

### 6.4 不开夹爪的机械臂测试

```bash
./build/tennis test-new-arm auto task home
./build/tennis test-new-arm auto task carry
./build/tennis test-new-arm auto task place-approach
./build/tennis test-new-arm auto task place-release
```

`place-release` 会先到固定接近姿态，再到配置的最终放球姿态，但不会打开夹爪。命令
从舵机实际位置起步；目标已经到位时几乎不动作是正常现象。

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

### 6.8 状态机测试

```bash
./run_lekiwi_test.sh          # 架空追球，确认后退出
./run_bucket_place_demo.sh    # 找桶、靠近、放球一次后退出
./run_lekiwi_full.sh          # 完整持续闭环
```

桶演示和完整闭环会驱动小车，必须现场观察，不应在无法看到障碍物时通过 SSH 盲跑。

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
