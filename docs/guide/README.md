# 三足捡球机器人 — 使用文档

`aka-rk3588` 是运行在 Orange Pi 5 Plus（RK3588）三轮机器人上的用户态控制程序。
它把 UVC 摄像头、RK3588 NPU、三轮全向底盘和六关节 Feetech 机械臂串联成一套完整
的捡球系统：机器人自主寻找网球、靠近并抓取，再寻找红桶并放入球。

这组文档既是系统展示，也是使用和二次开发的入口。建议按由浅入深的顺序阅读：

1. [功能与系统概览](overview.md) — 机器人能做什么、硬件和软件如何组成
2. [用户态程序使用、测试与调参](configuration-and-tuning.md) — 编译、Linux/StarryOS运行、单项测试和实机调参
3. [程序运行与控制原理](runtime-architecture.md) — 图像到动作的完整链路、状态机和控制机制
4. [性能与实测数据](performance.md) — 四种运行环境的同版本数据与瓶颈分析
5. [StarryOS 原生与 AxVisor 单客户机复现](reproduction-starry-native-single-guest.md) — 在另一台机器人上从主线 `dev` 复现构建、部署和运行

同一份用户态程序、模型和动作配置可用于 Linux、StarryOS，以及二者作为 AxVisor
客户机的环境。

更细的历史故障记录和研发过程保留在：

- [机械臂调试手册](../lekiwi_arm_debug_guide.md)
- [性能优化完整记录](../optimize/performance-optimization-summary.md)
