# HLJU RMIC —— 麦轮小车整车工程

黑龙江大学 2026 校内赛 RMIC 麦轮小车（云台 + 底盘 + 水弹波箱）的完整工程代码。

一架车的控制链路拆成五个模块：**PC 客户端控制 → Jetson 上位机 → 底盘固件 / 火控固件 / 云台步进电机 / 视觉自瞄**。

```
笔记本 ctl ──UDP 40001──► Jetson gimbal ──40pin UART 115200──► STM32F401 fire ──► 水弹波箱
   (X11)                     │  │── CAN0──► STM32F407 chassis（麦轮 + IMU） 和 ZDT X42S 云台步进 ×2
                             └── 27RM 自瞄（USB 摄像头）：──UDP 40002（瞄准数据）──► ctl（辅助/显示）
                                                            └──UDP 40003（命令口：切颜色/模式）◄── ctl
                             └── MJPEG 图传 :8080（27RM 内置 / gimbal.mjpeg_server）
```

## 模块一览

| 目录 | 一句话职责 | 运行平台 | 语言 / 构建 | 详细文档 |
|---|---|---|---|---|
| `ctl/` | PC 控制客户端：方向键控云台、WASD 控底盘、显示遥测与视频 | x86 Linux（X11） | C++17 / CMake | [ctl/README.md](ctl/README.md) |
| `gimbal/` | Jetson 上位机：两轴云台控制、底盘链路（UART + CAN 双后端）、图传服务 | Jetson Nano | C++17 / CMake | [gimbal/README.md](gimbal/README.md) |
| `chassis/` | 底盘固件：麦轮正逆解 + 四轮速度环 + IMU 陀螺外环 | STM32F407VGTx | C / CMake | [chassis/README.md](chassis/README.md) |
| `fire/` | 火控固件：水弹波箱开火，三层安全保护 | STM32F401RCT6 | C / CMake | [fire/README.md](fire/README.md) |
| `autoaim/` | 视觉自瞄：灯条检测 → 装甲板 PnP → 单板锁定 / 反小陀螺跟踪 → 自瞄 UDP 广播（40002）+ 内置 MJPEG 图传（8080） | Jetson Nano | C++17 / CMake + OpenCV | [autoaim/README.md](autoaim/README.md) |

## 快速开始

### PC 客户端 `ctl/`

```bash
cd ~/gimbal_ctl && ./build/gimbal_ctl --host 10.42.0.1      # 无线
# 或 ./build/gimbal_ctl --host 192.168.2.2                   # 有线
```

依赖 X11 + libjpeg（`libx11-dev`、`libjpeg-dev`）。线协议直接复用 `../gimbal` 的 `control_link.hpp`，**只有一个源头不复制**——本仓库里两个目录正好是兄弟关系，默认的 `GIMBAL_ROOT=../gimbal` 直接可用。

### Jetson 上位机 `gimbal/`

####  Jetson 侧起 AP

```bash
# 注意：car-ap 是 autoconnect=no，需要手动拉起
sudo nmcli con up car-ap
# 验证（关键！见 §8.1 假活问题）
cat /proc/net/dev | grep wlan0; sleep 3; cat /proc/net/dev | grep wlan0
```

####  Jetson Can 挂载

```bash
sudo slcand -o -s8 -t hw /dev/ttyACM1 can0 && sudo ip link set can0 up   # 编号以 ls /dev/ttyACM* 为准
candump -td can0      # 应看到 0x110~0x114 每 10ms 一组、组内连续
cansend can0 021#01   # EN 开；随后 020 速度才生效
cansend can0 000#     # 广播急停（DLC=0 也认）
```

####  Jetson 运行

```bash
cd gimbal
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/gimbal --device can0   # 全 CAN：两轴 ZDT 扩展帧 + 底盘标准帧同一条 can0
```

### 底盘固件 `chassis/`

```bash
cd chassis
cmake --preset Debug
cmake --build build/Debug --target INTRA_SCHOOL_COMPETITION_27RM
pyocd flash -t stm32f407vgtx build/Debug/INTRA_SCHOOL_COMPETITION_27RM.elf --connect under-reset
```

### 火控固件 `fire/`

```bash
cd fire
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --target biu
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
        -c "adapter speed 1000" \
        -c "program build/Debug/biu.elf verify reset exit"
```

### 视觉自瞄 `autoaim/`

```bash
cd autoaim
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DRM27_USE_PKGCONFIG=ON   # Jetson 上绕开 OpenCV+CUDA
cmake --build build -j4
./build/pose_test   --selftest    # 合成自检：4 角点 → 距离/yaw/pitch
./build/detect_test --selftest    # 合成自检：灯条检测链路
./build/27RM --track-test         # 反小陀螺合成自检（不用相机/标定）

# 实机（三件套一键起：gimbal + 27RM；--27rm-daemon 带 2 秒自动拉起）
tools/start_all.sh --27rm-daemon
./build/27RM --no-gui              # 无显示器时；GUI 下 u=切换单板/反小陀螺
```

USB 摄像头 800×600 MJPG → 灯条检测 → 装甲板 PnP → 单板锁定 / 反小陀螺跟踪；
输出相机系距离 / yaw / pitch / 板面偏角，并通过 **UDP 40002** 广播给 `ctl`（辅助显示/瞄准），
`ctl` 用 **UDP 40003** 命令口切颜色/模式；内置 **MJPEG 8080** 图传。

## 硬件一览

| 部件 | 型号 / 说明 |
|---|---|
| 上位机 | Jetson Nano（L4T R32.6.1 / Ubuntu 20.04） |
| 底盘主控 | STM32F407VGTx（168 MHz） |
| 火控主控 | STM32F401RCT6（84 MHz） |
| 云台电机 | ZDT X42S 闭环步进 ×2（Emm5.0，yaw=2 / pitch=1，2:1 减速） |
| 底盘驱动 | TB6612 ×2（左右板） |
| 底盘电机 | 直流减速电机 ×4（麦轮） |
| IMU | LSM6DSV16X（ATK-MS6DSV 模块，I2C1 400 kHz） |
| 总线 | USART 115200 8N1 / CAN 1 Mbps（CANable 2.0 → `can0`） |
| 图传 | V4L2 MJPG → HTTP multipart（`mjpeg_server` :8080） |

## 各模块进度速览

- **ctl**：方向键云台 + WASD 底盘 + 遥测/视频显示，可用。
- **gimbal**：云台 ZDT 协议、底盘 UART/CAN 双后端、UDP v2 线协议、9 段自检，可用；云台双轴联调进行中。
- **chassis**：速度环 PID 调完、IMU 陀螺外环接入、USART2 + CAN1 双链路定型（v0.6）。
- **fire**：开火链路 USART2 + PA12 低电平驱动，看门狗/超时保护验证通过。
- **autoaim**：800×600 相机标定完成（RMS 0.409px）；单板锁定 + 反小陀螺跟踪、自瞄 UDP 40002 广播、
  命令口 40003、内置 MJPEG 图传、`start_all.sh`/`run_27rm.sh` 守护脚本；与 ctl 联调完成（辅助瞄准可用）。

各模块的详细设计、踩坑记录与联调数据都在对应 README 及其 `docs/` 里，这里不重复。

## 关于本仓库

2026-11 由五个独立仓库合并而成（`HLJU_RMIC_gimbal` / `HLJU_RMIC_chassis` / `HLJU_RMIC_ctl` / `HLJU_RMIC_fire` / `HLJU_RMIC_autoaim`），
合并时**每个模块的完整提交历史都保留了**，按子目录就能单独查：

```bash
git log --oneline -- gimbal/     # 云台上位机的全部历史
git log --oneline -- chassis/    # 底盘固件的全部历史
```

日常开发在对应子目录里改、`git add <子目录>/`、一起提交即可；五个模块也可以各自独立构建、互不干扰。
