# HLJU_RMIC_fire —— 水弹波箱开火控制器

STM32F401RCT6 水弹波箱（发射机构）固件：通过 **USART2** 接收开火命令，
用 **PA12 低电平**驱动 MOS 管发射；带三层安全保护，断流/超时自动停火。

> ⚠️ **这条 UART 链路当前没有接进整车**：`gimbal` 里还没有 `fire_link`
> （没有 `fire_link.hpp`、没有 `/dev/ttyTHS1` 发帧代码），实测在跑的开火通道是
> **Jetson 40pin GPIO 直驱 MOS**。本固件是按 UART 方案写好、可独立烧录自测的
> **预留 / 备份方案**。两种方案的差别见下节。

本目录只包含 **STM32F401 固件**；Jetson 侧（`gimbal/`）与笔记本客户端（`ctl/`）见对应工程，
协议、接线、构建、台架自测、排障与实测记录全部收在**本文件**（原先的 `FIRE_LINK.md` 已并入）。

---

## 开火链路：两种方案

| | **① GPIO 直驱（★ 当前在用）** | **② UART 到本板（本文固件，预留）** |
|---|---|---|
| 执行者 | Jetson `fire_worker.py`（`Jetson.GPIO`） | 本板 STM32F401 |
| 信号引脚 | 40pin **BOARD 12**，**高电平导通** | **PA12**，**低电平导通** |
| 命令通道 | gimbal 管道写 `1` / `0` | USART2 115200 二进制帧（见「通信协议」） |
| 断流保护 | UDP Fire 位 **0.3 s** 窗口 + stdin EOF 自停火 | **100 ms** 看门狗 |
| 连续导通上限 | **无**（按住就一直打节拍） | **5 s** 超时锁存，松扳机解锁 |
| 输出口径 | 高 **80 ms** / 低 **120 ms** 节拍（`FIRE_GRIP_MS` / `FIRE_GAP_MS`） | 请求即持续导通 |

### 方案 ①：GPIO 直驱（当前实际链路）

```
笔记本 gimbal_ctl ──UDP 40001(Fire 位)──► Jetson gimbal ──管道──► fire_worker.py ──BOARD 12──► MOS ──► 波箱
```

- **客户端**：按住 `F` → 控制包带 `ctrl_flag::Fire`（0x80），100 Hz 随控制包发出；
  `空格` HOLD 时不发。
- **Jetson**：`FireControl`（`gimbal/src/device/fire_control.cpp`）fork/exec
  `python3 <--fire-script> <--fire-pin>`，只在状态变化时往管道写 `1\n` / `0\n`。
  最后一帧带 Fire 的包**只在 0.3 s 内有效**，超时自动停火；
  本地 stdin 的 `f 1|0` 也能开火，`x` 急停会清掉本地请求。
- **worker**（`gimbal/src/device/fire_worker.py`）：`BOARD` 模式、`initial=GPIO.LOW`
  上电即关火；`1` → 进入高 80 ms / 低 120 ms 节拍循环，`0` → 立即拉低，
  `q` 或 stdin EOF（父进程崩了）→ 停火 + `GPIO.cleanup()`。
- **收尾**：gimbal 析构写 `0\nq\n`，等不到进程退出 1 s 后 `SIGTERM`。
- ⚠️ 电平与固件方案相反，接 MOS 时**不要**照抄下方 §硬件 里 PA12 的接法。

### 方案 ②：UART 到本板（本文固件，预留）

```
笔记本 gimbal_ctl ──UDP──► Jetson gimbal(Fire 位) ──40pin UART(115200)──► 本板 STM32F401 ──► PA12 ──► MOS ──► 波箱
```

固件侧已完整实现（协议 / 保护 / 状态反馈全在 `Core/Src/main.c`），缺的是 Jetson 侧：
`fire_link`（收发 6 B 命令帧 / 7 B 反馈帧、100 Hz 发送、0.3 s 超时）尚未编写。
要切回这条链路需要：① 补 `gimbal/include/gimbal/link/fire_link.hpp` + `.cpp` 并接管扳机；
② 不再启动 GPIO worker（`--no-fire` 或去掉 `FireControl`）；③ 按 §硬件 的 40pin 接线。

---

## 功能特性（本板固件能力，对应方案 ②）

- **USART2（PA2/PA3）115200 8N1** 二进制帧协议，CRC-8/ATM 校验
- **PA12 低电平开火**，上电/复位默认高电平（关火）
- **50 Hz 状态反馈**：开火中 / 链路超时 / 超时保护
- **三层安全**：100 ms 看门狗、连续导通 5 s 超时保护、CRC 校验
- 事件日志（`[biu] FIRE ON/OFF`、`LINK OK/LOST`）从同一 USART2 输出

## 硬件

| 项 | 说明 |
|---|---|
| MCU | STM32F401RCT6（84 MHz，HSE 25 MHz） |
| 开火输出 | **PA12** → MOS 栅极（**低电平导通**，高电平关断） |
| 控制串口 | USART2：PA2 = TX，PA3 = RX |
| 调试 | DAP-LINK：SWD + USART2（可与 Jetson 共用同一串口，注意不要并线） |
| 状态灯 | PC13（链路正常 1 Hz 慢闪，断链 5 Hz 快闪） |

> ⚠️ `biu.ioc` / `Core/Src` 里还初始化了 **SPI1（PA5 = SCK、PA6 = MISO、PA7 = MOSI，42 Mbit/s）**
> 和 **USART1（PA9 = TX、PA10 = RX，115200）**，但 `main.c` 全程没用到
> （`hspi1` 无任何调用，`huart1` 只挂了中断没有回调）。
> 这两组引脚目前是「配了没用的死代码」，接线时别当空闲脚拿去用。

### 固件方案接线（40pin UART，★ 预留，当前未接入）

> 这组接线只在切到方案 ② 时才需要；方案 ① 只用 40pin 的 **12 脚 + GND**，不占 pin 8 / pin 10。

```
Jetson 40pin pin 8  (TXD)  →  F401 PA3 (USART2_RX)
Jetson 40pin pin 10 (RXD)  ←  F401 PA2 (USART2_TX)
Jetson 40pin pin 6  (GND)  ↔  GND
```

> ⚠️ 低电平开火：接线时 PA12 若碰到 GND 会误触发一次开火，接线前先拔波箱电池。

## 通信协议（F401 固件侧，仅方案 ② 使用）

> 方案 ①（GPIO）不走这里的任何一个字节，下表仅供固件自测 / 将来改回 UART 时对照。
> 与 Jetson 侧**逐字节一致**的参考实现尚未编写（`fire_link` 缺失）。

### 命令帧（Jetson → 本板，6 B，100 Hz）

| off | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| 字段 | `0xA5` | `0x5A` | `0x10` | seq | flags | crc8 |

- `flags` bit0 = FIRE 请求（1 = 开火，0 = 停火）
- `crc8` 覆盖 0~4 字节，CRC-8/ATM（poly `0x07`，init `0`）

### 反馈帧（本板 → Jetson，7 B，50 Hz）

| off | 0 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|---|
| 字段 | `0x5A` | `0xA5` | `0x81` | seq 回显 | state | crc8 | `0x0D` |

`state`：`0x01` = 开火中（PA12 低），`0x02` = 链路超时，`0x04` = 连续导通超时保护已触发

## 安全机制（F401 固件自身）

> 下表是方案 ② 的三层保护。方案 ① 的保护由 Jetson 侧实现，**少一层**：没有 5 s 连续导通上限，
> 只有 UDP Fire 位 0.3 s 窗口 + 父进程退出（stdin EOF）自停火 —— 见「开火链路」对比表。

| 层 | 条件 | 动作 |
|---|---|---|
| 链路看门狗 | 100 ms 收不到合法命令帧 | 强制 PA12 高（关火） |
| 导通保护 | 连续导通超过 5 s | 强制关断并锁存，**松开扳机才解锁** |
| 上电默认 | 复位 / 上电 | PA12 输出高（关火） |
| 帧校验 | 帧头/CRC 错误 | 丢弃，不刷新看门狗 |

## 构建与烧录

```bash
# 构建（需要 arm-none-eabi-gcc + Ninja + CMake）
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug --target biu

# 烧录（DAP-LINK，SWD）
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
        -c "adapter speed 1000" \
        -c "program build/Debug/biu.elf verify reset exit"
```

> 首次构建前若未生成 `build/Debug`，也可以用 `cmake --preset Debug && cmake --build --preset Debug`。

## 台架自测（不接整车）

**方案 ①（GPIO worker，当前用）** —— 在 Jetson 上直接跑 worker，手敲 `1` / `0` / `q` 看 MOS 是否按节拍通断：

```bash
# 需要 Jetson.GPIO；普通用户须在 gpio 组内（或 sudo），否则 GPIO.setup 报权限错
FIRE_VERBOSE=1 python3 gimbal/src/device/fire_worker.py 12
# 输入 1 开火、0 停火、q 退出；FIRE_VERBOSE=1 会打印每次脉冲时间戳（无示波器时看 80/120 ms 节拍）
```

**方案 ②（F401 固件）** —— 串口按 §通信协议 发 6 B 命令帧，看 7 B 反馈帧：

```bash
# 用 Python(pyserial) / 串口助手按上文「通信协议」的 6 B 命令帧自己发即可；
# 当年用的临时脚本 /tmp/fire_test.py 重启就没了，按协议随时可重建。
```

期望输出（固件自测实测值）：

```
idle:        seq=0   state=0x02        ← 没收到命令（TIMEOUT 位）
trigger ON : seq=16  state=0x01        ← 开火：PA12 低
trigger OFF: seq=9   state=0x00        ← 停火：PA12 高
```

## 排障

### 设备/串口被占用

```bash
fuser -v /dev/ttyTHS1        # 方案 ②：看谁占着开火串口
pkill -x gimbal              # 停掉 gimbal（别用 pkill -f build/gimbal，会误杀自己的 shell）
```

`gimbal` 运行时会独占 `/dev/ttyTHS1`，再跑测试脚本会打不开。

### 命令发不过去（反馈正常、开火没反应）

先确认 Jetson 真的在发、F401 真的收到。**最可靠的办法是直接读 STM32 内部状态**（SWD，方案 ②）：

```bash
# 读 g_fire 结构体（地址先跑 arm-none-eabi-nm build/Debug/biu.elf | grep g_fire 确认，以下为某次编译的实例）
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 1000" -c "init" -c "halt" \
  -c "mdw 0x2000002c 9" -c "reset run" -c "shutdown"
```

| 偏移 | 字段 |
|---|---|
| +8 | `last_rx_ms` |
| +12 | `rx_frames`（累计合法命令数） |
| +16 | `bad_frames` |
| +20 | `last_seq` |
| +21 | `fire_req` |
| +22 | `fire_on` |

- `rx_frames` 不涨 → 命令没到 F401（查 TX 线 / 引脚）
- `rx_frames` 涨但 `fire_req=0` → 收到的是停火帧（查客户端 / Jetson 侧）

**收线电平诊断**（把 PA3 配成输入再读 IDR）：

```bash
# 清 MODER bits 7:6 把 PA3 配成输入，然后读 IDR bit3
openocd ... -c "mww 0x40020000 0xA928A820" -c "mdw 0x40020010 1"
```

- 上拉/下拉读数都恒 0 → 引脚被外部拉低，TX 没接对
- 发着数据时读数不跳变 → 线上根本没有数据

**输出极性验证**（读 GPIOA ODR，bit12 = PA12 实际输出）：

```bash
openocd ... -c "init" -c "halt" -c "mdw 0x40020014 1" -c "reset run" -c "shutdown"
```

- 关火时应为 `0x...1000`（bit12=1）
- 开火时应为 bit12=0

### USB-TTL 枚举失败（`error -71`）

`dmesg` 里出现 `device not accepting address ... error -71` 基本都是线/口/模块硬件问题：
换线、换口、别过 hub；或者干脆改用 40pin UART 方案。

### gimbal 起不来（打不开 can0）

开火链路本身不依赖 CAN，但 `gimbal` 启动要先打开云台总线：

```bash
sudo slcand -o -s8 -t hw /dev/ttyACM0 can0    # -s8 = 1 Mbps；档位按实际配置改
sudo ip link set can0 up
ip -br link | grep can                        # 应看到 can0 UP
```

### 反馈帧率不对 / 偶发丢帧

- 反馈固定 **50 Hz**（20 ms 一帧），命令 **100 Hz**（10 ms 一帧）；
- 事件日志是**阻塞发送**（几毫秒）且只在状态变化时触发，不影响正常节奏。

## 实测记录（固件自测，方案 ②）

| 项 | 结果 |
|---|---|
| 反馈帧率 | 50 Hz（0.6 s 内 41 帧） |
| 命令回显 | 正常（`seq` 非 0） |
| 开火 | `state=0x01`，PA12 低电平（`ODR` bit12=0） |
| 停火 | `state=0x00`，PA12 高电平（`ODR` bit12=1） |
| 按住 6.5 s | 出现 `state=0x04` OVERTIME，PA12 强制关断 |
| 松开后重按 | 能重新开火（锁存解除） |
| 停发命令 0.3 s | `state=0x02` TIMEOUT，PA12 回高 |
| 上电默认 | PA12 高（关火），`state` 含 TIMEOUT 位 |
| UART1（PA9/PA10） | **不通**（PA10 恒低），已废弃（对应 §硬件 注：USART1 配了但没用） |
| CH340（旧方案） | 模块损坏（`error -71` / 无法枚举），已废弃 |

## 注意事项

1. **低电平开火**：接线时 PA12 碰 GND 会误触发一次；接线前拔波箱电池。
2. **DAP-LINK 的串口线不要并到 PA2/PA3**：会和 Jetson 的 TX 对顶；SWD 调试不受影响。
3. **开火数据与调试日志同口**：日志是 ASCII，解析器按帧头同步不会误判，但别在日志里写出 `0xA5/0x5A` 字节。
4. **OVERTIME 锁存**：按住超过 5 s 会强制停火，必须松开再按才能恢复——这是防烧波箱的保护，不是 bug。
5. **方案 ① 没有 5 s 导通上限**（见「开火链路」对比表）；方案 ① 的 `F` 键是 `XQueryKeymap` **轮询**实现，
   X11 自动重复不会干扰，空格（停手）时按 F 不响应。
6. **PA5/PA6/PA7（SPI1）与 PA9/PA10（USART1）** 虽然被 CubeMX 初始化，但代码里没用，别当空闲脚。

## 仓库结构

```
biu/
├── Core/
│   ├── Inc/            寄存器/外设头文件（main.h、gpio.h、usart.h…）
│   └── Src/
│       ├── main.c      ★ 开火协议/状态机/安全逻辑（USER CODE 区）
│       ├── gpio.c      ★ FIRE(PA12) 默认输出高（关火）
│       ├── usart.c     ★ USART2 接收中断使能
│       ├── stm32f4xx_it.c  ★ USART1/USART2_IRQHandler
│       └── system_stm32f4xx.c
├── Drivers/            STM32F4 HAL 驱动
├── cmake/              CubeMX 生成的构建脚本
├── CMakeLists.txt / CMakePresets.json
├── startup_stm32f401xc.s
├── STM32F401xx_FLASH.ld
├── biu.ioc             CubeMX 工程
└── README.md           本文件（协议 / 接线 / 构建 / 台架自测 / 排障 / 实测 / 注意事项）
```

## 相关文档

- 整车链路与上位机：`gimbal/README.md`（Jetson 侧，含 `--fire-script` / `--fire-pin` 火力参数）、
  `ctl/README.md`（笔记本侧，`F` = 按住发射）；
- 底盘与云台的完整文档：`chassis/docs/`（底盘工程仓库的文档目录）。
