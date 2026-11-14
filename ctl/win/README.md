# gimbal_ctl (Windows) —— 原生 Win32 客户端

Windows 上的云台/底盘控制客户端：**Win32 + Winsock + GDI + stb_image**，单文件 `main_win.cpp`，
静态链接、不需要装任何运行库（Win10/11 64 位直接双击）。

线协议**不复制**：`#include "gimbal/link/control_link.hpp"`，靠 `-I../../gimbal/include`
吃 Linux 版同一个源头（`GIMBAL_ROOT` 可覆盖）。

> ⚠️ **这份是 2026-11-07 的版本，和 Linux 版（`../src/main.cpp`）有一批功能差距**，见
> [与 Linux 版的差距](#与-linux-版的差距)一节 —— 用之前先看，别以为两边一样。

## 文件

| 文件 | 作用 |
|---|---|
| `main_win.cpp` | 全部逻辑（约 1360 行）：UDP 控制 100 Hz、MJPEG 拉流线程、GDI 双缓冲渲染、键盘轮询、`gimbal_ctl.ini` |
| `stb_image.h` | v2.30 单头文件 JPEG 解码（只开 `STBI_ONLY_JPEG`），免得在 Windows 上折腾 libjpeg |
| `build_win.sh` | 交叉编译脚本：优先找 zig（自带 mingw-w64 sysroot），退回 `x86_64-w64-mingw32-g++` |
| `gimbal_ctl.ini` | 配置（主机 IP / 速度 / 平滑 / 窗口 / 小陀螺 / 枪口方向），全中文注释，改完重新双击生效 |
| `start.cmd` | 双击启动器：`chcp 65001` + 非零退出码时 `pause`（否则闪退看不到红字） |
| `readme.txt` | 给队友的说明书（连接方式、按键、排障、安全提示） |
| `dist/gimbal_ctl_win.zip` | 打包好的分发件（exe + ini + start.cmd + readme.txt） |

## 编译

```bash
./build_win.sh                        # 默认 -O2，产物 build/gimbal_ctl.exe
./build_win.sh --debug                # -O0 -g
GIMBAL_ROOT=~/gimbal ./build_win.sh   # 线协议头文件不在 ../../gimbal 时
```

没有 zig 也没 mingw 时脚本会直接报错退出（需要 `sudo apt install g++-mingw-w64-x86-64`，
或把 zig 解压到 `~/tools/`）。产物是 **PE32+ console 程序**，只依赖
`GDI32/KERNEL32/USER32/WINMM/WS2_32` + UCRT。

## 运行

```cmd
gimbal_ctl.exe --selftest          :: 先跑这个：纯协议 + UDP 回环，不开窗口
gimbal_ctl.exe                     :: 双击即可，默认 192.168.2.2:40001
gimbal_ctl.exe --host 10.42.0.1    :: 走 Jetson 热点
gimbal_ctl.exe --no-video          :: 不要视频
```

双击不方便带参数，所以支持同级目录的 `gimbal_ctl.ini`（`host=` / `gimbal_rate=` / `size=1280x720` …，
完整 key 见该文件内的注释）。**ini 永远从 exe 所在目录读**，快捷方式放桌面也没关系。

窗口出来后**先点一下窗口**拿焦点：左上角出现红字 `NOT FOCUSED` 就是没拿到，
此时键盘一律无效（`GetAsyncKeyState` 是全局的，不加这个判断你在浏览器里打字会把车开走）。

## 按键

| 键 | 作用 |
|---|---|
| `←/→` | 云台 yaw 左 / 右（按住转，松开平滑停） |
| `↑/↓` | 云台 pitch 抬头 / 低头 |
| **`W/S`** | **沿枪口指向前进 / 后退**（默认驾驶系，见下） |
| `A/D` | 枪口左右侧横移 |
| `Q/E` | 底盘左转 / 右转（小陀螺模式下让位给自旋） |
| **`Shift`** | 按一下切换「快速/普通」档（锁存）：普通 300 mm/s、1.5 rad/s；快速 600 mm/s、3.0 rad/s |
| **`R`** | 小陀螺模式开关（底盘自旋 + yaw 反向补偿让相机站住） |
| **`T`** | 切换驾驶坐标系：**枪口方向 ↔ 车头方向** |
| **`Y`** | 把**当前枪口指向**标定为"正前"（校正枪口零点，`gun_offset`） |
| **`F`**（按住） | 水弹发射（按住连发；HOLD 时不响应） |
| **空格** | 停手 / 继续（HOLD，云台+底盘+发射一起停） |
| `g` / `j` | 云台 / 底盘 使能开关（底盘急停后按 `j` 恢复） |
| `x` | 急停（云台 + 底盘，立即停 + 失能） |
| `h` / `z` | 云台回中 / 当前位置归零（`z` 同时把 `gun_offset` 复位为 0） |
| `k` | 清空当前速率（不回中） |
| `c` | 底盘里程计清零 |
| `v` | 视频开关 |
| `Esc` | 退出（退出前自动发停手帧） |

## 枪口方向驾驶（本版特有）

方向盘不是"车头朝哪走哪"，而是**枪口朝哪走哪**（麦轮能横移，所以车头会歪着走，正常）：

```
theta  = gun_sign × (yaw_deg − gun_offset)      // yaw_deg = 云台相对车体的转角
vx_car = u_f·cosθ − u_l·sinθ                    // u = 键盘给的"枪口系"速度
vy_car = u_f·sinθ + u_l·cosθ
```

- 先在**枪口系**做加速度限幅（平移 3000 mm/s²、转向 6000 mrad/s²），再旋转下发，手感一致；
- `T` 随时切回"车头方向"老实开；
- **遥测断流时 `theta` 无从得知 → 自动退回车体系**，界面红字
  `NO TELEM -> fallback to chassis`；
- **左右镜像了怎么办**：云台往左转 90° 再按 `W`，车应该"横着往左走"；若往右前方走，
  说明转角符号相反 → 把 ini 里 `gun_sign` 改成 `-1`（`A/D` 会一起纠正）。

校准步骤：把枪口对准车头正前 → 按 `Y`（或按 `z` 让云台归零并复位 `gun_offset`）→
记下角度改进 ini 的 `gun_offset` 就不用每次按。

## 界面

- 视频按**等比缩放居中**贴在窗口里（窗口比画面大/小都不变形）；
- 正中画**准星**（十字 + 圆点，暗色描边 + 亮绿），指针锁定视角下靠它瞄；
- 左上角状态栏：`mouse`→ 无（本版没有鼠标控制）、`yaw/pitch`、`motor`、`link`、`chassis`、
  `video`、`aim`→ 无、以及 `** HOLD ** / ** FAST ** / ** SPIN ** / ** FIRE ** / ** GIMBAL DISABLED **`；
- 每秒往控制台打一行 `[video] 绘制 x fps 拉流 y fps`，无头也能看链路状态。

## 与 Linux 版的差距（重要）

| 功能 | Linux 版 | Windows 版 |
|---|---|---|
| 云台输入 | **鼠标（主控）** + 方向键叠加，指针锁定、隐形光标 | 只有方向键 |
| 鼠标左键开火 | ✅（三重门控） | ❌ 只有 `F` 键 |
| 27RM 自瞄 UDP 40002 | ✅ 显示 + `t` 键辅助 | ❌ 未接 |
| pitch 降灵敏 / 软限位减速 | ✅ `--pitch-scale` / `--pitch-limit` / `--pitch-margin` | ❌ 未实现 |
| 屏幕准星 | ✅ | ✅ |
| 枪口方向驾驶（`T`/`Y`） | ❌ | ✅ |
| 小陀螺 / 快速档 | ✅ | ✅ |
| 配置方式 | 命令行参数 | 命令行 + `gimbal_ctl.ini` |

> 两边共用同一个 `control_link.hpp`，线协议一定一致；差的是**输入方式与显示**。
> 想让 Windows 也有鼠标控云台/左键开火/自瞄，需要把 Linux 版那几段移植过来（未做）。

## 已知问题 / 注意

1. **本环境无法运行验证**：Linux 侧没有 wine，只能在 Windows 上实测；本仓库只做了编译期检查
   （`-Wall -Wextra` 干净）。
2. 没签名的自制 exe 首次运行可能被 SmartScreen 拦：点「更多信息 → 仍要运行」。
3. 静态链接跑在 Win10/11 64 位没问题；Win7 需要 UCRT 更新。
4. 视频解码用 `stb_image`，**只支持 baseline JPEG**；UVC 摄像头的 MJPG 都是 baseline，
   遇到 progressive 会解码失败并在控制台打出来。
5. `--mode grab|warp`、`--sens`、`--invert-y` 等老参数在 Windows 版里**不存在**（Linux 版才有）。
