# gimbal_ctl —— 云台/底盘控制客户端（PC 端）

原生 X11 小程序：**鼠标（主控）/ 方向键 → 换算指令 → UDP 发给 Jetson → 显示遥测 + MJPEG 视频 + 自瞄数据**。

## 2026-11-08 本轮更新（鼠标控云台 / 自瞄 / 左键开火 / 准星 / pitch 保护）

| 变化 | 一句话 |
|---|---|
| **鼠标控云台**（主控输入） | 移动鼠标 = 给云台角速度，指针锁在窗口中心（可以一直朝一个方向甩）；方向键保留叠加。`m` 键临时关掉 |
| **自瞄数据接入** | 收 27RM 的 `UDP 40002`（48 B @ 50 Hz），状态栏显示 valid / 瞄准角 / 距离 / 转速；`t` 键开"辅助"，按 aim 角比例把云台怼过去 |
| **左键开火** | 按住鼠标左键连发（`F` 保留备用）；三重门控防误触：指针在窗口内 + 窗口有焦点 + 上一拍已有焦点 |
| **屏幕准星** | 窗口正中十字 + 圆点（黑描边 + 亮绿）；指针锁定时没有光标，靠它瞄 |
| **pitch 降灵敏 + 软限位** | `--pitch-scale 0.5`（半速）；离 ±20° 限位 5° 内**线性降速**，不再一头撞上服务端硬夹 |
| **调试参数** | `--mouse-debug` 打印鼠标采样与开火判定；`--mouse-ignore-focus`、`--raw-motion` |

> 本轮 **Jetson 侧没动**（`~/gimbal` 未改；只在 2026-11-08 单独修过"退出后 yaw 电机还转"的退出流程，见 `~/gimbal/src/main.cpp` 注释）。

## 目录结构

```
gimbal_ctl/
├── CMakeLists.txt   # 构建脚本：C++17、X11 + libjpeg，线协议头文件复用 ../gimbal
├── README.md        # 本文件：使用说明与目录/文件说明
├── LICENSE          # MIT License（Copyright (c) 2026 saki）
├── .gitignore       # 忽略 build/、*.o、*.log、.vscode/、.DS_Store
├── include/         # 公共头文件（mjpeg_view.hpp、aim_view.hpp），见 include/README.md
├── src/             # 实现源码（main.cpp、mjpeg_view.cpp、aim_view.cpp），见 src/README.md
├── win/             # Windows 原生客户端 + 交叉编译脚本 + ini，见 win/README.md
└── build/           # CMake 构建输出目录（自动生成，已 gitignore，可删）
```

## 根目录文件说明

| 文件 | 作用 |
|---|---|
| `CMakeLists.txt` | 工程构建脚本：C++17、Release 默认；`find_package(X11)` + `pkg_check_modules(libjpeg)`；把 `include/` 与 `${GIMBAL_ROOT}/include` 加入头文件目录（`GIMBAL_ROOT` 默认 `../gimbal`，可用 `-DGIMBAL_ROOT=<路径>` 覆盖）；编译 `src/main.cpp` + `src/mjpeg_view.cpp`，链接 X11 与 libjpeg。 |
| `README.md` | 本文件。使用说明（编译/运行/按键/视频/协议）见下文；目录与文件级说明见 `include/README.md`、`src/README.md`。 |
| `LICENSE` | MIT 许可证。 |
| `.gitignore` | 忽略 `build/`、`*.o`、`*.log`、`.vscode/`、`.DS_Store`。 |
| `include/` | 公共头文件（C++ `.hpp`）。 |
| `src/` | 源码实现（`.cpp`）。 |
| `build/` | 构建产物目录（CMake 生成），不入库。 |

## 编译

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

依赖：X11（`libx11-dev`）、libjpeg（`libjpeg-dev`）。
线协议直接复用 `../gimbal/include/gimbal/link/control_link.hpp` —— **只有一个源头，不复制**。
gimbal 工程不在 `../gimbal` 时用 `-DGIMBAL_ROOT=<路径>`。

> 💡 想用 clangd/IDE 索引不报“找不到头文件”，配置时加上
> `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`，生成 `build/compile_commands.json` 即可。

## Windows 版（`win/`）

同一套控制逻辑的 **Windows 原生客户端**（静态链接 exe，拷过去双击就能跑，无需装运行库），
源码在 `win/`。**完整说明见 [`win/README.md`](win/README.md)**，给队友的分发说明书是 `win/readme.txt`。

```bash
# 在 Linux 上交叉编译（默认用 zig，找不到则用 mingw-w64 g++）
cd win && ./build_win.sh                    # 产物 win/build/gimbal_ctl.exe
# 或指定 gimbal 工程路径
GIMBAL_ROOT=~/gimbal ./build_win.sh
```

- 配置：`win/gimbal_ctl.ini`（host / 速度 / 平滑 / 小陀螺速率等，都有中文注释）；
- 启动：双击 `start.cmd`（或 `gimbal_ctl.exe --selftest` 先自检）；
- 键位**不完全一致**：Windows 版云台是方向键（没有鼠标控云台）、开火只有 `F`、
  没接 27RM 自瞄、没有 pitch 软限位保护；但它多一个「枪口方向驾驶」（`T`/`Y`）。
  逐项对照见 [`win/README.md`](win/README.md#与-linux-版的差距重要)。
- Windows 下没有 X11，窗口/键鼠用 Win32 API，视频解码用 `stb_image`。

## 运行

两种连法，**只差 `--host`**：

| 链路 | Jetson 地址 | 图传可用？ |
|---|---|---|
| **无线（比赛用）** | Jetson 自己开的 AP：`MECAR-CTRL` → **10.42.0.1** | ✅ **图传只在这条链路上** |
| 有线（调试用） | `192.168.2.2`（网线直连，静态） | ❌ Jetson 侧 8080 只绑 wlan0，有线会 `connection refused` |

### 无线运行（比赛：连 Jetson 的 AP，推荐）

**Jetson 侧**（开热点，`car-ap` 已设为开机自启）：

```bash
sudo nmcli con up car-ap          # SSID MECAR-CTRL / 密码 mecar12345 / 10.42.0.1/24
ip -br addr show wlan0            # 确认是 10.42.0.1
```

**笔记本侧**：

```bash
sudo nmcli device wifi connect MECAR-CTRL password mecar12345
sudo iw dev wlp14s0 set power_save off        # ⚠️ 必须：不关的话 RTT 会抖到 100 ms
iw dev wlp14s0 get power_save                 # 必须是 off
ip -br addr show wlp14s0                      # 拿到 10.42.0.x

ip route get 10.42.0.1                        # 显示 dev Meta 就是被 Clash 劫持了
sudo ip rule add to 10.42.0.0/24 lookup main priority 8000   # 网段按实际改

./build/gimbal_ctl --host 10.42.0.1
```

- **图传只走 AP**：Jetson 侧 `27RM --stream-bind wlan0` 让 8080 只绑在 wlan0（=10.42.0.1）；
- 自瞄广播用 `--aim-udp auto`，按 wlan0 的地址+掩码自动算 = **10.42.0.255**；
- 单客户端实测 30 fps / **1.6 Mbps**（800×600 编码缩放到 600×450、q60）；
- **别同时开两个看画面的客户端**（浏览器开 `:8080/stream` 也算一个），会把 2.4G 抢崩。

### 有线运行（调试用）

笔记本 `enp8s0 192.168.2.1` ↔ Jetson `eth0 192.168.2.2`，两边静态地址，开机即用。
**控制 ✅ / 图传 ❌**（8080 只绑 wlan0）。

```bash
./build/gimbal_ctl --host 192.168.2.2 --no-video     # 有线只用来调控制链路
./build/gimbal_ctl --selftest --host 192.168.2.2     # 只验 UDP 链路（不开窗口）
```

## 云台控制（鼠标为主，方向键备用）

```
移动鼠标 ──位移/时间 × --sens(0.15)──► yaw/pitch 角速度 ─┐
                                                          ├─► 相加 → 限幅 → 一阶低通 --tau → 发给 Jetson
按住 ←/→/↑/↓ ── --gimbal-rate(120°/s) 且 pitch ×0.5 ────┘
```

- **鼠标**：右移右转、左移左转、下移低头、上移抬头；指针每拍被拉回窗口中心（隐形光标），
  所以可以一直朝一个方向甩，不会顶到屏幕边缘。
- **灵敏度** `--sens 0.15 °/s per px/s`：鼠标 800 px/s ≈ 云台 120°/s。
- **方向键** `--gimbal-rate 120`：与鼠标**相加**后限幅到 `--max-rate 180`。
- **`m` 键**：临时关掉鼠标控制，把指针还给系统（去点别的窗口不用退程序）。
- **失焦即停**：窗口没有输入焦点时，鼠标和左键**都不响应**（安全设计）。

### pitch 半速 + 软限位保护

- `--pitch-scale 0.5`：pitch 灵敏度是 yaw 的一半（鼠标和方向键都乘）。
- `--pitch-limit <a> <b>`（默认 `-20 20`）：**必须与 gimbal 的 `--pitch-limit` 对齐**。
- `--pitch-margin <deg>`（默认 5）：还差这么多度就开始 `guard = (限位 − 当前角) / margin` 线性降速。
- `--pitch-max-rate <dps>`（默认 120）：pitch 指令上限，对齐 gimbal `--pitch-rate`。

为什么不是直接顶到限位：gimbal 侧 `axis.cpp` 的 `clampToLimits()` 到边界是**把速率硬夹到 0**（急停），
对电机和机构都不友好。客户端提前减速，实测角度会沿 `1.5°→19°/s、17.6°→8.2、19.4°→1.6、19.9°→0.03` 平滑贴住。

⚠️ **遥测断流时这层保护不生效**（拿不到当前角），此时只剩 gimbal 自己的硬夹。

## 按键

| 键 | 作用 |
|---|---|
| `←/→` | 云台 yaw 左 / 右（按住转，松开停） |
| `↑/↓` | 云台 pitch 抬头 / 低头 |
| **空格** | **锁死 / 解除**：锁死时云台失能 + 底盘拉低 STBY（`Disable|Hold` + `EStop`），并拒收所有输入 |
| `g` | 云台使能 / 失能切换 |
| `j` | 底盘使能 / 失能切换（**底盘急停后按 `j` 恢复**） |
| **鼠标左键**（按住） | **水弹发射**（按住连发，松开停；HOLD 时不响应；需指针在窗口内 + 窗口有焦点） |
| `F`（按住） | 水弹发射（**备用**，和左键等效） |
| `x` | **急停**（云台 + 底盘，立即停止 + 失能） |
| `h` | 云台回中 |
| `z` | 把当前位置当作 0° |
| `k` | 清空当前速率（不回中，只把速度清零；**原 `r` 键，已被小陀螺占用**） |
| `c` | 底盘里程计清零 |
| `W/S` | 底盘前进 / 后退（满速 `--chassis-speed`，默认 300 mm/s） |
| `A/D` | 底盘左移 / 右移 |
| `Q/E` | 底盘左转 / 右转（满速 `--chassis-turn`，默认 1500 mrad/s；小陀螺模式下让位给自旋） |
| **Shift** | **按一下切换「快速/普通」档（锁存，不用按住）**，详见下文 |
| **`R`** | **切换小陀螺模式**（底盘自旋 + 云台反向站住），详见下文 |
| **`C`** | **切敌方颜色 蓝↔红**（发给 27RM 命令口 40003） |
| **`N`** | 底盘里程计清零（**原 `c`，已被颜色切换占用**） |
| **`U`** | **切 27RM 的 单板锁定 / 反小陀螺**（同上，走 40003） |
| **`t`** | **自瞄辅助开关**（开=按 27RM 的 aim 角把云台怼过去；关=只显示数据） |
| **`m`** | **鼠标控制开关**（关掉把指针还给系统） |
| `v` | 视频开关 |
| `Esc` | 退出（退出前会自动发一帧停止指令） |

> 底盘坐标约定：`vx` 前进 / `vy` 左移 / `wz` 逆时针为正。

## 参数速查（本轮新增 / 语义变更）

| 参数 | 默认 | 说明 |
|---|---|---|
| `--sens <f>` | **0.15** | 鼠标灵敏度 °/s per px/s（原 0.09 废弃、0.30 已砍半） |
| `--mouse-window <s>` | 0.03 | 估鼠标速度的滑窗，越小越跟手、越抖 |
| `--mode <relative\|off>` | relative | `off` = 只用方向键（旧的 grab/warp/auto 一律按 relative） |
| `--pitch-scale <f>` | 0.5 | pitch 灵敏度倍率 |
| `--pitch-limit <a> <b>` | -20 20 | pitch 软限位，**与 gimbal 对齐** |
| `--pitch-margin <deg>` | 5 | 距限位多少度开始减速 |
| `--pitch-max-rate <dps>` | 120 | pitch 指令上限 |
| `--reticle-size <px>` / `--no-reticle` | 18 | 准星臂长 / 关掉准星 |
| `--aim-port <n>` / `--aim-off` | 40002 | 自瞄广播端口 / 不接自瞄 |
| `--aim-assist` | 关 | 启动即开自瞄辅助（等价进去按 `t`） |
| `--aim-kp <f>` | 4.0 | 辅助增益 °/s per °，同时设两轴 |
| `--aim-kp-yaw` / `--aim-kp-pitch` | — | 分轴增益，**填负值 = 该轴方向反** |
| `--aim-deadband <deg>` | 0.3 | 自瞄死区 |
| `--aim-stale <s>` | 0.25 | 超过这么久没新包就不信任 |
| `--no-send` | — | **只收不发**（观战）：27RM 用 `--gimbal-aim` 控云台时用，避免抢指令 |
| `--start-locked` | — | 启动即锁死（云台+底盘失能），按空格解除 |
| `--27rm-cmd-port <n>` | 40003 | 给 27RM 发命令的口（`C`/`U` 键） |
| `--mouse-debug` | — | 打印鼠标原始采样 + `FIRE ev/mask/ptr_in/focused` 判定 |
| `--mouse-ignore-focus` | — | 鼠标只要求指针在窗口内，不看焦点（部分 WM 拿不到焦点时用） |
| `--raw-motion` | 关 | 用 XInput2 原始位移；**XWayland 下会被自己 warp 的位移污染，别开** |

## 鼠标控制细节（踩过的坑，改之前先看）

1. **用指针位置差分，不用事件**：每拍读一次指针位置、和窗口中心做差，再把指针拉回中心。
   实测 `XWarpPointer` 对**还没 viewable 的窗口是空操作**，所以"首拍只同步不计速度"是必须的。
2. **Mutter 会把 X11 的 warp 当运动做指针加速**，warp 到中心实际会落在"中心 −104,−71"附近，
   直接当成位移会瞬间灌出 **114°/s** 的假指令。所以有：跳变门限（>20% 窗口宽高直接丢）+ 启动/切焦点静默期。
3. **XWayland 会把 warp 也报成 `XI_RawMotion`**，所以 `--raw-motion` 反而更差，默认走位置差分。
4. **焦点门**：只有「窗口有焦点 且 指针在窗口内」才接管；否则你一点别的窗口它就把指针拽回来，
   整个桌面没法用。
5. 让窗口真拿到焦点要用 `_NET_ACTIVE_WINDOW`（`XSetInputFocus` 会被 Mutter 抢回去）——调测试脚本时会遇到。

## 自瞄对接（27RM · UDP 40002，48 B @ 50 Hz）

数据来自 **`~/stmproject/27RM_AUTO_AIM`**（不是底盘工程）。报文结构体**复用**它的
`inc/aim_udp.hpp`（`-DAIM_ROOT=<路径>`，缺头文件时自瞄功能被编译掉，不影响其它功能）。

| 字段 | 说明 |
|---|---|
| `magic` | `0x524D3732`（"27RM"） |
| `aim_yaw_deg` / `aim_pitch_deg` | 预测瞄准角，**相机系，右/上为正** |
| `dist_m` | 目标板距离 |
| `spin_rate_dps` | 反小陀螺转速（只显示，不参与控制） |
| `valid` / `flags` / `plate_slot` | 有无目标 / bit0 反小陀螺跟踪中、bit1 板朝向相机 / 板序号 |

**自瞄辅助**（默认关，`t` 或 `--aim-assist`）：

```
yaw 指令   += -kp_yaw   × aim_yaw_deg      （右为正 → yaw 减小）
pitch 指令 += +kp_pitch × aim_pitch_deg    （上为正 → pitch 增大）
仅当 valid=1 且数据新鲜(< --aim-stale) 且未 HOLD 时生效，叠加在鼠标/方向键之上
```

实测（假 27RM 广播端）：aim `yaw+3.0 pitch+1.0`、`kp=4` → 正好 `yaw −12.0 / pitch +4.0 °/s`；
`valid` 翻 0 后 0.5 s 内指令归零；死区 0.3° 内不动；`--aim-kp-pitch -4` 能让 pitch 单独反号。

排障：`aim 无数据` = 27RM 没跑或端口不对；`aim 无可打目标` = 相机前没有装甲板（`valid=0` 是正常的）。

### 27RM 侧给控制端的四点约定（2026-11-08 核对过源码）

| # | 接口 | 说明 |
|---|---|---|
| 1 | **图传** `http://<jetson-ip>:8080/stream` | MJPEG 不变，但**画面已经带检测框**（装甲板/灯条/预测环/瞄准文字）：27RM 把叠加层画进 `view` 再按 `--stream-fps` 编码（JPEG q85）。客户端直接显示即可，不用自己画 |
| 2 | **自瞄数据** UDP 40002 | 48 字节报文不变；另外 **`reserved` 字节现在填当前敌方颜色**（`1=蓝 2=红`），客户端状态栏用它显示 `敌方蓝/红` |
| 3 | **27RM 自己控云台** | `--gimbal-aim` 让 27RM 直接往 UDP 40001 发速率（默认 `127.0.0.1:40001`、`--gimbal-kp` 默认 20、yaw 180 / pitch 120 上限、死区 0.5°、退出时失能）。**此时客户端必须加 `--no-send`**，否则两边同时发 40001 会抢指令（见下） |
| 4 | **命令口** UDP 40003 | 一个字节的命令：`'c'` 切蓝↔红、`'r'` 设红、`'b'` 设蓝、`'u'` 切单板/反小陀螺；27RM 回 `'k'` 作回执。客户端已接：`C` = 切蓝红、`U` = 切模式 |

> ⚠️ **为什么必须 `--no-send`**：gimbal 的控制链路只记一个 peer（回遥测用），但**不按 peer 过滤指令** ——
> 只要有人发，就执行。所以 27RM 用 `--gimbal-aim` 控云台时，客户端若还按 100 Hz 发（哪怕全是 0 速率），
> 两个源会交错，云台会抖。`--no-send` 就是"只收不发"的观战模式（已实测：假 gimbal 收 0 帧）。
> 代价：`--no-send` 时**收不到遥测**（gimbal 只回给最后一个发包方 = 27RM），只能看画面 + 自瞄。

## 视频

从 `http://<host>:8080/stream` 拉 MJPEG，libjpeg 解码后居中画在窗口里。
540p 窗口放 640×480 的画面刚好。

⚠️ 只支持常见 24/32 位 TrueColor 视觉（红=0xFF0000 绿=0xFF00 蓝=0xFF）。
启动时会检查，不匹配就只显示状态文字不显示画面。

## 状态显示

```
gimbal_ctl -> 192.168.2.2:40001   [mouse + arrows]
mouse  +412,   -38 px/s  sens 0.15  lock on  (m=toggle)
** SPIN cmd +2.00  act +1.85 rad/s ** yaw comp -106 deg/s (R)   <- 小陀螺开着才有
** FAST ** 600 mm/s  3.0 rad/s (shift toggled)                 <- 快速档才有
aim 反小陀螺 板2(朝向)  yaw  +3.00 pitch  +1.00  d 3.45m  spin -1800deg/s  50Hz lost 0
** AIM ASSIST ** yaw  -12.0 pitch   +4.0 deg/s  (t=toggle)      <- 辅助开着才有
yaw    cmd   +90.0  act   +88.5  out   +1234.56 deg
pitch  cmd   +30.0  act   +28.1  out     +12.34 deg  lim -20/+20  guard  63%
** PITCH LIMIT ** 18.2 deg 内减速（限位 -20/+20，别硬顶）        <- 进减速区才有
motor  +30.0 /  +0.0 RPM  24.10V  27C  status=0x07
link 100Hz lost 0  RTT 0.8ms  OK  rx=100Hz
chassis pos +1000,   +500 mm  yaw  +45.00  cmd  +300,   +0 mm/s
        wheel FL+100.0 FR+100.0 BL+100.0 BR+100.0 RPM
        act  +300,    +0 mm/s  gyro_yaw  +45.00  chassis ONLINE
video 14.2 fps connected
```

**status 异常时会直接标出来**：`ESTOP` / `STALL!` / `WATCHDOG!` / `PITCHLIM` / `MOTOR LOST!`，
以及 `** GIMBAL DISABLED/LOCKED **`（云台被失能/锁死，按 `g` 两下重新授权）
—— 开着车的时候你不该去数字段。

## 快速模式（Shift 键）

**按一下 Shift（左右均可）切换，锁存**；再按一下切回普通档。松开 Shift 不会退回。

| 档位 | 平移满速 | 转向满速 |
|---|---|---|
| 普通 | `--chassis-speed` 默认 300 mm/s | `--chassis-turn` 默认 1500 mrad/s |
| 快速 | `--shift-speed` 默认 600 mm/s | `--shift-turn` 默认 3000 mrad/s |

- 加速度斜坡不变（平移 3000 mm/s²、转向 6000 mrad/s²），所以切换瞬间不会窜；
- 快速档生效时状态栏多一行 `** FAST ** 600 mm/s  3.0 rad/s (shift toggled)`；
- 终端会打一行 `[chassis] 切到快速档：平移 600 mm/s / 转向 3000 mrad/s`。

> 实现用 `XQueryKeymap` 轮询做**上升沿检测**，不是靠 `KeyPress` 事件 ——
> X11 对长按的键会连发 KeyPress，用事件法会在按住期间疯狂来回切档。

## 小陀螺模式（R 键）

参考 RM 正赛：**底盘原地自旋，云台（相机）在世界坐标系里站住不转**，方向键照常控制云台。

```
按一下 R -> 底盘按 --spin-rate 自旋，yaw 电机反向补偿
方向键   -> 叠加在补偿之上，仍然控制云台（世界系）
再按 R   -> 退出，底盘平滑停转（减速段继续补偿，相机不会被甩）
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--spin-rate <mrad/s>` | `2000`（2.0 rad/s ≈ 115°/s） | 自旋速率，**正 = 逆时针**；取负值 = 反向自旋 |
| `--spin-gain <f>` | `1.0` | yaw 补偿倍率；实机发现相机"转得更快"（补偿同向叠加）时填 `-1` 翻转 |

### 补偿量怎么算的

```cpp
comp（°/s） = -spin_gain × wz（rad/s → °/s）
cc.yaw_dps  = 方向键速率 + comp
```

`wz` 的取法（**这里是踩过坑的地方**）：

1. **以指令值 `cwz` 为基准**；
2. 只有底盘陀螺**实测** wz 满足「与指令同号 且 ≥ 指令的 25%」时才改用实测值 ——
   这样地面打滑、掉速能自动吸掉；
3. 否则退回指令值。**原因**：陀螺坏掉/轴映射不对时会恒报 0，
   无脑采信实测会让补偿直接变成 0，表现就是"云台跟着车一起转"。

实测值还会过 **30 ms 一阶低通**压噪声；关掉小陀螺后 **0.6 s 内继续补偿**，
因为底盘还要约 0.33 s 才停转，不补的话相机会被甩约 19°。
按空格 HOLD 时底盘自旋会停，但补偿仍然生效（相机继续站住）。

状态栏（`act` 是低通后的实测自旋速率，`comp` 是补偿量）：

```
** SPIN cmd +2.00  act +1.85 rad/s ** yaw comp -106 deg/s (R)
```

### 已知限制

| 限制 | 数值 / 原因 |
|---|---|
| **yaw 必须能无限连续旋转** | 小陀螺期间 yaw 相对底盘一直转。若电机线/电源线没走导电滑环，会被绞紧 → 转半圈到一圈后**堵转保护**（电机状态位 `0x6B`），云台直接停住。**这是硬性条件** |
| 轮速天花板 | 自旋 2 rad/s 约占 105 RPM，快速档平移 600 mm/s 约占 143 RPM，最坏轮 ≈248 RPM 顶到 250 上限 → **小陀螺时用普通档平移** |
| 云台转速上限 | Jetson 云台默认 `--yaw-rate 180°/s`。自旋 3.0 rad/s 时补偿就吃掉 172°/s，方向键几乎没余量；要边转边瞄，Jetson 端起时加 `--yaw-rate 300` |
| 打滑补偿依赖实测陀螺 | 底盘反馈帧里的 `gyro_wz` 若恒为 0（IMU 故障），只能按指令值补偿，打滑和掉速补偿不了 |

## 现场排障手册

小陀螺/云台出问题时，按这张表查（**都是实际踩过的**）：

| 现象 | 检查 | 处理 |
|---|---|---|
| 窗口显示 `** GIMBAL DISABLED/LOCKED **` | 遥测 `status` 的 bit0=0，云台被失能/急停锁死 | **按 `g` 关再开（两下）**重新授权 |
| yaw 完全不动，但底盘正常 | Jetson 会话卡在 `kLocked`（等使能/位置反馈/启动抬升）时**整段不下发速率** | 先按 `g` 两下；仍不行看 Jetson 的 `~/gimbal_loop.log` |
| 电机状态 `0x6B` | bit3 = **堵转保护**触发，电机会拒绝执行速度命令 | `cansend can0 00000100#0E526B`（yaw）/ `...00000200#0E526B`（pitch）清除；客户端重新使能时也会自动清一次 |
| yaw 转半圈就停、状态 `0x6B` | 线缆被绞紧（没走滑环） | **机械问题**，刷软件没用，得改走线 |
| 底盘陀螺 `act` 一直 +0.00 | 底盘反馈帧 `gyro_wz` 恒 0（IMU 故障/轴映射不对） | 查底盘 LSM6DSV16X 的 I2C 与零偏，与本工程无关 |
| 左上角红字 `NOT FOCUSED`、鼠标不动 | 窗口没拿到输入焦点（鼠标和左键都故意不响应） | 用 `_NET_ACTIVE_WINDOW`/点一下窗口；排查时加 `--mouse-debug` |
| 鼠标在动但云台不转 | `--mouse-debug` 看 `ptr=` 是否在窗口内、`focused=1`；或按过 `m` 关掉了鼠标 | 点窗口 / 再按 `m` 打开 |
| 按左键不开火 | 看 `--mouse-debug` 的 `FIRE ev/mask/ptr_in/focused`：哪一项是 0 就是哪道门没过 | 指针要在窗口内、窗口要有焦点；HOLD 时空格退出 |
| `aim 无数据` | 27RM 没跑 / 端口不是 40002 / 走无线时广播地址没改 | `python3 ~/stmproject/27RM_AUTO_AIM/tools/aim_udp_listen.py` 先验 |
| `aim 无可打目标` | 27RM 在跑但相机前没装甲板（`valid=0`），或灯条候选为 0 | 属正常，放个装甲板进画面 |
| 自瞄一开就往反方向怼 | `aim_yaw/aim_pitch` 符号与本机约定相反 | `--aim-kp-yaw -4` / `--aim-kp-pitch -4` 单独翻轴 |
| pitch 到 17° 左右就不听使唤 | 进入软限位减速区（`guard < 100%`），属设计行为 | 确需更大行程：改 gimbal 的 `--pitch-limit`，再同步客户端 `--pitch-limit` |

### 直接查电机（Jetson 上）

```bash
# 查状态：返回 3A 01 6B = 使能正常；3A 08 6B 之类含 bit3 = 堵转保护
cansend can0 00000100#3A6B      # yaw（地址 1）
cansend can0 00000200#3A6B      # pitch（地址 2）

# 抓总线看谁在发命令（ZDT 电机帧是扩展帧，ID = 地址<<8，如 0x100/0x200）
candump -L can0 | grep -E "00000100|00000200"

# 观察器（本机临时工具，可能已被清理）：每 2 秒打印
#   底盘 wz 命令 / 陀螺实测 / yaw 速度命令 / 电机状态 / 四轮转速
bash ~/turbo_selftest/spin_watch.sh 90
```

> ⚠️ 抓包诊断两个坑：① ZDT 的 CAN ID（如 `0x100`）**数值小于 0x7FF**，
> 不能按"ID > 0x7FF 才是扩展帧"判断，要看 `candump -L` 打印成 8 位还是 3 位；
> ② 只在"数值变化"时打印会骗人——进程死了和空闲看起来一样，**必须打心跳/帧龄**。

## 详细文档索引

| 想看什么 | 去哪 |
|---|---|
| 公共头文件（`MjpegView` 类）说明 | [`include/README.md`](include/README.md) |
| 源码（`main.cpp` / `mjpeg_view.cpp`）说明 | [`src/README.md`](src/README.md) |
| 线协议（上行/下行逐字节格式） | `../gimbal/include/gimbal/link/control_link.hpp` |
| 云台控制端（Jetson 侧） | `../gimbal` 工程 |
| Windows 原生版（方向键控云台 + 枪口方向驾驶） | [`win/README.md`](win/README.md) |
| 自瞄数据来源（报文定义 / 监听脚本） | `../stmproject/27RM_AUTO_AIM`（`inc/aim_udp.hpp`、`tools/aim_udp_listen.py`） |
