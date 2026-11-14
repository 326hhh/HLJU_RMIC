# 27RM_AUTO_AIM —— 27RM 校内赛水弹小车 · 自动瞄准视觉

USB 摄像头（**800×600 MJPG**）→ 灯条检测 → 装甲板 PnP 位姿 → 平滑滤波 →
**反小陀螺四板环目标跟踪/预测** → 终端输出 / 调试可视化。

本项目只做**小装甲板识别 + 位置姿态解算**（校内赛不设大装甲板），不做云台控制；
输出相机坐标系下的距离 / yaw / pitch / 板面偏角 / 中心坐标，供上层控制自行使用。

- 参考工程：VSAG 练习框架（`git.simba-robotics.com/mzxwww/VSAG_AUTO_AIM_PRACTICE_27RM`）、
  `sp_vision_25`、`jlu_vision_26`（检测沿用 jlu 的传统 CV 灯条管线）
- 相机：Sonix USB2.0 Camera（`0c45:6368`），`/dev/video0`
- 平台：Jetson Nano（L4T 4.9.253-tegra，g++ 8.4，OpenCV 4.8）+ 笔记本（OpenCV 4.10）
- **当前状态（2026-11-07）**：已完成 800×600 相机标定（RMS 0.409px），实机卷尺
  1m 验证通过（距离/yaw 与真值一致，效果非常好）；装甲板 W=67.40mm 实测确认。

## 目录结构

```
27RM_AUTO_AIM/
├── README.md                 # 本文件（总览）
├── CMakeLists.txt            # 构建脚本（5 个程序）
├── chessboard_10x7_15mm_A4.pdf  # 棋盘格打印文件（100% 比例打印，备用）
├── inc/                      # 头文件（见 inc/README.md）
│   ├── camera_usb.hpp        # USB 摄像头 V4L2 直采封装
│   ├── camera_calib.hpp      # 标定数据加载 / 去畸变 / PnP 位姿
│   ├── light_bar.hpp         # 灯条检测器（传统 CV）
│   ├── aim_filter.hpp        # 位姿平滑 + 目标状态机
│   └── target_tracker.hpp    # 反小陀螺：4 板环模型目标跟踪
├── src/                      # 源码（见 src/README.md）
│   ├── main.cpp              # ★ 自瞄主程序（输出名 27RM）
│   ├── camera_usb.cpp
│   ├── camera_calib.cpp
│   ├── light_bar.cpp
│   ├── capture_calib.cpp     # 标定图采集
│   ├── calibrate.cpp         # 内参计算（含自动剔除坏图）
│   ├── detect_test.cpp       # 检测验证 / 调参
│   └── pose_test.cpp         # 几何链路验证
├── configs/                  # 配置（见 configs/README.md）
│   ├── camera.yaml           # 标定内参（由 calibrate 生成，已入库）
│   ├── camera_runtime.yaml   # 曝光 / 白平衡 / 采集格式
│   ├── light_bar.yaml        # 灯条检测参数
│   ├── armor.yaml            # 装甲板尺寸
│   └── target.yaml           # 反小陀螺参数（环半径/前瞻/转速上限等）
└── tools/
    ├── start_all.sh          # 三件套一键起（gimbal + 27RM），支持 --27rm-daemon
    ├── run_27rm.sh           # ★ 27RM 守护：崩了 2 秒自动拉起（比赛必用）
    ├── aim_udp_listen.py     # 自瞄报文监听取证（40002）
    ├── make_chessboard.py    # 棋盘格 PDF 生成器（见 tools/README.md）
    └── usb_recover.sh        # 相机 USB 卡死一键恢复
```

> `build/`、`calib_images/`、`*.jpg` 为产物/临时文件，不参与分发。

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

- **Jetson 上**：其 OpenCVConfig.cmake 会 `find_package(CUDA)` 并要求
  `CUDA_TOOLKIT_ROOT_DIR`，直接编译视觉用不到 CUDA，加开关改用 pkg-config：

  ```bash
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DRM27_USE_PKGCONFIG=ON
  cmake --build build -j4
  ```

- 想要 clangd 索引：加 `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`。

## 程序清单

| 程序 | 用途 |
|---|---|
| `27RM` | **自瞄主程序**：采集 → 检测 → PnP → 平滑 → 反小陀螺跟踪/预测 → 输出（内置 MJPEG 图传） |
| `capture_calib` | 相机标定图采集（USB 摄像头，棋盘格；GUI/`--headless`） |
| `calibrate` | 标定计算：`calib_images/` → `configs/camera.yaml`（自动剔除坏图） |
| `detect_test` | 检测验证/调参：实时 / 静态图 / 合成自检，可热重载参数 |
| `pose_test` | 几何链路验证：4 角点 → 距离/yaw/pitch，含合成自检 |

## 快速上手

```bash
# 0) 合成自检（不需要相机/标定，验证代码链路）
./build/pose_test   --selftest
./build/detect_test --selftest

# 1) 标定（需要显示器；本队板：10×7 内角点、15mm 方格）
./build/capture_calib                      # 举板扫遍画面中心/四角/边缘，空格存 25~30 张
./build/calibrate --cols 10 --rows 7 --square <卡尺实测方格边长>

# 2) 卷尺验证（装甲板放量好的 1.00m，看距离/yaw 是否对上）
./build/pose_test --size 67.40x28.05

# 3) 检测调参
./build/detect_test                        # 现场按 c 切颜色、r 热重载参数

# 4) 跑自瞄
./build/27RM                               # 有显示器
./build/27RM --no-gui                      # 无窗口（ssh 跑）
./build/27RM --bench --frames 300          # 只测采集帧率（不需要标定）
```

> 程序默认在工程根目录运行（`configs/` 与 `calib_images/` 是相对路径）。
> 完整用法、输出解读、调参与排障见下方「[使用手册](#使用手册怎么跑怎么看怎么调)」。

## 相机标定

### 棋盘格规格（本队实际用板）

- **10×7 内角点**（= 11 列 × 8 行黑白方格），方格边长 **15.0 mm**，
  整板物理尺寸约 200 × 150 mm
- 打印文件：`chessboard_10x7_15mm_A4.pdf`（备用），打印时选 **100% / 实际大小**；
  打完用卡尺量一个方格，按实测值传给 `calibrate --square`
- 重新生成（换方格尺寸/行列数）：`python3 tools/make_chessboard.py --help`

### 拍摄要求（四条缺一不可）

| 维度 | 做法 | 不满足的后果 |
|---|---|---|
| **画面位置** | 中心、四角、边缘都要有，板角接近画面边 | 畸变/主点解不准，cx 会飘 |
| **倾角** | 正对/上下倾/左右倾/旋转，倾斜占多数 | 焦距与主点耦合，fx 不稳 |
| **距离** | 板占画面宽度 60% ~ 20%（比例 ≥2 倍） | **fx 锚不牢**：距离相近时焦距×距离可整体缩放，优化器落到错误值且 RMS 仍很低 |
| **质量** | 清晰不糊、光照均匀无反光 | 角点顺序/位置错乱，一张坏图就能把整组标定带偏 |

**25~30 张**起步，越多样越好。

### 标定结果判读

| 重投影 RMS | 评价 |
|---|---|
| < 0.3 px | 很好 |
| 0.3 ~ 0.5 px | 可接受 |
| > 0.5 px | 建议删差图重拍 |

`calibrate` 会打印 **`[覆盖度]` 报告**（角点横向/纵向覆盖率、棋盘格大小比例），
覆盖率 <80% 或大小比例 <1.5 会给出 ⚠️ 建议；并**自动迭代剔除坏图**
（单张误差最大者，直到 RMS ≤0.45），剔除清单会打印出来。RMS 低 ≠ 参数对，
最终用卷尺验证。

### 标定现状（2026-11-07）

| 项 | 值 |
|---|---|
| 分辨率 / 板 | 800×600 / 10×7 内角点 15mm |
| 有效图 | 56 张中自动剔除 9 张坏图 |
| RMS | **0.409 px** |
| fx / fy | **1241.74 / 1247.54** |
| cx / cy | 536.15 / 368.89 |
| 畸变 k1 k2 p1 p2 k3 | -0.564 0.569 -0.0055 0.0197 -0.539 |
| 验证 | 装甲板放卷尺 1.00m 正对 → 距离/yaw 与真值一致 ✅ |

## 相机 USB 稳定性（重要）

这台 Sonix 相机（`0c45:6368`）的 **640×480@120fps MJPG 档很不稳定**，
所以**默认用 800×600 MJPG**。已踩过的坑与对策：

1. **`cv::VideoCapture` 的 V4L2 后端**读几帧后 `select() timeout` 卡死 →
   本项目用**直接 V4L2 mmap 采集**（`camera_usb.cpp`，与 `v4l2-ctl` 同款方式）。
2. **`cap.set(CAP_PROP_FPS, 120)` / `v4l2-ctl --set-parm=120` 会把设备 parm
   卡在 120.101 并停流**——代码不设帧率，交给相机自己跑。
3. **UVC 控制项（曝光/白平衡）提交偶发失败**（`Failed to set UVC probe
   control : -32`），帧被截断或停流 → 控制项统一在**开始取流之前**设置。
4. **USB 自动挂起**会加剧问题 → 对相机/hub/根 hub 关闭（脚本已含）：

   ```bash
   sudo sh -c "echo -1 > /sys/module/usbcore/parameters/autosuspend"   # 全局
   for d in 1-2 1-2.3 usb1; do
     sudo sh -c "echo on > /sys/bus/usb/devices/$d/power/control"
   done
   ```

5. **自动降级**：MJPG 模式 2 秒内拿不到 5 帧时，程序会自动关闭重开并降级为
   **YUYV（同分辨率，标定内参通用）**，打印警告后按 30fps 继续跑。
6. **卡死恢复**：拔插摄像头（断电 10 秒）或软复位：

   ```bash
   sudo sh -c 'echo 0 > /sys/bus/usb/devices/1-2.3/authorized'
   sleep 1
   sudo sh -c 'echo 1 > /sys/bus/usb/devices/1-2.3/authorized'
   ```

   或直接跑 `./tools/usb_recover.sh`（重新枚举 + 关挂起 + 验证）。

`27RM --bench` 在 2 秒收不到新帧时会报错退出，不会无限占着相机。

## 主程序 `27RM`

每 10 Hz 打印一行（节选）：

```
[锁定] 距离 1.234m | yaw +3.20° | pitch -1.10° | 位置(+35,+12,+1230)mm | 板偏角 2.3° | 重投影 1.2px | 检测 6.5ms | 采集 48fps | 处理 45fps | 模式:单板
[丢失] 无目标（灯条候选 0）| 检测 4.2ms | 采集 60fps | 模式:反小陀螺
```

- `[确认中]`：连续命中帧数未达 `acquire_hits`（默认 3 帧，防单帧误检上锁）
- `[锁定]`：目标有效；`yaw` 向右为正、`pitch` 向上为正（相机系）
- `板偏角`：板面法线与视线夹角（0° = 正对相机）

| 选项 | 说明 |
|---|---|
| `--calib FILE` / `--lightbar FILE` / `--armor FILE` / `--runtime FILE` / `--target FILE` | 各配置文件路径 |
| `--device N` / `--width W` / `--height H` | 相机编号与分辨率（默认 0 / **800×600**） |
| `--format mjpg/yuyv` | 强制采集格式（默认 MJPG 优先） |
| `--color blue/red` | 敌方颜色（默认 blue） |
| `--no-gui` | 无窗口（无显示器时用） |
| `--frames N` | 采集 N 帧后退出（测速/自动化） |
| `--bench` | 只测采集帧率，不做检测（不需要标定） |
| `--track-test` | 反小陀螺合成自检（不用相机/标定） |
| `--anti-spin` | 启动即开启反小陀螺模式（默认关；运行中按 `u` 切换） |
| `--save-frame F` | 抓一帧存 PNG 后退出（离线调参用） |

按键（GUI）：`q`/ESC 退出 | `u` 切换 单板锁定/反小陀螺 | `c` 切换敌方颜色 | `r` 重载灯条参数 | `s` 截图。

## 反小陀螺（目标跟踪）

> **模式切换：默认「单板锁定」——PnP 锁最近的装甲板；按 `u` 切到「反小陀螺」**
> （4 板环预测瞄准）。`--anti-spin` 可让程序启动即进入反小陀螺模式。
> 因为很多队伍不做小陀螺，平时用单板模式就够；遇到会转的目标再按 `u`。

敌方机器人做「小陀螺」（底盘原地旋转、云台反向补偿保持朝向）时，4 块装甲板
一直在转圈。反小陀螺就是**从少量观测反推整圈板的位置并预测未来位置**，实现锁定。

### 模型

- 敌方机器人 4 块装甲板排在水平环上：中心 c、半径 R、相位 φ、转速 ω
- 每块板中心 = `c + R·dir(φ + k·90°)`，板面法线 = `dir(φ + k·90°)`（指向板外侧）
- 检测到任意 1~2 块板即可：板中心 p、板面法线 n（PnP 解出）→
  **机器人中心 `c = p - R·n`**；板的角度给出相位，相位变化率给出转速 ω
- 同时看到 2 块板时在线估计半径 R：`R = (n₂-n₁)·(p₂-p₁) / |n₂-n₁|²`

### 滤波器

- 中心/速度：6 维线性卡尔曼（匀速模型）
- 相位/转速：2 维卡尔曼（相位残差按 π 回绕），转速钳制在 `max_spin_rate_dps`
- 关联：观测板中心与 4 个预测板位置最近匹配（门限 `assoc_gate_mm`）
- 0.8 秒没有新观测自动复位（防误检后"假锁定"）

### 预测瞄准

每帧用 `t + predict_time_s` 预测整圈板位置，选出**板面最朝向相机**的板，
输出其预测 `yaw/pitch`（相机系）。终端额外打印：

```
[反小陀螺] 中心(+10,+120,+1980)mm R=150mm 转速+342°/s | 预测瞄准[板2] yaw+3.12° pitch-0.58° (t+40ms)
```

### 配置与验证

- 参数在 `configs/target.yaml`：**`radius_mm` 必须实测**（机器人中心到板面中心的
  水平距离，拿卷尺量）
- 合成自检：`./build/27RM --track-test` —— 模拟 4 板环旋转机器人（R=150mm，
  转速 6 rad/s，每帧随机看到 1~2 块板，带噪声），验证中心/转速/半径估计与
  +40ms 预测位置误差；当前 Jetson 上：中心 2.4mm、转速 2.5°/s、半径 2.5mm、
  预测位置 4.2mm ✅
- GUI 里会画：预测环（橙色线）+ 4 块预测板位置（绿点=朝向相机，橙点=背对）

### 限制

- 相位/中心都在**相机系**（无云台/世界外参），云台大角度转动时预测会有偏差
- 板面法线由小板 PnP 估计，距离远时噪声大，中心估计会被拉偏
- 装甲板板面朝外的假设成立（RM 装甲板灯条都在外侧）

## 使用手册（怎么跑、怎么看、怎么调）

### 1. 首次准备（只需一次）

1. **相机标定**：`camera.yaml` 已有（见「标定现状」），换相机/换分辨率才需要重标；
2. **装甲板尺寸**：`configs/armor.yaml`（当前 W=67.40 / L=28.05 / 灯条宽 5.85mm）；
3. **反小陀螺半径**：`configs/target.yaml` 的 `radius_mm` —— 拿卷尺量
   敌方机器人**中心 → 装甲板板面中心**的水平距离，填进去（默认 150mm 是占位值）。

### 2. 启动命令

```bash
cd ~/stmproject/27RM_AUTO_AIM
```

| 场景 | 命令 |
|---|---|
| 接显示器：实时画面 + 自瞄 | `./build/27RM` |
| ssh 远程无窗口（只打印） | `./build/27RM --no-gui` |
| 跑 N 帧自动退出 | `./build/27RM --no-gui --frames 500` |
| 只测采集帧率（不用标定） | `./build/27RM --bench --frames 300` |
| 反小陀螺合成自检（不用相机） | `./build/27RM --track-test` |
| 抓一帧存图（离线调参） | `./build/27RM --no-gui --save-frame test.png`（无显示器必须加 `--no-gui`） |
| 换相机编号/分辨率 | `./build/27RM --device 1 --width 1280 --height 720` |
| 强制采集格式 | `./build/27RM --format yuyv` |
| 打红方 | `./build/27RM --color red` |
| 启动即开反小陀螺 | `./build/27RM --anti-spin`（平时默认单板，运行中按 `u` 切） |
| **闭环自瞄（直接控云台）** | `./build/27RM --gimbal-aim`（需 gimbal 守护进程在跑；见「云台闭环自瞄」） |
| **控制+图传+自瞄一起开** | Jetson: `./tools/start_all.sh --device can0 --27rm-daemon`（CAN 云台 + 27RM 守护）；PC: `./build/gimbal_ctl --host <Jetson IP>`（见「三件套一起开」「守护启动与比赛开机流程」） |
| 图传（控制端看画面） | `./build/27RM` 默认开 `:8080`，浏览器/gimbal_ctl 开 `http://<jetson-ip>:8080/stream` |
| 换图传端口 / 关闭 / 降帧率 | `--stream-port 8090` / `--stream-port 0` / `--stream-fps 15` |
| **图传只走某个网卡** | `--stream-bind wlan0`（比赛：图传只走 WiFi，有线访问不到 8080） |
| 自瞄数据输出（给控制端） | 默认子网广播 `192.168.2.255:40002`（50Hz）；`--aim-udp off` 关闭；**`--aim-udp auto` 按 wlan0 自动算广播**；多目标用逗号：`--aim-udp "auto,192.168.2.255:40002"` |

> ⚠️ **同一时刻只能一个程序占相机**（跑之前 `pgrep -af 27RM` 确认没有残留）。

### 3. 输出怎么看

终端每 10Hz 打印两行：

```
[锁定] 距离 1.234m | yaw +3.20° | pitch -1.10° | 位置(+35,+12,+1230)mm | 板偏角 2.3° | 重投影 1.2px | 检测 6.5ms | 采集 48fps | 处理 45fps
  [反小陀螺] 中心(+10,+120,+1980)mm R=150mm 转速+342°/s | 预测瞄准[板2] yaw+3.12° pitch-0.58° (t+40ms)
```

| 字段 | 含义 |
|---|---|
| `[锁定]/[确认中]/[丢失]` | 单块装甲板状态（确认中=连续命中帧数还不够，防单帧误检） |
| `距离 / yaw / pitch` | 板中心距离（m）与角度（°）：yaw 向右为正、pitch 向上为正（相机系） |
| `位置(x,y,z)` | 板中心在相机系坐标（mm） |
| `板偏角` | 板面法线与视线夹角（0°=正对相机） |
| `重投影` | PnP 重投影误差（px，越小越可信） |
| `检测/采集/处理` | 单帧检测耗时、相机实测帧率、主循环帧率 |
| `[反小陀螺]` 行 | 仅在跟踪有效时出现：机器人中心/R/转速 + 预测瞄准角 |

GUI 覆盖层：绿灯条+绿框 = 检测到的装甲板；**橙色圆环** = 预测的机器人环；
**绿点** = 各板预测位置中板面朝向相机的（可打），**橙点** = 背对相机的。

按键：`q`/ESC 退出 | `c` 切换敌方颜色 | `r` 热重载 `light_bar.yaml` | `s` 截图。

### 4. 反小陀螺怎么用

- **平时默认「单板锁定」**：PnP 直接锁最近的装甲板（输出 `模式:单板`）
- **按 `u` 切到反小陀螺**（输出 `模式:反小陀螺`）：从 1~2 块板反推整圈并预测瞄准；
  再按 `u` 切回单板（会重置跟踪器，避免旧目标残留）
- 敌方转小陀螺时，检测到任何 1~2 块板，第二行就会出现（中心/半径/转速）；
- **`预测瞄准 yaw/pitch` 就是你要打的角度**（已含 `predict_time_s` 提前量），
  接云台时把这两个角送过去即可；
- 看「转速」和实际是否一致：真实小陀螺约 300~700°/s，差太多检查 `radius_mm`
  和 `max_spin_rate_dps`；
- 目标「总打不到前面」→ 加大 `predict_time_s`（0.06~0.08）；
  过冲（总打过）→ 减小。

### 5. 调参速查

`configs/target.yaml`（反小陀螺）：

| 参数 | 调法 |
|---|---|
| `radius_mm` | **实测**，最重要；填错则中心/预测位置按比例偏 |
| `predict_time_s` | 打提前量；处理/击发延迟大就加大 |
| `max_spin_rate_dps` | 转速上限（默认 900°/s）；估计被限制住就调大 |
| `center_accel_std` | 中心跟手程度：抖动大调大、太黏调小 |
| `phase_accel_std_deg` | 转速估计的跟手程度 |
| `assoc_gate_mm` | 观测↔预测关联门限；误关联多就调小 |
| `center_meas_std_mm` / `phase_meas_std_deg` | 观测噪声；法线噪声大时调大中心噪声 |

`configs/light_bar.yaml`（检测）：颜色 HSV 范围（现场按 `c` 切颜色）、
`min_length`（远景灯条太小检测不到就调小）、`min_ratio`、配对门限。

`configs/camera_runtime.yaml`（相机）：曝光 `exposure_absolute_100us`（30=3ms）、
白平衡；`format: yuyv` 可强制降级。

### 6. 验证流程（按顺序）

```bash
# 1) 算法自检（不用相机）
./build/pose_test   --selftest
./build/detect_test --selftest
./build/27RM        --track-test

# 2) 相机采集/帧率
./build/27RM --bench --frames 300

# 3) 空场景（应：无目标，无 [反小陀螺] 输出）
./build/27RM --no-gui --frames 200

# 4) 卷尺验证（装甲板放量好的 1.00m 正对：距离≈1.00m、yaw≈0°）
./build/detect_test                       # 有显示器
./build/detect_test --no-gui --frames 300 # 无显示器：跑 300 帧打印距离/角度

# 5) 实车：板子静止/旋转，看距离与转速读数是否符合实际
./build/27RM
```

### 7. 排障表

| 现象 | 原因 / 处理 |
|---|---|
| 启动就退、`cv::imshow` 报错 | 没有 DISPLAY：`ssh -Y` 或 `--no-gui` / `--headless` |
| 相机打不开 / 2 秒无帧 | 相机 UVC 卡死：拔插（断电 10s）或 `./tools/usb_recover.sh` |
| 提示「自动降级 YUYV」 | MJPG 档卡死，程序已降级 30fps 继续跑；恢复需拔插 |
| 提示「标定分辨率不一致」 | 采集分辨率≠标定分辨率：在对应分辨率下重标 |
| 反小陀螺不出现 | 检测没出板：先 `./build/detect_test` 调 HSV/换颜色 |
| 反小陀螺中心乱跳/转速不对 | `radius_mm` 填错、法线噪声大（调大 `center_meas_std_mm`） |
| 一直假锁定 | 已内置 0.8s 无观测自动复位；仍出现说明有持续误检，查灯条参数 |
| 画面卡 | 800×600 已是默认；相机 60fps 下 CPU 占用低，卡在别处就 `top` 查 |
| 图传连不上 | 27RM 没开图传（`--stream-port 0`）或端口被占（gimbal 的 mjpeg_server 也在 8080）；换端口或关掉占用的程序 |
| 图传很卡/断流 | `--stream-fps 15` 降帧率；无线热点下带宽吃紧（800×600@30 ≈ 20Mbps） |
| WASD 控制不了 / ctl 显示 OFFLINE | gimbal 守护进程没起来（没收到 UDP 40001）。常见原因：`/dev/ttyUSB0`（CH340）没插 → 用全 CAN 模式：`./tools/start_all.sh --gimbal-args "--can can0"`（需 ZDT 切 CAN 模式）；或插回 CH340 用默认 `./tools/start_all.sh`。ctl 侧急停/失能过要按 `j`（底盘使能）/`x`（解除急停） |

## 图传（MJPEG，控制端看画面）

27RM **内置 MJPEG-over-HTTP 服务器**：27RM 占着相机时，控制端也能看画面
（不用再起单独的 `mjpeg_server`）。

- 默认地址：`http://<jetson-ip>:8080/stream`（**30 fps**），
  `gimbal_ctl` 客户端默认就是拉这个地址 → 直接兼容
- **画面带检测覆盖层**：检测到的灯条/装甲板框、预测环（反小陀螺）、
  十字准星、瞄准文字（距离/yaw/pitch/模式）都会画进图传画面——像标定窗口一样
- 覆盖层在 27RM 里画好再编码（`cv::imencode`，仅在有客户端时、限速 30fps）
- 浏览器直接打开该地址也能看；同 27RM 的窗口可以同时开着

```bash
# 默认：自瞄 + 图传 8080
./build/27RM

# 只图传不显示窗口（ssh 场景）
./build/27RM --no-gui

# 换端口 / 关闭 / 降帧率
./build/27RM --stream-port 8090
./build/27RM --stream-port 0          # 关闭图传
./build/27RM --stream-fps 15          # 无线带宽紧张时降到 15fps

# 只在一个网卡上提供图传（比赛常用：图传只走 WiFi，有线访问不到 8080）
./build/27RM --stream-bind wlan0      # 按网卡名，DHCP 换 IP 也不用改
./build/27RM --stream-bind 192.168.1.107   # 也可以直接给 IP
```

`--stream-bind` 的语义：

| 写法 | 效果 |
|---|---|
| 不给（默认） | 绑 `0.0.0.0`，有线/无线都能看 |
| `--stream-bind wlan0` | **只在该网卡的 IPv4 上监听**（启动时用 `getifaddrs` 解析，DHCP 换地址自动跟随） |
| `--stream-bind <IPv4>` | 只在该地址监听 |
| 网卡名/IP 解析失败 | 打一行告警后**退回 `0.0.0.0`**（不会因为绑不上就没画面） |

启动日志会打印实际绑定，例如：
```
[图传] 只监听 192.168.1.107:8080（wlan0）
[图传] MJPEG 服务已启动: http://<jetson-ip>:8080/stream （30 fps）
```

| 指标 | 数值 |
|---|---|
| 帧率 | 30.0 fps（实测，客户端可调） |
| 帧大小 | 约 82 KB/帧（800×600） |
| 带宽 | 约 2.4 MB/s ≈ 20 Mbps（网线直连没问题；无线热点吃紧时降帧率/分辨率） |
| CPU | MJPG 直转发几乎零开销；YUYV 降级编码约 5~8 ms/帧（仅 30fps 限速时） |

> 端口冲突：`gimbal_ctl` 配套的 `~/gimbal` 里也有 `mjpeg_server`（同样 8080），
> 两个别同时开；用 `./tools/usb_recover.sh` 或 `--stream-port` 换端口即可。

## 自瞄输出（UDP，给控制端）

27RM 把瞄准结果以 **50 Hz、48 字节小端报文** 发到 UDP 端口（默认子网广播
`192.168.2.255:40002`）。控制端收到后即可驱动云台（把 yaw/pitch 换算成
`gimbal_ctl` 的上行指令，见「控制链路」）。

### 报文格式（48 字节，全部小端）

| off | size | type | 字段 | 含义 |
|---|---|---|---|---|
| 0 | 4 | u32 | magic | `0x524D3732`（"27RM"） |
| 4 | 4 | u32 | seq | 递增序号（算丢包用） |
| 8 | 4 | f32 | aim_yaw_deg | 预测瞄准 yaw（°），**右正**（相机系） |
| 12 | 4 | f32 | aim_pitch_deg | 预测瞄准 pitch（°），**上正** |
| 16 | 4 | f32 | dist_m | 目标板距离（米） |
| 20 | 4 | f32 | spin_rate_dps | 反小陀螺转速（°/s，0=不转） |
| 24 | 4 | i32 | center_x_mm | 机器人中心 x（相机系，mm；跟踪无效时 0） |
| 28 | 4 | i32 | center_y_mm | 同上 |
| 32 | 4 | i32 | center_z_mm | 同上 |
| 36 | 4 | i32 | radius_mm | 机器人环半径（mm；跟踪无效时 0） |
| 40 | 1 | u8 | valid | 1=有可打目标 |
| 41 | 1 | u8 | flags | bit0=反小陀螺跟踪中；bit1=选中板朝向相机 |
| 42 | 1 | u8 | plate_slot | 选中的板序号（0..3） |
| 43 | 1 | u8 | reserved | 0 |
| 44 | 4 | u32 | reserved2 | 0 |

- 无目标时 `valid=0`（其余字段为 0 或缓存值），控制端据此停火/急停
- `valid=1` 的含义：单板模式下 = 已锁定最近板；反小陀螺模式下 = 跟踪器已锁定
- `flags.bit0`（反小陀螺跟踪中）只在反小陀螺模式下可能为 1
- 发送地址用 `--aim-udp` 指定，**支持逗号分隔多个目标**（一条报文同时发到所有目标）：

  | 写法 | 效果 |
  |---|---|
  | `--aim-udp auto` | **按 `wlan0` 的地址+掩码自动算广播地址**（换路由器/换网段不用改命令行） |
  | `--aim-udp auto@eth0` | 同上，但指定网卡名 |
  | `--aim-udp 192.168.2.255:40002` | 指定子网广播（默认值就是这个） |
  | `--aim-udp 192.168.2.1:40002` | 单播给某一台 PC |
  | `--aim-udp "auto,192.168.2.255:40002"` | **两套链路同时发**：比赛 WiFi 的客户端和有线调试的笔记本都能收 |
  | `--aim-udp off` | 关闭 |

  启动日志会逐条打印解析结果，方便现场确认：
  ```
  [自瞄UDP] auto -> 192.168.1.255:40002（网卡 wlan0 的广播地址）
  [自瞄UDP] 自瞄数据输出到 192.168.2.255:40002
  [自瞄UDP] 共 2 个目标，50 Hz，48 字节/包
  ```
  非法地址会被跳过并告警（不会影响其它目标）。
- 与图传、控制互不干扰（图传走 TCP:8080，控制走 UDP:40001）

### 接收示例

最简单的验证：`python3 tools/aim_udp_listen.py`（监听 40002 并解析打印）。
C++ 接收：`bind(0.0.0.0:40002)`，按上表逐字节解析（注意小端、48 字节）。

### 控制端开发者需要的东西（交付清单）

| 内容 | 位置 |
|---|---|
| 图传接口 | 本文「图传（MJPEG）」：`http://<jetson>:8080/stream`，multipart JPEG |
| 自瞄数据 | 本节：UDP 40002，48 字节报文 |
| 云台/底盘控制协议 | `~/gimbal/include/gimbal/link/control_link.hpp`（上行 24B/下行 64B，UDP 40001） |
| 参考客户端实现 | `~/gimbal_ctl`（Linux X11 + `win/` Windows 原生客户端，已含 MJPEG 显示与 UDP 控制） |
| 一键启动 | `./tools/start_all.sh`（见「三件套一起开」） |
| 调试脚本 | `./tools/aim_udp_listen.py`（自瞄数据）、`candump can0`（CAN）、`curl :8080/stream`（图传） |

## 三件套一起开（控制 + 图传 + 自瞄）

三件套的物理分工：

```
PC 笔记本                                Jetson Nano
┌──────────────┐   UDP 40001（控制）    ┌──────────────────────┐
│  gimbal_ctl  │ ─────────────────────► │  gimbal 守护进程       │ → 云台 / 底盘
│  控制 + 图传 │ ◄── MJPEG :8080/stream ┤  27RM（自瞄 + 图传服务）│ → 装甲板位姿/反小陀螺
└──────────────┘                        └──────────────────────┘
```

- **控制**：`gimbal_ctl`（PC）↔ `~/gimbal` 守护进程（Jetson），UDP 40001
- **图传**：27RM 内嵌 MJPEG 服务器（Jetson `:8080`），`gimbal_ctl` 窗口自动显示
- **自瞄**：27RM 检测/位姿/反小陀螺，与图传同进程

### 启动

```bash
# ① Jetson：一键起「gimbal 守护进程 + 27RM（自瞄+图传）」
cd ~/stmproject/27RM_AUTO_AIM
./tools/start_all.sh                     # 默认 --device /dev/ttyUSB0（底盘+云台同串口）
./tools/start_all.sh --chassis can0      # 混合后端：云台串口 + 底盘 CAN
./tools/start_all.sh --no-gimbal         # 不接硬件，只起自瞄+图传
./tools/start_all.sh --stop              # 全部停止

# ② PC：控制端（视频会自动来自 Jetson 的 8080）
cd ~/gimbal_ctl
./build/gimbal_ctl --host 192.168.2.2    # 控制 + 视频（默认）
./build/gimbal_ctl --host 192.168.2.2 --no-video   # 只要控制
```

- `gimbal_ctl` 窗口里按 `v` 开关视频；视频就是 27RM 的实时画面（含检测框）
- 想用守护进程的交互命令（`s`/`q`）就单独前台跑 `~/gimbal/build/gimbal`
- 图传/自瞄日志：Jetson 上 `/tmp/gimbal.log`、`/tmp/27rm.log`

### 注意

| 事项 | 说明 |
|---|---|
| **别同时跑 `mjpeg_server`** | `~/gimbal` 里那个图传程序和 27RM 抢相机、抢 8080，用 27RM 的图传即可 |
| 端口 | 8080 归 27RM；换端口 `./tools/start_all.sh --27rm-args "--stream-port 8090"` |
| 无线 | 图传约 20Mbps，热点吃紧时 `--27rm-args "--stream-fps 15"` 或降分辨率 |
| 启动顺序 | 谁先谁后都行；ctl 先起会显示「无视频」，按 `v` 或重启 ctl 即可重连 |

## 守护启动（run_27rm.sh）与比赛开机流程

### 为什么要有守护

27RM 一退出，**图传（:8080）和自瞄（UDP 40002）会一起消失**。2026-11-08 实测它自己挂过两次
（日志停在"检测 x ms"中途、没有退出信息；`sudo dmesg` 里有带 `Comm: 27RM` 的内核告警），
所以提供 `tools/run_27rm.sh`：**崩了默认 2 秒自动拉起**。

```bash
# 推荐：用 start_all 的开关（它会用正确的 detach 方式起守护）
cd ~/stmproject/27RM_AUTO_AIM
./tools/start_all.sh --device can0 --27rm-daemon

# 或者只守护 27RM
nohup setsid sh ./tools/run_27rm.sh >/dev/null 2>&1 &

# 看状态
pgrep -a -x 27RM
cat /tmp/run_27rm.pid          # 守护自己的 pid
tail -f ~/27rm_loop.log        # 守护日志：启动/退出/为什么重启
tail -f /tmp/27rm.log          # 27RM 自己的输出（检测/采集/自瞄发送端）

# 停止（顺序别反：先杀守护，否则它立刻把 27RM 拉起来）
./tools/start_all.sh --stop
# 或手动： pkill -f "[r]un_27rm\.sh"; pkill -x 27RM
```

| 情况 | 守护的处理 |
|---|---|
| 正常退出 / 被杀 | 等 `RM27_DELAY`（默认 2 s）重启 |
| 运行 <5 s 就退出（秒退） | 判为起不来（相机/端口问题），退避 `3×DELAY`；连续 10 次放弃并写日志 |
| `/dev/video0` 不存在 | 先等相机枚举（最多 30 s），不硬起刷日志 |
| 已有守护在跑 | pid 文件 `/tmp/run_27rm.pid` 判断，直接退出（不会两份抢相机） |

参数可用环境变量覆盖：`RM27_ARGS`（默认
`--stream-port 8080 --stream-bind wlan0 --aim-udp auto,192.168.2.255:40002 --no-gui`）、
`RM27_LOG`、`RM27_STDOUT`、`RM27_DELAY`、`RM27_PIDFILE`。

> 写这类脚本的两个坑：**别用 `pgrep -f run_27rm.sh` 判重**（会把含这串字的 ssh 命令也匹配成守护，
> 导致它误以为"已有守护"直接退出）；**`pkill -f "xxx.sh"` 会把执行这条命令的 shell 自己也杀掉**，
> 要用 `pkill -f "[x]xx.sh"` 绕开。

### 比赛现场开机流程（照抄）

**Jetson（车）**

```bash
# 1) 连比赛 WiFi
sudo nmcli device wifi connect SR-602-2G4 password ********
ip -br addr show wlan0                 # 记下 IP，例如 192.168.1.107

# 2) 一键起：gimbal（CAN）+ 27RM（守护）
cd ~/stmproject/27RM_AUTO_AIM
./tools/start_all.sh --device can0 --27rm-daemon

# 3) 三查
pgrep -x gimbal; pgrep -x 27RM
ss -lnt | grep 8080                    # 应只有 wlan0 的 IP:8080
ss -lnu | grep 40001                   # 只能 1 个监听
```

**笔记本**

```bash
sudo nmcli device wifi connect SR-602-2G4 password ********
sudo iw dev wlp14s0 set power_save off       # ⚠️ 不关的话 RTT 抖到 100 ms，控车一顿一顿
iw dev wlp14s0 get power_save                # 必须是 off
ip route get <Jetson IP>                     # 若走 dev Meta(Clash)：加绕行规则
sudo ip rule add to 192.168.1.0/24 lookup main priority 8000

cd ~/gimbal_ctl && ./build/gimbal_ctl --host <Jetson IP>
```

> ⚠️ **`start_all.sh --device can0` 和 `~/run_gimbal.sh` 二选一**：两个都会起 gimbal，
> 同时开会造成**两个 gimbal 抢 CAN、`0.0.0.0:40001` 两个监听**（2026-11-08 实测踩过）。
> 想让 gimbal 也自愈就用 `~/run_gimbal.sh`（内置 `--device can0`），此时 `start_all` 加 `--no-gimbal`。

## 控制链路（gimbal_ctl ↔ gimbal 守护进程）

控制不是 27RM 管，而是 `~/gimbal` 守护进程负责：

```
gimbal_ctl (PC) ──UDP 40001──► gimbal 守护进程 (Jetson) ──► 云台(ZDT 步进) + 底盘(F407)
                              ▲
                              └─ 27RM 的图传(:8080) 只是给 ctl 看画面，与控制无关
```

### 三种后端组合（看你的硬件接线选）

| 组合 | 命令 | 需要什么 |
|---|---|---|
| 默认：云台+底盘共用 CH340 | `./build/gimbal --device /dev/ttyUSB0` | CH340 USB 串口（`/dev/ttyUSB0`） |
| **混合（推荐）**：云台 UART + 底盘 CAN | `./build/gimbal --device /dev/ttyUSB0 --chassis can0` | CH340 + CANable（can0） |
| **全 CAN**：云台 ZDT + 底盘都在 can0 | `./build/gimbal --can can0` | CANable + **ZDT 步进电机已切 CAN 模式** |

> CAN 后端需要先起 can0（CANable 2.0，SLCAN）：
> `sudo slcand -o -s8 -t hw /dev/ttyACM0 can0 && sudo ip link set can0 up`
> （`ttyACM*` 编号随插拔变化，看 `ls /dev/ttyACM*`；`candump can0` 可看底盘反馈帧 `0x110~0x114` 确认底盘在线）

配合一键脚本：

```bash
./tools/start_all.sh                                    # CH340 默认
./tools/start_all.sh --chassis can0                     # 混合（云台串口 + 底盘 CAN）
./tools/start_all.sh --gimbal-args "--can can0"         # 全 CAN（无 CH340 也能跑）
```

### ctl 侧操作前提

| 键 | 说明 |
|---|---|
| `W/S/A/D` | 底盘 前进/后退/左移/右移（默认已使能） |
| 方向键 | 云台 yaw/pitch |
| `j` | 底盘使能切换（**急停后要按 `j` 恢复**） |
| `g` | 云台使能切换（ctl 显示 `GIMBAL DISABLED/LOCKED` 时按 `g` 两下重新授权） |
| `x` | 急停（云台+底盘立即停） |
| `空格` | HOLD（云台/底盘/发射一起停） |
| `F`（按住） | 水弹发射 |

### 排障：ctl 显示 OFFLINE / WASD 不动

按顺序查：

1. **守护进程在跑吗**：Jetson 上 `pgrep -af "build/gimbal"`、`ss -uln | grep 40001`
2. **日志**：`tail -f /tmp/gimbal.log`——最常见是 `打开 /dev/ttyUSB0 失败`（CH340 没插）→ 换全 CAN `--gimbal-args "--can can0"` 或插回 CH340
3. **网络**：ctl 的 `--host` 对不对（有线 `192.168.2.2` / 热点 `10.42.0.1`）；Jetson 热点每次开机要手动 `sudo nmcli con up car-ap`
4. **急停/失能状态**：ctl 上按 `x` 解除急停，按 `j` 使能底盘（ctl 显示 `[ESTOP]`/`[DISABLED]` 就是这两类）
5. **云台不动但底盘能动**：ZDT 步进电机没切 CAN 模式（全 CAN 时）或 CH340 没插（UART 时）——云台链路问题，底盘不受影响
6. **串口是独占的**：旧守护进程没退干净，新进程打不开串口 → 先 `pkill -f "build/gimbal"` 再起

## 云台闭环自瞄（27RM 直接控云台）

之前 27RM 只算角度不闭环；`--gimbal-aim` 打开后，**27RM 检测到装甲板就直接驱动云台
转过去跟踪**：

```
27RM 检测装甲板 → PnP 位姿/反小陀螺预测 → 瞄准偏差 yaw/pitch
    → 角速度 = Kp × 偏差（死区/限幅）→ UDP 24 字节指令 → gimbal 守护进程 → 云台电机
```

- 报文与 `~/gimbal/include/gimbal/link/control_link.hpp` 完全一致（上行 24B，
  `yaw_cdps/pitch_cdps` + Enable/Disable flag + XOR 校验），默认发到
  `127.0.0.1:40001`（本机守护进程），50Hz
- 无目标时发 **0 角速度 + Enable**（云台保持不动）；退出时发 **Disable**

### 使用

```bash
# ① Jetson 先起云台守护进程（按你的接线选后端）
./tools/start_all.sh --gimbal-args "--can can0"     # 或默认 CH340 / --chassis can0

# ② 再起 27RM 闭环自瞄（无显示器就 --no-gui）
./build/27RM --no-gui --gimbal-aim
#   想同时开反小陀螺预测：--gimbal-aim --anti-spin
#   通过 start_all.sh 一起起：
./tools/start_all.sh --gimbal-args "--can can0" --27rm-args "--gimbal-aim"
```

### 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--gimbal-aim` | 关 | 开启闭环自瞄 |
| `--gimbal-host` / `--gimbal-port` | `127.0.0.1` / `40001` | 守护进程地址 |
| `--gimbal-kp` | 20.0 | 比例增益（°/s 每 ° 偏差）；抖动大就调小 |
| `--gimbal-yaw-max` / `--gimbal-pitch-max` | 180 / 120 | 角速度限幅（°/s） |
| `--gimbal-deadband` | 0.5 | 瞄准死区（°），小于此角误差不发指令 |
| `--gimbal-invert-yaw` / `--gimbal-invert-pitch` | 关 | 方向反了时翻转 |

### ⚠️ 注意

1. **闭环模式下不要再开 gimbal_ctl 手动控制**——两边同时发 40001 会抢指令（谁后到谁生效）
2. 云台软限位（pitch ±20°）、急停、堵转保护仍由 **gimbal 守护进程**负责；27RM 只发角速度
3. 相机必须装在云台上、指向与云台一致（相机系 → 云台系近似重合）
4. 方向验证：目标在画面右侧但云台往左转 → 加 `--gimbal-invert-yaw`；上下同理
5. 单板模式和反小陀螺模式都适用：`--gimbal-aim` 与 `--anti-spin` 可同时开

## 实测性能（Jetson Nano）

| 项目 | 结果 |
|---|---|
| 采集 **800×600 MJPG（默认）** | **≈ 48 ~ 60 fps** |
| 采集 640×480 MJPG | ≈ 97 ~ 99.6 fps（该档不稳定，随时可能卡死） |
| 采集 320×240 MJPG | ≈ 118.5 fps |
| 采集 1280×720 MJPG | ≈ 38 ~ 59 fps |
| 采集 1920×1080 MJPG | ≈ 14 fps |
| 采集 640×480 YUYV | ≈ 29 ~ 30 fps（MJPG 卡死时的降级档） |
| 单帧检测耗时 | ≈ 6 ~ 11 ms（视场景灯条数量） |
| 端到端主循环 | 800×600 采集时能跑满 ~48-60fps |

## 踩坑记录（写代码时别重蹈）

1. **OpenCV 4.8（Jetson）的 `findChessboardCornersSB` 有 bug**：会抛
   `(-215) result.full() in function 'getNeighbors'`（flann kdtree 断言），
   且 800×600 下实测 381ms/帧。所以捕获/标定工具都**经典检测器为主**，
   SB 仅兜底并 try/catch；单帧异常只警告不崩进程。
2. **`calibrate` 只认 jpg** 而 `capture_calib` 存 PNG → 已让两者都支持 png/jpg。
3. **低 RMS 不等于参数对**：主点/焦距会互相抵消，必须靠「覆盖度报告 +
   卷尺验证」把关……一张 45px 误差的坏图就能让 fx 从 1000 多暴涨到 6000+。
4. **GUI 程序没有 DISPLAY 会直接崩**（`cv::imshow` 断言）→ 四个工具都加了
   启动检查：无图形环境时打印中文提示（`--headless`/`--no-gui`/`ssh -Y`）。
5. **曝光/白平衡值**：UVC 的 `exposure_auto` 自动=3、手动=1（0/2 会 EINVAL）。

## 已知限制 / 未做

- 只支持小装甲板（校内赛要求），不做大装甲分类
- 反小陀螺**默认关闭**（按 `u` 开启）——大部分队伍不做小陀螺，平时单板锁定就够
- 反小陀螺没有弹道补偿（不建模水弹飞行时间），只有时间前瞻 `predict_time_s`
- 位姿/跟踪都在**相机坐标系**，没有相机→云台/世界的外参（云台大角度转动时预测会有偏差）
- 27RM 本身**不直接控制云台**：瞄准结果经终端打印/自瞄 UDP（40002）给控制端，
  由控制端（gimbal_ctl 或自制程序）把 yaw/pitch 换算成 40001 指令
- 检测器只返回一个最优目标（多目标场景不处理；单帧最多输出 1~2 块同一机器人板用于反小陀螺）
- MJPG 640×480 档不稳定（已改 800×600）；更换相机请重做标定

## 参考与子目录文档

- 检测管线思路来自 jlu_vision_26（HSV 灯条 + PCA 角点修正 + 配对门控）
- 坐标系/角点约定与 VSAG 练习框架保持一致（`CameraCalib::solvePose`、`poseToYawPitch`）
- 目录级说明：[`inc/README.md`](inc/README.md) · [`src/README.md`](src/README.md) ·
  [`configs/README.md`](configs/README.md) · [`tools/README.md`](tools/README.md)

## 更新记录（2026-11-07）

| 批次 | 内容 |
|---|---|
| 基建 | USB 摄像头 V4L2 直采（800×600 MJPG ~48-60fps，MJPG 卡死自动降级 YUYV）、灯条检测（经典 CV）、IPPE PnP、棋盘格标定工具链（10×7 / 15mm，含覆盖度报告与坏图自动剔除） |
| 标定 | 800×600 标定完成（RMS 0.409px，fx=1241.74 fy=1247.54），装甲板 W=67.40mm 实测确认，卷尺 1m 验证通过 |
| 反小陀螺 | 4 板环模型跟踪（1~2 块板反推整圈）+ 转速估计 + 预测瞄准；默认关闭，按 `u` 切换；`--track-test` 合成自检 |
| 图传 | 内嵌 MJPEG-over-HTTP（`:8080/stream`，30fps，MJPG 零转码），控制端/gimbal_ctl/浏览器直接看 |
| 自瞄输出 | UDP 50Hz 48 字节报文（`40002`，子网广播；支持 `auto`/多目标），给控制端对接；`tools/aim_udp_listen.py` 接收调试 |
| 一键启动 | `tools/start_all.sh`：gimbal 守护进程 + 27RM（自瞄+图传）一起起，含全 CAN 变体；`--stop` 停止 |
| 控制链路 | gimbal_ctl ↔ gimbal 守护进程（UDP 40001）；三种后端组合（CH340 / 混合 / 全 CAN）；ctl 按键与排障写入文档 |
| 模式切换 | 默认单板锁定（PnP 锁最近板）；`u` 键切反小陀螺；`--anti-spin` 启动即开 |
