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
| `27RM` | **自瞄主程序**：采集 → 检测 → PnP → 平滑 → 反小陀螺跟踪/预测 → 输出 |
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
[锁定] 距离 1.234m | yaw +3.20° | pitch -1.10° | 位置(+35,+12,+1230)mm | 板偏角 2.3° | 重投影 1.2px | 检测 6.5ms | 采集 48fps | 处理 45fps
[丢失] 无目标（灯条候选 0）| 检测 4.2ms | 采集 60fps
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
| `--save-frame F` | 抓一帧存 PNG 后退出（离线调参用） |

按键（GUI）：`q`/ESC 退出 | `c` 切换敌方颜色 | `r` 重载灯条参数 | `s` 截图。

## 反小陀螺（目标跟踪）

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
| 抓一帧存图（离线调参） | `./build/27RM --save-frame test.png` |
| 换相机编号/分辨率 | `./build/27RM --device 1 --width 1280 --height 720` |
| 强制采集格式 | `./build/27RM --format yuyv` |
| 打红方 | `./build/27RM --color red` |

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
./build/detect_test

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
- 没有目标跟踪/弹道预测（EKF、运动补偿），只有 EMA 平滑 + 跳变门限
- 位姿在**相机坐标系**输出，没有相机→云台/世界的外参
- 没有串口/UDP 控制输出；要云台自动瞄准请在上层把 `yaw/pitch` 接过去
- 检测器只返回一个最优目标（多目标场景不处理）
- MJPG 640×480 档不稳定（已改 800×600）；更换相机请重做标定

## 参考与子目录文档

- 检测管线思路来自 jlu_vision_26（HSV 灯条 + PCA 角点修正 + 配对门控）
- 坐标系/角点约定与 VSAG 练习框架保持一致（`CameraCalib::solvePose`、`poseToYawPitch`）
- 目录级说明：[`inc/README.md`](inc/README.md) · [`src/README.md`](src/README.md) ·
  [`configs/README.md`](configs/README.md) · [`tools/README.md`](tools/README.md)
