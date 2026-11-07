# inc/ —— 头文件说明

| 文件 | 作用 |
|---|---|
| `camera_usb.hpp` | USB 摄像头封装：打开/关闭、取最新帧、运行时参数、V4L2 控制项 |
| `camera_calib.hpp` | 标定数据（内参/畸变）加载、逐帧去畸变、装甲板 PnP 位姿解算 |
| `light_bar.hpp` | 灯条检测器：HSV 掩膜 → 轮廓 → 灯条 → PCA 精修 → 配对 → 装甲板角点 |
| `aim_filter.hpp` | 位姿平滑（EMA）+ 跳变门限 + 单目标状态机（锁定/确认中/丢失） |
| `target_tracker.hpp` | 反小陀螺：4 板环模型目标跟踪（中心/半径/相位/转速 + 预测） |

> 约定：所有类/结构都在全局命名空间（检测器在 `rm27::`），单位统一为 **mm / 弧度 / 像素**，
> 坐标系为 OpenCV 相机系（x 右、y 下、z 前）。

## camera_usb.hpp

```cpp
UsbCamera cam;
cam.Open(0, 640, 480, "configs/camera_runtime.yaml");  // 打开+启动采集线程
cv::Mat img;
std::chrono::steady_clock::time_point ts;
uint64_t seq;
while (cam.GetLatestFrame(&img, &ts, &seq)) { /* 下游只拿最新帧 */ }
std::printf("fps=%.1f  %dx%d %s\n", cam.MeasuredFps(), cam.Width(), cam.Height(),
            cam.Fourcc().c_str());
```

- **用法**：`Open(device, w, h, runtime_yaml)` 内部一次完成
  「设控制项 → 设 MJPG/YUYV 格式 → mmap 4 缓冲 → STREAMON → 起采集线程」；
  后台线程 poll+DQBUF，`cv::imdecode` 后**只保留最新一帧**（自动丢旧帧，降延迟）
- **禁止**：流开始后再改 V4L2 控制项（实测会把该 UVC 相机关停流）；
  也不要设置 `CAP_PROP_FPS` / `S_PARM=120`（同理会停流，且 parm 会卡在 120.101）
- **控制项**：`ApplyRuntimeConfig()` 支持
  `format / exposure_auto / exposure_absolute_100us / white_balance_auto /
  white_balance_temperature / brightness / contrast / saturation / gamma / sharpness`；
  静态接口 `SetControl()/GetControl()` 可对任意 `/dev/videoN` 单独设置
- **自动降级**：MJPG 模式 2 秒内拿不到 5 帧（相机该档卡死）时，自动关闭重开并
  降级 **YUYV 同分辨率**（标定内参通用），打印警告后按 30fps 继续
- **实测**：800×600 MJPG ≈48~60fps（默认）；640×480 MJPG ≈97~99.6fps（该档不稳定）；
  YUYV 640×480 仅 30fps

## camera_calib.hpp

```cpp
CameraCalib calib;
calib.load("configs/camera.yaml");          // 或 loadFromMats() 用合成内参（自检用）
calib.undistort(frame, undist);             // 映射表缓存，尺寸不变只建一次
auto pose = calib.solvePose(corners, size); // corners: 左下 左上 右上 右下
```

- `solvePose` 流程：`solvePnPGeneric(IPPE)` 取全部候选 → 筛「非有限值 / z≤0 /
  重投影 > 8px」→ 取重投影最小者；IPPE 数值退化（NaN/垃圾解）时退回
  `SOLVEPNP_ITERATIVE`（两条退化路径都稳）
- `ArmorSize`：`width_mm` = 两灯条中心距，`height_mm` = 灯条长度，
  `bar_width_mm` = 灯条宽度；`LoadArmorSize()` 从 `configs/armor.yaml` 读取
- `poseToYawPitch(tvec, &yaw, &pitch)`：yaw 向右为正、pitch 向上为正（相机系）

## light_bar.hpp

- 管线（`LightBarDetector::detect()`，参数全部来自 `configs/light_bar.yaml`）：
  1. HSV 颜色掩膜（默认敌方=蓝）→ 闭运算
  2. 轮廓 → `LightBar`（`minAreaRect` + 质心 + 上下端点 + 倾角）
  3. PCA 角点精修：灰度矩求对称轴 + 沿轴亮度骤降找端点；
     **灯条太细（宽 ≤2px）时自动退回矩形端点**（`pca_ok=false`，不强制要求）
  4. 左右配对五道门：倾角差 / 长度比 / 中心纵向差 / 中心距/灯条长比 / （可选）PCA 必须成功
  5. 输出 4 角点（板左下→左上→右上→右下）与灯条调试信息
- 调试：`bars()`（灯条候选）、`mask()`（二值掩膜）、`drawDebug()`（绘制）、
  `setEnemyColor()`（现场切红/蓝）
- 关键参数与调参建议见 `configs/README.md`

## aim_filter.hpp

```cpp
AimConfig cfg;                       // ema_alpha / acquire_hits / max_lost / jump_gate
PoseFilter filter(cfg);
const AimOutput& out = filter.Update(has_pose, pose.rvec, pose.tvec, reproj);
if (out.valid) { /* out.distance_m / out.yaw_deg / out.pitch_deg / out.plate_tilt_deg */ }
```

- **EMA 平滑** tvec/距离/yaw/pitch（`ema_alpha` 越大越跟手）
- **跳变门限**：单帧距离跳变 > `jump_distance_m` 或角度 > `jump_angle_deg` →
  重置平滑（应对换目标/误检）
- **状态机**：连续命中 `acquire_hits` 帧才 `valid`；连续丢失 `max_lost` 帧后清状态
- `plate_tilt_deg`：用原始 rvec 算板面法线与视线夹角（0°=正对相机）

## target_tracker.hpp

```cpp
rm27::TargetTracker tracker("configs/target.yaml");
tracker.Update({pose.tvec, PoseNormal(pose.rvec)}, t);   // 每块检测到的板一次
auto plates = tracker.Predict(t + 0.04);                 // 预测整圈 4 块板位置
rm27::PredictedPlate aim;
tracker.SelectAimPlate(t + 0.04, &aim);                  // 选朝向相机的板
```

- 观测：板中心 + 板面法线（相机系，mm）；法线指向板外侧（远离机器人中心）
- 模型：4 板水平环，中心 `c = p - R·n`，相位/转速由板角度序列估计
- 中心/速度用 6 维卡尔曼，相位/转速用 2 维卡尔曼（相位残差 π 回绕）
- `--track-test` 合成自检验证跟踪与预测精度
