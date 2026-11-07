# src/ —— 源码说明

| 文件 | 对应程序 | 作用 |
|---|---|---|
| `main.cpp` | **27RM** | 自瞄主程序：采帧 → 检测 → PnP → 平滑 → 打印/可视化 |
| `camera_usb.cpp` | 公共库 | V4L2 直采（mmap）+ 解码 + 控制项 |
| `camera_calib.cpp` | 公共库 | 标定加载/去畸变/PnP/装甲板尺寸读取 |
| `light_bar.cpp` | 公共库 | 灯条检测器实现（`detectAll` 支持同帧多板） |
| `target_tracker.cpp` | 27RM | 反小陀螺：4 板环目标跟踪 + 预测 |
| `capture_calib.cpp` | capture_calib | 棋盘格采集（GUI / `--headless` 自动存图；经典检测器 + 60ms 限速 + 窗口 fps 显示） |
| `calibrate.cpp` | calibrate | 标定计算（读 `calib_images/` → `configs/camera.yaml`；自动剔除坏图 + 覆盖度报告；支持 png/jpg） |
| `detect_test.cpp` | detect_test | 检测验证：实时/静态图/`--selftest` 合成自检 |
| `pose_test.cpp` | pose_test | 几何验证：4 角点→位姿；`--selftest` 合成自检 |

公共库（`camera_usb/camera_calib/light_bar`）被 `27RM`、`detect_test`、
`pose_test` 复用；`capture_calib` 只用采集层，`calibrate` 纯 OpenCV 不碰相机。

## 主程序流程（main.cpp）

```
加载 camera.yaml（缺失则报错退出，必须标定）
加载 light_bar.yaml / armor.yaml / camera_runtime.yaml / target.yaml
UsbCamera.Open（控制项在取流前应用）
循环：
  GetLatestFrame → 同一帧不重复处理
  det.detectAll()（可能 0/1/2 块板）
  每块 solvePose() → 喂 TargetTracker（反小陀螺更新）
  PoseFilter.Update()（单板平滑）
  TargetTracker.SelectAimPlate(t + 前瞻)（预测瞄准角）
  10 Hz 打印（单板行 + [反小陀螺] 行）/ ~25 Hz 可视化（含预测环）
  按键：q 退出 / c 切颜色 / r 重载灯条参数 / s 截图
  0.8 秒无观测 → TargetTracker.Reset()
```

另外四种模式：`--bench`（纯测采集帧率，2 秒无帧保护）、`--track-test`
（反小陀螺合成自检）、`--save-frame`（抓一帧存 PNG）、`--frames N`（跑 N 帧退出）。

## 检测器（light_bar.cpp）

1. `findBars`：BGR→HSV → `inRange`（蓝/红两套）→ 闭运算 → 外轮廓
   → 面积/长度/长宽比/倾角过滤 → `LightBar`
2. `correctBar`（PCA 精修，`fromContour` 的矩形端点作兜底）：
   扩 ROI → 灰度矩求对称轴（指向上）→ 两端沿轴找最大亮度骤降点
3. `pairBars`：按 x 排序配对，五道门（角度差/长度比/纵向差/中心距比/PCA 要求），
   得分取「平均灯条长 − 纵向差」最大者；输出角点
   `{右条底, 右条顶, 左条顶, 左条底}`（= 板左下→左上→右上→右下，已处理面对面镜像）

## PnP（camera_calib.cpp）

- 模型点（mm）：`(0, +W/2, ∓L/2)` / `(0, -W/2, ∓L/2)`，W=灯条中心距、L=灯条长
- `solvePnPGeneric(IPPE)` 全候选筛选 → 失败退回 `SOLVEPNP_ITERATIVE`
- 重投影门限默认 8px；`distance_mm = |tvec|`

## 自检工具（detect_test / pose_test 的 --selftest）

- `pose_test --selftest`：用（标定或合成）内参把已知位姿投影成 2D 再解回来，
  验证坐标约定/角点顺序（含「交叉乱序必须被拒」用例）
- `detect_test --selftest`：按**真实尺寸**把装甲板渲染成发光灯条图（3D 投影，
  灯条宽按 5.85mm 换算像素），检测→PnP 与真值对比；另存 `detect_case*.jpg` 调试图
- 两者在没有 `configs/camera.yaml` 时用合成内参 fx=fy=480 并打印提示
