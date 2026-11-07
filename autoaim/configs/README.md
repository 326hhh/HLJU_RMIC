# configs/ —— 配置说明

| 文件 | 何时生成/维护 | 被谁读取 |
|---|---|---|
| `camera.yaml` | `./calibrate` 生成（标定结果，800×600 / RMS 0.409） | 27RM / detect_test / pose_test |
| `camera_runtime.yaml` | 手工维护（相机参数） | 27RM / detect_test / pose_test / capture_calib* |
| `light_bar.yaml` | 手工维护（检测参数） | 27RM / detect_test |
| `armor.yaml` | 手工维护（装甲板尺寸） | 27RM / detect_test / pose_test |

> 程序默认在工程根目录运行，以下路径均为相对路径，可命令行覆盖。

## camera.yaml（标定结果）

由 `./calibrate` 生成并**整体重写**，不要手改（除了补注释）。

当前版本（2026-11-07，800×600，10×7 内角点 15mm，RMS 0.409px）：
`fx=1241.74 fy=1247.54 cx=536.15 cy=368.89`，
`k1=-0.564 k2=0.569 p1=-0.0055 p2=0.0197 k3=-0.539`。关键键：

| 键 | 含义 |
|---|---|
| `camera_matrix` | 3×3 内参 K |
| `dist_coeffs` | 1×5 畸变 [k1,k2,p1,p2,k3] |
| `image_width/height` | 标定图分辨率（必须与运行分辨率一致） |
| `rms` / `square_mm` / `board_cols` / `board_rows` | 标定质量与板规格（记录用） |

## camera_runtime.yaml（采集参数）

| 键 | 默认 | 说明 |
|---|---|---|
| `format` | （无） | `mjpg` / `yuyv`；不写则 MJPG 优先（640×480 只有 MJPG 能到 ≈99fps） |
| `exposure_auto` | 1 | 0=自动 1=手动（比赛建议手动，否则 HSV 阈值漂移） |
| `exposure_absolute_100us` | 30 | 手动曝光时间，单位 100µs（30=3ms；帧周期 10.1ms，别超） |
| `white_balance_auto` | 0 | 0=手动 1=自动（手动才能让颜色稳定） |
| `white_balance_temperature` | 4600 | 色温 K |
| `brightness` / `contrast` / `saturation` / `gamma` / `sharpness` | -10 / 40 / 80 / 120 / 2 | 其余 V4L2 控制项 |

⚠️ 这些控制项在**开始取流之前**应用（Open 内部完成）；该 UVC 相机对
「流中改控制项」很敏感，会停流。相机掉线/停流时的恢复见根目录 README。

## light_bar.yaml（检测参数）

### 颜色（OpenCV HSV：H∈[0,180]，S/V∈[0,255]）

| 键 | 默认 | 说明 |
|---|---|---|
| `enemy_color` | blue | 敌方颜色；打红方队时改 `red`（或现场按 `c`） |
| `blue_h_min/h_max` | 100/140 | 蓝色 H 范围 |
| `blue_s_min` / `blue_v_min` | 60 / 60 | 蓝色 S/V 下限 |
| `red_h_min_1/red_h_max_1` | 0/12 | 红色 H（跨 0 一端） |
| `red_h_min_2/red_h_max_2` | 168/180 | 红色 H（跨 180 一端） |
| `red_s_min` / `red_v_min` | 60 / 60 | 红色 S/V 下限 |

### 形态学 / 灯条筛选 / 配对

| 键 | 默认 | 说明 |
|---|---|---|
| `morph_kernel` / `morph_iters` | 3 / 1 | 闭运算；iters 别开 2，远景灯条会被抹掉 |
| `min_length` | 5.0 | 灯条最小长度 px（28.05mm 灯条 1.5m 处 ≈9px） |
| `min_ratio` | 1.4 | 长/宽最小比；阈值化会把掩膜撑宽，别设太高 |
| `max_tilt_deg` | 45 | 与竖直方向最大夹角 |
| `min_area` / `max_area` | 8.0 / 200000 | 轮廓面积范围（px²） |
| `max_angle_diff_deg` | 15 | 两灯条倾角差 |
| `max_len_ratio` | 2.0 | 两灯条长度比上限 |
| `max_center_y_ratio` | 0.6 | 中心纵向差 / 平均灯条长 |
| `center_dist_ratio_min` / `_max` | 1.2 / 3.2 | 中心距/平均灯条长（小装甲 W/L≈2.40） |

### PCA 角点精修

| 键 | 默认 | 说明 |
|---|---|---|
| `use_pca` | 1 | 是否启用 |
| `pca_required` | 0 | 1=两侧灯条必须都精修成功才配对（小板别开：灯条太细精修不了） |
| `pass_optimize_lightbar_width` | 2 | 灯条宽超过此值才尝试精修 |
| `normalize_max_brightness` | 25.0 | ROI 归一化亮度上限 |
| `lightbar_min_mean_brightness` | 0.0 | ROI 平均亮度下限（太暗直接放弃） |
| `padding_scale` | 0.07 | 扩框比例 |
| `search_start_ratio` / `search_end_ratio` | 0.4 / 0.6 | 端点搜索区间（相对灯条长） |

## armor.yaml（装甲板尺寸，毫米）

| 键 | 值 | 说明 |
|---|---|---|
| `width_mm` | 67.40 | 两灯条中心距 W（实测；错 1% 测距错 1%） |
| `height_mm` | 28.05 | 灯条长度 L |
| `bar_width_mm` | 5.85 | 灯条宽度（检测门限/合成自检图用） |

> 校内赛只有小装甲板，无大小分类；换板子只需改这里。
