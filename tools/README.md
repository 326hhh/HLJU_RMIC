# tools/ —— 工具说明

## usb_recover.sh —— 相机 USB 卡死一键恢复

相机「能打开但不出帧」时执行：重新枚举设备（软插拔）、关闭 USB 自动挂起、
并用本项目 `27RM --bench` 验证取流（MJPG 失败会提示降级/需物理拔插）。

```bash
./tools/usb_recover.sh              # 默认设备 1-2.3、/dev/video0
./tools/usb_recover.sh 1-2.4        # 相机换了 USB 口时指定新路径
```

> 软复位救不回来时：**物理拔插摄像头，断电 10 秒再插回**（实测唯一可靠的复位方式）。

## aim_udp_listen.py —— 自瞄 UDP 数据接收调试

```bash
python3 tools/aim_udp_listen.py          # 监听 0.0.0.0:40002
python3 tools/aim_udp_listen.py 40002    # 指定端口
```

27RM 默认以 50Hz 广播 `192.168.2.255:40002`（48 字节报文，格式见 README
「自瞄输出」）。收到后打印 seq/valid/瞄准角/距离/转速/中心/丢包，供控制端验证链路。

## start_all.sh —— 一键启动「控制 + 图传 + 自瞄」

```bash
./tools/start_all.sh                     # gimbal 守护进程 + 27RM（自瞄+图传 8080）
./tools/start_all.sh --chassis can0      # 混合后端：云台串口 + 底盘 CAN
./tools/start_all.sh --gimbal-args "--can can0"   # 全 CAN（CH340 没插时用）
./tools/start_all.sh --no-gimbal         # 只起 27RM（不接硬件）
./tools/start_all.sh --27rm-args "--stream-fps 15"   # 给 27RM 追加参数
./tools/start_all.sh --stop              # 全部停止
```

- 后台启动，日志 `/tmp/gimbal.log`、`/tmp/27rm.log`
- 无 `DISPLAY` 时自动给 27RM 加 `--no-gui`
- 启动前只清理「本次要启动的」进程（`--no-gimbal` 不会动已运行的 gimbal 守护进程）

## make_chessboard.py —— 棋盘格 PDF 生成器

纯 Python 标准库实现（无第三方依赖），直接写 PDF，保证 A4 毫米尺寸精确。

```bash
python3 tools/make_chessboard.py                          # → chessboard_10x7_15mm_A4.pdf
python3 tools/make_chessboard.py --cols 10 --rows 7 --square 15
python3 tools/make_chessboard.py --cols 11 --rows 8 --square 20 --out my_board.pdf
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--cols` | 10 | 内角点列数（方格数 = cols+1） |
| `--rows` | 7 | 内角点行数（方格数 = rows+1） |
| `--square` | 15.0 | 方格边长（mm） |
| `--out` | 自动命名 | 输出 PDF 路径 |

### 打印与使用

1. 用 PDF 阅读器打印，**选「100% / 实际大小」，不要「适应页面」**
2. 打印后用卡尺量一个方格的实际边长（打印机可能有 ±1~2% 缩放）
   - 标定：`./calibrate --square <实测值>`
3. 页面内容：11×8 方格（内角点 10×7）、外框、100mm 比例尺、说明文字
   （图案 165×120mm，A4 居中；`100mm 比例尺` 可快速目测打印是否缩放）

### 与标定流程的关系

- `capture_calib --cols 10 --rows 7` 按内角点数检测棋盘格（默认已是 10×7/15mm）
- `calibrate --cols 10 --rows 7 --square <实测mm>` 计算内参（默认已对齐）
- 换板子尺寸就重新生成 PDF，并同步改这两个参数
