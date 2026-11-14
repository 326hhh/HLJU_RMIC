# src —— gimbal_ctl 源码

本目录放**实现源码**（C++，`.cpp`），构建入口是根目录 `CMakeLists.txt`，
使用说明（编译/运行/按键）见根目录 [`../README.md`](../README.md)。

## 目录结构

```
src/
├── main.cpp         # 主程序：X11 窗口 + 鼠标/键盘控制 + UDP 链路 + 自瞄数据 + 遥测渲染 + 自检
├── aim_view.cpp     # 自瞄数据接收：UDP 40002 收 27RM 的 48 B 报文、校验/统计/只读解析
├── mjpeg_view.cpp   # 视频实现：TCP 拉 MJPEG、按 SOI/EOI 切帧、libjpeg 解码、帧缓存
└── README.md
```

## 文件说明

### `main.cpp` —— 主程序（约 1444 行）

程序职责：**鼠标/方向键（云台）+ WASD（底盘）→ 换算指令 → 打包 UDP 发给 Jetson → 收遥测/自瞄数据 → 渲染**，全程单线程主循环。

- **命令行解析**：`--host`（默认 `192.168.2.2`）、`--ctrl-port`（默认 40001）、`--http-port`
  （8080）、`--no-video`、`--tau`（平滑时间常数，默认 0.045 s）、`--max-rate`（°/s 上限，默认 180）、
  `--gimbal-rate`（方向键云台速率，默认 120 °/s）、`--chassis-speed`（底盘平移满速 mm/s，默认 300）、
  `--chassis-turn`（底盘转向满速 mrad/s，默认 1500）、`--shift-speed` / `--shift-turn`（Shift 快速档满速，默认 600 mm/s / 3000 mrad/s）、
  `--spin-rate`（小陀螺自旋速率 mrad/s，默认 2000）、`--spin-gain`（小陀螺 yaw 补偿倍率，默认 1.0）、
  `--size w h`、`--hz`（发送频率，默认 100）、`--selftest`。
  **鼠标/自瞄**：`--sens`（鼠标灵敏度 0.15 °/s per px/s）、`--mouse-window`（0.03 s 滑窗）、
  `--no-mouse-lock`、`--raw-motion`（XInput2 原始位移）、`--mouse-ignore-focus`、`--mouse-debug`、
  `--start-locked`、`--no-send`（只收不发，27RM `--gimbal-aim` 时用）、`--pitch-scale`（默认 0.5）、
  `--no-pitch-guard`、`--no-reticle` / `--reticle-size` / `--reticle-dx/dy`、
  `--aim-port`（40002）/ `--aim-off` / `--aim-kp`（4.0）/ `--aim-kp-yaw` / `--aim-kp-pitch`、
  `--aim-deadband`（0.3°）/ `--aim-bias-yaw` / `--aim-bias-pitch` / `--aim-stale`（0.25 s）/ `--aim-assist`。
  > `--mode` / `--invert-y` 仍兼容（旧 grab/warp 视作 relative）；鼠标已是主控输入，指针锁在窗口中心。
- **`keyDown()`**：`XQueryKeymap` 轮询键盘状态，用于方向键/WASD/空格等**按住**行为
  （X11 的 KeyPress 只能拿到按下事件，连按要靠轮询）。
- **`UdpClient`**：`open()` 用 `getaddrinfo` + `connect()` 建 UDP 通道；`send()` 把
  `ControlCommand` 编码后发 UDP，同时记录 `seq` 的发送时刻用于 RTT；`recv()` 非阻塞收遥测
  （`MSG_DONTWAIT`），解码失败计入坏包数，按 `echo_seq` 算 RTT。
- **`runSelftest()`**：不开窗口的链路自检 —— 上行编解码往返、v2 底盘字段往返、篡改包被拒、
  UDP socket 建立、发 50 包等遥测回包、坏包计数。
- **`main()` 主流程**：
  1. **X11 初始化**：开 display、建 960×540 窗口、检查 TrueColor 像素格式（决定能否直写视频）。
     **鼠标指针锁在窗口中心**（XInput2 原始位移或普通 MotionNotify），可以一直朝一个方向甩；
     `m` 键临时释放，`--no-mouse-lock` / `--raw-motion` 可调。
  2. **云台输入（鼠标主控 + 方向键叠加）**：鼠标位移 / `--mouse-window` → px/s × `--sens`（0.15）
     = yaw/pitch 角速度；方向键 `←/→`、`↑/↓` 以 `--gimbal-rate`（120 °/s）叠加。
     合成目标经一阶低通（`--tau`，45 ms）平滑、`--max-rate` 限幅；pitch 再乘 `--pitch-scale`（0.5），
     离 ±20° 软限位 5° 内线性降速（`--no-pitch-guard` 关）；`空格` HOLD 时目标为 0。
  3. **底盘 WASD**：`W/S` 前后、`A/D` 左右、`Q/E` 转向；速度目标再经**加速度限幅**
     （平移 3000 mm/s²、转向 6 rad/s²）平滑，防打滑/冲击；`HOLD` 或底盘急停时目标为 0。
     `Shift` 按一下切换**快速档**（平移 600 mm/s / 转向 3.0 rad/s，锁存），用 `XQueryKeymap` 上升沿检测。
  3b. **小陀螺（R 键）**：底盘按 `--spin-rate` 原地自旋，yaw 电机反向补偿让云台在世界系不转；
     补偿以指令值为基准、底盘陀螺实测值只作修正（同号且 ≥25% 才采信），30 ms 低通，关掉后 0.6 s 内继续补偿。
     `Q/E` 让位给自旋，WASD 平移照常；按空格 HOLD 时自旋停但补偿继续。
  4. **组包发送**：填 `ControlCommand`（yaw/pitch 角速度、云台 flags、底盘 vx/vy/wz 与
     chassis flags、一次性 `Home/Zero/EStop/Clear`），`udp.send()`；发送频率 `--hz`（默认 100 Hz）。
     Enable/Disable 二选一必带其一（显式失能，避免上电空档问题）。
  5. **收遥测**：`ControlTelemetry` 解析后更新丢包统计（`echo_seq` 跳变）、RTT、状态。
  5b. **收自瞄数据**：`AimView` 后台线程收 27RM 的 UDP 40002（48 B @ 50 Hz，`aim_udp.hpp` 结构体直接复用），
      状态栏显示 valid / 瞄准角 / 距离 / 转速；`t` 键开"辅助"按 aim 角比例把云台怼过去；
      `C`/`U` 走 `gimbal_cmd` 命令口 40003 切换 27RM 的敌方颜色 / 单板↔反小陀螺。
  6. **渲染**（约 30 fps，独立于控制节拍）：视频帧居中 `XPutImage`（`img->data=nullptr`
     防止 XDestroyImage 释放缓存），再叠状态文字：云台指令/实际/输出角、电机转速/电压/温度、
     status 异常标志（急停/堵转/看门狗/电机失联）、链路 Rx/RTT/丢包、底盘位姿/四轮转速/陀螺 yaw、
     视频 fps。
  7. **定频**：`sleep_until` 对齐目标周期（`t_next` 累加，落后则重置）。
  8. **收尾**：退出前连发 3 帧 `Hold` 停止指令，避免云台保持最后的速率。
- **按键**：**移动鼠标=转云台**（主控）、`←/→`/`↑/↓` 云台备用方向键、**按住鼠标左键=连发**（`F` 备用；
  三重门控：指针在窗口内 + 窗口有焦点 + 上一拍有焦点）、空格=HOLD（全部停）、`g`=云台使能/失能、
  `j`=底盘使能/失能（急停后按它恢复）、`x`=急停、`c`=底盘里程计清零、`h`=云台回中、`z`=当前位置归零、
  `k`=清空速率、`Shift`=快速/普通档切换、`R`=小陀螺模式开关、`m`=临时开关鼠标控制、
  `t`=自瞄辅助开关、`C`=切敌方颜色、`U`=切 27RM 单板/反小陀螺、`v`=视频开关、`Esc`=退出。
- **线协议**：不在本工程里定义，**直接复用** `../gimbal/include/gimbal/link/control_link.hpp`
  （上行 24 B / 下行 64 B，小端，显式字节偏移，两端逐字节一致；v1 的 16/32 B 旧帧仍可解码）。

### `mjpeg_view.cpp` —— MJPEG 视频实现（267 行）

对应声明见 [`../include/README.md`](../include/README.md)（`MjpegView` 类）。

- **连接**：`connectTcp()` 用 `getaddrinfo` 遍历地址族建 TCP，`TCP_NODELAY` 关 Nagle；
  发 `GET /stream HTTP/1.0` 请求；断线自动重连（20×50ms 退避）。
- **收流线程 `rxLoop()`**：独立线程 `recv()` 进缓冲，先跳过 HTTP 响应头（找 `\r\n\r\n`），
  再按 JPEG 的 **SOI(`FFD8`) … EOI(`FFD9`)** 从字节流里切出完整帧
  （multipart/x-mixed-replace 的 boundary 不必解析，靠 SOI/EOI 更稳），
  缓冲超过 4 MB 异常保护清空。
- **解码 `decodeJpeg()`**：libjpeg 解码为 RGB；**替换默认错误处理为 `setjmp/longjmp`**
  （libjpeg 默认 error_exit 会 `exit()` 整个进程）；限制尺寸 ≤ 4096；
  逐行转成 X11 ZPixmap 需要的 **BGRX（32bpp 小端）**，写完放入 `latest_` 并递增 `seq_`。
- **帧率统计**：每秒窗口内统计解码帧数写入 `fps_`。
- **线程模型**：解码线程持锁更新 `latest_`；主线程 `getLatest()` 加锁拷贝最新帧，
  按 `seq` 判断是否有新帧，**渲染永不阻塞控制循环**。

## 构建关系

`CMakeLists.txt` 编译 `src/main.cpp` + `src/aim_view.cpp` + `src/mjpeg_view.cpp`，头文件目录加
`include/`、`${GIMBAL_ROOT}/include`（复用 gimbal 线协议）与 `AIM_ROOT/inc`（复用 27RM 的 `aim_udp.hpp`，
默认按 `../HLJU_RMIC_autoaim` → `../stmproject/27RM_AUTO_AIM` → `../autoaim` 顺序自动查找）；
链接 X11（含 Xi）+ libjpeg。

### `aim_view.cpp` —— 自瞄数据接收（152 行）

对应声明见 [`../include/README.md`](../include/README.md)（`AimView` 类）。

- 后台线程 `bind(0.0.0.0:40002)` 收 27RM 广播的 48 B 小端报文（magic `0x524D3732` "27RM"），
  校验 magic、统计频率/丢包，解析成 `ctl::AimState`（瞄准角 / 距离 / 机器人中心 / 环半径 / 转速 / valid / flags / plate_slot）；
- 报文结构体**不复制**：直接 `#include "aim_udp.hpp"`（AIM_ROOT 找不到时编译期退化，`CTL_HAVE_AIM_LINK=0`，编译照常通过）。
