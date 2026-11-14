// ===========================================================================
//  gimbal_ctl —— 云台/底盘控制客户端（PC 端，原生 X11）
//
//  职责: 方向键/WASD -> 换算指令 -> UDP 发给 Jetson -> 显示遥测 + 视频
//
//  云台控制（两种输入叠加）:
//    · 鼠标（主控，FPS 式）：移动鼠标 = 给云台一个角速度，指针被锁在窗口
//      中心，所以可以一直朝一个方向甩。灵敏度 --sens（默认 0.30 °/s 每 px/s，
//      即鼠标 400 px/s ≈ 云台 120°/s）。
//    · 方向键（备用）：按住 ←/→ = yaw 左/右，↑/↓ = pitch 抬头/低头，速率
//      --gimbal-rate（默认 120°/s）。
//    两者相加后经一阶低通 --tau（默认 45ms）平滑；松手/停鼠标 -> 平滑归零。
//    m 键 = 临时开关鼠标控制（关掉顺手能把指针拿出去点别的窗口）。
//
//  用法:
//      gimbal_ctl --host 192.168.2.2                 方向键控制 + 开视频
//      gimbal_ctl --host 192.168.2.2 --no-video       只要控制，不要视频
//      gimbal_ctl --host 192.168.2.2 --gimbal-rate 150  方向键更快
//      gimbal_ctl --selftest                          只测 UDP 收发，不开窗口
//
//  按键:
//      ↑/↓ = 云台俯仰        ←/→ = 云台左右       空格 = 停手/继续
//      W/S = 底盘前进/后退    A/D = 底盘左移/右移    Q/E = 底盘左转/右转
//      g = 云台使能开关       j = 底盘使能开关       x = 急停（云台+底盘）
//      c = 底盘里程计清零     h = 云台回中           z = 云台当前位置归零
//      k = 清空当前速率       v = 视频开关           F = 按住发射（水弹波箱）
//      R = 切换小陀螺模式（再按一下退出）
//      按一下 Shift（左/右均可）= 切换底盘「快速/普通」模式（锁存，不用一直按）：
//          普通档：平移 300 mm/s、转向 1.5 rad/s
//          快速档：平移 600 mm/s、转向 3.0 rad/s
//          再按一下 Shift 切回普通档；松开 Shift 不会退回，模式一直保持。
//          加速度斜坡仍是 3000 mm/s²（0->600 约 0.2 s），切换不会窜。
//          数值可用 --shift-speed / --shift-turn 改。
//
//  小陀螺模式（R 键切换，参考 RM 正赛）:
//      底盘按 --spin-rate（默认 2.0 rad/s）原地自旋，yaw 电机反向补偿，
//      让云台/相机在世界坐标系里保持不转；方向键照常控制云台（叠加在补偿上）。
//      补偿量优先用底盘陀螺实测角速度（chassis_gyro_wz_mradps），
//      这样地面打滑、掉速都能自动跟上；遥测断流或底盘 IMU 离线时退回指令值。
//      --spin-rate 取负值 = 反向自旋；--spin-gain -1 = 补偿方向反了时用。
// ===========================================================================
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <X11/extensions/XInput2.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gimbal/link/chassis_link.hpp"   // chassis_status 状态位（判断底盘 IMU 是否在线）
#include "gimbal/link/control_link.hpp"
#include "aim_view.hpp"
#include "mjpeg_view.hpp"

using namespace gimbal;
using namespace ctl;

// ---------------------------------------------------------------------------
static double nowS() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

/// 物理键是否按下（用 XQueryKeymap，天然免疫 X11 自动重复的 Press/Release 抖动）
static bool keyDown(Display* dpy, KeySym ks) {
    if (!dpy) return false;
    char keys[32];
    XQueryKeymap(dpy, keys);
    const KeyCode kc = XKeysymToKeycode(dpy, ks);
    if (kc == 0) return false;
    return (keys[kc >> 3] & (1 << (kc & 7))) != 0;
}

struct Options {
    std::string host      = "192.168.2.2";
    int         ctrl_port = CTRL_DEFAULT_PORT;
    int         http_port = 8080;
    bool        video     = true;
    std::string mode      = "relative";    ///< relative | off（旧的 grab/warp/auto 视作 relative）
    double      sens      = 0.15;          ///< °/s per (px/s)：800 px/s ≈ 120°/s（原 0.30 砍半）
    double      mouse_window = 0.03;       ///< 估 px/s 的滑动窗口 s（越小越跟手）
    bool        mouse_lock = true;         ///< 指针锁在窗口中心（--no-mouse-lock 关）
    bool        mouse_debug = false;       ///< 打印鼠标原始采样（--mouse-debug）
    bool        use_raw_motion = false;    ///< 用 XInput2 原始位移（--raw-motion）
    bool        ignore_focus  = false;     ///< 不看输入焦点（--mouse-ignore-focus）
    bool        start_locked  = false;     ///< 启动即锁死（--start-locked，按空格解除）
    bool        no_send      = false;      ///< 只收不发（--no-send）：27RM --gimbal-aim 自己控云台时用
    bool        no_pitch_guard = false;    ///< 关掉客户端的 pitch 软限位保护（--no-pitch-guard）
    bool        no_reticle    = false;     ///< 不画准星（--no-reticle）
    int         reticle_size  = 18;        ///< 准星臂长（像素，--reticle-size）
    int         reticle_dx    = 0;         ///< 准星水平偏移（像素，+右）
    int         reticle_dy    = 0;         ///< 准星垂直偏移（像素，+下）
    double      tau       = 0.045;         ///< 平滑时间常数 s（30→45ms：起停更顺）
    double      max_rate  = 180.0;         ///< 角速度上限 °/s
    double      gimbal_rate = 120.0;       ///< 方向键控制云台的速率 °/s
    double      pitch_scale = 0.5;         ///< pitch 灵敏度倍率（相对 yaw，默认半速）
    double      pitch_min   = -20.0;       ///< pitch 软限位下限（与 gimbal --pitch-limit 对齐）
    double      pitch_max   =  20.0;       ///< pitch 软限位上限
    double      pitch_margin = 5.0;        ///< 距限位多少度开始线性减速（°）
    double      pitch_max_rate = 120.0;    ///< pitch 指令上限（对齐 gimbal --pitch-rate 默认 120）
    double      deadzone  = 0.5;           ///< px/s 死区
    bool        invert_y  = false;         ///< 鼠标上下反向（默认下移=低头）
    int         win_w     = 960;
    int         win_h     = 540;
    double      hz        = 100.0;
    bool        selftest  = false;

    // ---- 底盘（USART2 链路，坐标：vx 前进 / vy 左移 / wz 逆时针）----
    double      chassis_speed = 300.0;    ///< mm/s，WASD 满速
    double      chassis_turn  = 1500.0;   ///< mrad/s = 1.5 rad/s，←/→ 满速
    double      chassis_speed_fast = 600.0;   ///< mm/s，快速档（Shift 切换）的平移满速
    double      chassis_turn_fast  = 3000.0;  ///< mrad/s = 3.0 rad/s，快速档的转向
    bool        chassis_en    = true;     ///< j 键切换

    // ---- 27RM 自瞄广播（UDP 40002，48B @ 50Hz）----
    bool        aim_off       = false;   ///< 不接自瞄数据
    int         aim_port      = ctl::kAimDefaultPort;
    int         rm27_cmd_port = 40003;   ///< 给 27RM 发命令的 UDP 口（U 键切模式）
    double      aim_kp        = 4.0;     ///< 辅助增益总开关（同时设 yaw/pitch）
    double      aim_kp_yaw    = 4.0;     ///< yaw 增益：°/s per ° 误差（负值=方向反）
    double      aim_kp_pitch  = 4.0;     ///< pitch 增益（负值=方向反）
    double      aim_deadband  = 0.3;     ///< 死区（°），小于它不动
    double      aim_bias_yaw  = 0.0;     ///< 瞄偏右多少度（正值=瞄目标右侧）
    double      aim_bias_pitch = 0.0;    ///< 瞄偏下多少度（正值=瞄目标下方，水弹下坠补偿也用这个）
    double      aim_stale     = 0.25;    ///< 超过这么久没新包就不信任（s）
    bool        aim_assist_on = false;   ///< 启动就开自瞄辅助（--aim-assist）

    // ---- 小陀螺（R 键切换）：底盘原地自旋 + yaw 反向补偿让云台不转 ----
    double      spin_rate_mradps = 2000.0;  ///< 0.001 rad/s，+ 逆时针；负值 = 顺时针
    double      spin_gain        = 1.0;     ///< yaw 反向补偿倍率（方向反了填 -1）
};

static void usage(const char* a0) {
    std::printf(
        "用法: %s [选项]\n"
        "  --host <ip|名字>     Jetson 地址 (默认 192.168.2.2)\n"
        "  --ctrl-port <n>      UDP 控制口 (默认 %u)\n"
        "  --http-port <n>      MJPEG 端口 (默认 8080)\n"
        "  --no-video           不显示视频\n"
        "  发射: 按住鼠标左键（指针在窗口内且窗口有焦点才响应，HOLD 时不响应）；F 键备用\n"
        "  自瞄: 按住鼠标右键 = 临时锁定最近装甲板（此时鼠标不控云台，松手交还）\n"
        "  --mode <模式>        relative（鼠标控云台，默认）| off（只用方向键）\n"
        "  --sens <f>           鼠标灵敏度 °/s per px/s (默认 0.15)\n"
        "  --mouse-window <s>   估鼠标速度的滑窗 (默认 0.03，越小越跟手、越抖)\n"
        "  --no-mouse-lock      不把指针锁回窗口中心（调试用）\n"
        "  --start-locked       启动就锁死（云台+底盘失能），按空格解除\n"
        "  --no-send            只收不发（观战）：27RM 用 --gimbal-aim 控云台时用这个，\n"
        "                       否则两边同时发 40001 会抢指令\n"
        "  --no-reticle         不画屏幕中央的准星\n"
        "  --reticle-size <px>  准星臂长 (默认 18)\n"
        "  --mouse-debug        打印鼠标原始采样（调试用）\n"
        "  --mouse-ignore-focus 鼠标只要求指针在窗口内，不看输入焦点（部分 WM 下\n"
        "                       拿不到焦点时用；默认要看焦点，防误控）\n"
        "  --raw-motion         用 XInput2 原始位移（原生 Xorg 可用；XWayland 下会被\n"
        "                       自己 warp 的位移污染，默认关）\n"
        "  --tau <s>            平滑时间常数 (默认 0.045)\n"
        "  --max-rate <dps>     角速度上限 (默认 180)\n"
        "  --gimbal-rate <dps>  方向键云台速率 (默认 120)\n"
        "  --pitch-scale <f>    pitch 灵敏度倍率，相对 yaw (默认 0.5)\n"
        "  --pitch-limit <a> <b> pitch 软限位 (默认 -20 20，与 gimbal 对齐)\n"
        "  --pitch-margin <deg> 距限位多少度开始减速 (默认 5)\n"
        "  --no-pitch-guard     关掉客户端 pitch 软限位保护（只靠 gimbal 自己的限位）\n"
        "  --pitch-max-rate <dps> pitch 指令上限 (默认 120，对齐 gimbal)\n"
        "  --invert-y           鼠标上下反向（默认鼠标下移 = 低头）\n"
        "  --size <w> <h>       窗口尺寸 (默认 960 540)\n"
        "  --hz <n>             发送频率 (默认 100)\n"
        "  --chassis-speed <mm/s> 底盘平移满速 (默认 300)\n"
        "  --chassis-turn <mrad/s> 底盘转向满速 (默认 1500)\n"
        "  --shift-speed <mm/s> 快速档平移满速（Shift 切换，默认 600）\n"
        "  --shift-turn <mrad/s> 快速档转向满速（Shift 切换，默认 3000）\n"
        "  --spin-rate <mrad/s> 小陀螺自旋速率（+逆时针，默认 2000 = 2.0 rad/s）\n"
        "  --spin-gain <f>      小陀螺 yaw 补偿倍率（默认 1.0，方向反了填 -1）\n"
        "  --aim-port <n>       27RM 自瞄广播端口 (默认 40002)\n"
        "  --27rm-cmd-port <n>  给 27RM 发命令的口 (默认 40003，U/C 键用)\n"
        "  --aim-off            不接自瞄数据\n"
        "  --aim-kp <f>         自瞄辅助增益，同时设两轴 (默认 4.0)\n"
        "  --aim-kp-yaw <f>     只改 yaw 增益（负值 = 方向反）\n"
        "  --aim-kp-pitch <f>   只改 pitch 增益（负值 = 方向反）\n"
        "  --aim-deadband <deg> 自瞄死区 (默认 0.3)\n"
        "  --aim-bias-yaw <deg>   自瞄瞄偏右多少度（+ = 瞄目标右侧）\n"
        "  --aim-bias-pitch <deg> 自瞄瞄偏下多少度（+ = 瞄目标下方，可补弹道下坠）\n"
        "  --reticle-dx <px>      准星水平偏移（+右）\n"
        "  --reticle-dy <px>      准星垂直偏移（+下）\n"
        "  --aim-stale <s>      自瞄数据过期时间 (默认 0.25)\n"
        "  --aim-assist         启动就开自瞄辅助（等价于进去按 t）\n"
        "  --selftest           只测 UDP 收发，不开窗口\n"
        "  -h, --help\n", a0, CTRL_DEFAULT_PORT);
}

// ===========================================================================
// UDP 客户端
// ===========================================================================
class UdpClient {
public:
    bool open(const std::string& host, int port) {
        struct addrinfo hints;
        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family   = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        struct addrinfo* res = nullptr;
        if (::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0)
            return false;
        fd_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (fd_ < 0) { ::freeaddrinfo(res); return false; }
        if (::connect(fd_, res->ai_addr, res->ai_addrlen) != 0) {
            ::freeaddrinfo(res);
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        ::freeaddrinfo(res);
        return true;
    }
    ~UdpClient() { if (fd_ >= 0) ::close(fd_); }
    bool ok() const { return fd_ >= 0; }

    void send(const ControlCommand& c) {
        if (fd_ < 0) return;
        const auto b = c.encode();
        ::send(fd_, b.data(), b.size(), 0);
        sent_at_[c.seq % kRing] = nowS();
        if (c.seq % kRing == 0) { /* 环形复用，够用 */ }
    }

    /// 非阻塞收遥测
    bool recv(ControlTelemetry& t) {
        if (fd_ < 0) return false;
        bool got = false;
        for (;;) {
            uint8_t buf[512];
            const ssize_t n = ::recv(fd_, buf, sizeof(buf), MSG_DONTWAIT);
            if (n <= 0) break;
            ControlTelemetry u;
            if (!ControlTelemetry::decode(buf, static_cast<size_t>(n), u)) {
                ++bad_;
                continue;
            }
            const double t0 = sent_at_[u.echo_seq % kRing];
            if (t0 > 0) {
                rtt_ms_ = (nowS() - t0) * 1000.0;
            }
            t   = u;
            got = true;
        }
        return got;
    }

    uint64_t bad() const { return bad_; }
    double   rttMs() const { return rtt_ms_; }

private:
    static constexpr size_t kRing = 256;
    int      fd_ = -1;
    double   sent_at_[kRing] = {0};
    uint64_t bad_  = 0;
    double   rtt_ms_ = -1.0;
};

// ===========================================================================
// 给 27RM 发命令（UDP 命令口，默认 40003）：'u' = 切换 单板锁定 / 反小陀螺
//   27RM 用 --no-gui 后台跑时没终端、按不了 u，所以走网络
// ===========================================================================
class UdpCmd {
public:
    ~UdpCmd() { close(); }
    bool open(const std::string& host, int port) {
        close();
        struct addrinfo hints;
        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family   = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        struct addrinfo* res = nullptr;
        if (::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0)
            return false;
        fd_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (fd_ < 0) { ::freeaddrinfo(res); return false; }
        if (::connect(fd_, res->ai_addr, res->ai_addrlen) != 0) {
            ::freeaddrinfo(res);
            close();
            return false;
        }
        ::freeaddrinfo(res);
        return true;
    }
    void close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
    bool ok() const { return fd_ >= 0; }
    bool send(char c) { return fd_ >= 0 && ::send(fd_, &c, 1, 0) == 1; }
private:
    int fd_ = -1;
};

// ===========================================================================
// 自检（不开窗口，只验 UDP 收发 + 协议）
// ===========================================================================
static int runSelftest(const Options& o) {
    std::printf("=== gimbal_ctl 自检 ===\n");
    int fail = 0;
    auto CHECK = [&](bool ok, const char* m) {
        std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", m);
        if (!ok) ++fail;
    };

    ControlCommand c;
    c.seq = 42; c.yaw_dps = 45.0; c.pitch_dps = -30.0; c.flags = ctrl_flag::Enable;
    const auto b = c.encode();
    ControlCommand d;
    CHECK(ControlCommand::decode(b.data(), b.size(), d), "上行编解码");
    CHECK(std::fabs(d.yaw_dps - 45.0) < 1e-9 && std::fabs(d.pitch_dps + 30.0) < 1e-9,
          "角速度往返");

    // v2：底盘字段（坐标 vx 前进 / vy 左移 / wz 逆时针）
    ControlCommand cs;
    cs.seq = 9;
    cs.chassis_vx_mmps   = 500.0;
    cs.chassis_vy_mmps   = -250.0;
    cs.chassis_wz_mradps = 1200.0;
    cs.chassis_flags     = chs_flag::Enable;
    const auto cb = cs.encode();
    CHECK(cb.size() == CTRL_UP_LEN, "上行包长 24 字节（v2）");
    ControlCommand cd;
    CHECK(ControlCommand::decode(cb.data(), cb.size(), cd), "上行 v2 解码");
    CHECK(cd.chassis_vx_mmps == 500.0 && cd.chassis_vy_mmps == -250.0 &&
              cd.chassis_wz_mradps == 1200.0 && cd.chassisEnable(),
          "底盘字段往返一致");
    auto cbad = cb;
    cbad[16] ^= 0xFF;
    CHECK(!ControlCommand::decode(cbad.data(), cbad.size(), cd), "底盘字段篡改被拒");

    UdpClient u;
    const bool opened = u.open(o.host, o.ctrl_port);
    CHECK(opened, ("UDP socket 已建立 -> " + o.host + ":" +
                   std::to_string(o.ctrl_port)).c_str());
    if (!opened) return 1;

    std::printf("  发 50 个包并等 1 秒看回包…\n");
    for (uint32_t i = 1; i <= 50; ++i) {
        ControlCommand s;
        s.seq = i; s.yaw_dps = 0; s.pitch_dps = 0; s.flags = 0;
        u.send(s);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ControlTelemetry t;
    bool got = false;
    const double t0 = nowS();
    while (nowS() - t0 < 1.0 && !got) {
        got = u.recv(t);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (got) {
        std::printf("  收到遥测: echo_seq=%u yaw=%+.2f°/s pitch=%+.2f°/s "
                    "角度 %+.2f/%+.2f° 母线 %dmV %d°C status=0x%02X\n",
                    t.echo_seq, t.yaw_dps, t.pitch_dps, t.yaw_deg, t.pitch_deg,
                    t.vbus_mv, t.temp_c, t.status);
    }
    CHECK(got, "收到 Jetson 遥测回包");
    CHECK(u.bad() == 0, "无坏包");
    std::printf("\n%s（失败 %d 项）\n", fail ? "有失败" : "全部通过", fail);
    return fail ? 1 : 0;
}

// ===========================================================================
// 主程序
// ===========================================================================
int main(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next  = [&](double def) { return (i + 1 < argc) ? std::atof(argv[++i]) : def; };
        auto nexti = [&](int def)    { return (i + 1 < argc) ? std::atoi(argv[++i]) : def; };
        if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else if (a == "--host" && i + 1 < argc) o.host = argv[++i];
        else if (a == "--ctrl-port") o.ctrl_port = nexti(o.ctrl_port);
        else if (a == "--http-port") o.http_port = nexti(o.http_port);
        else if (a == "--no-video")  o.video = false;
        else if (a == "--mode" && i + 1 < argc) o.mode = argv[++i];
        else if (a == "--sens")      o.sens = next(o.sens);
        else if (a == "--mouse-window") o.mouse_window = next(o.mouse_window);
        else if (a == "--no-mouse-lock") o.mouse_lock = false;
        else if (a == "--mouse-debug")   o.mouse_debug = true;
        else if (a == "--raw-motion")    o.use_raw_motion = true;
        else if (a == "--mouse-ignore-focus") o.ignore_focus = true;
        else if (a == "--no-reticle")    o.no_reticle = true;
        else if (a == "--start-locked")  o.start_locked = true;
        else if (a == "--no-send")       o.no_send = true;
        else if (a == "--reticle-size")  o.reticle_size = nexti(o.reticle_size);
        else if (a == "--tau")       o.tau = next(o.tau);
        else if (a == "--max-rate")  o.max_rate = next(o.max_rate);
        else if (a == "--gimbal-rate") o.gimbal_rate = next(o.gimbal_rate);
        else if (a == "--invert-y")  o.invert_y = true;
        else if (a == "--pitch-scale") o.pitch_scale = next(o.pitch_scale);
        else if (a == "--pitch-limit") { o.pitch_min = next(o.pitch_min); o.pitch_max = next(o.pitch_max); }
        else if (a == "--pitch-margin") o.pitch_margin = next(o.pitch_margin);
        else if (a == "--no-pitch-guard") o.no_pitch_guard = true;
        else if (a == "--pitch-max-rate") o.pitch_max_rate = next(o.pitch_max_rate);
        else if (a == "--chassis-speed") o.chassis_speed = next(o.chassis_speed);
        else if (a == "--chassis-turn")  o.chassis_turn = next(o.chassis_turn);
        else if (a == "--shift-speed")   o.chassis_speed_fast = next(o.chassis_speed_fast);
        else if (a == "--shift-turn")    o.chassis_turn_fast = next(o.chassis_turn_fast);
        else if (a == "--spin-rate")     o.spin_rate_mradps = next(o.spin_rate_mradps);
        else if (a == "--spin-gain")     o.spin_gain = next(o.spin_gain);
        else if (a == "--aim-port")      o.aim_port = nexti(o.aim_port);
        else if (a == "--27rm-cmd-port") o.rm27_cmd_port = nexti(o.rm27_cmd_port);
        else if (a == "--aim-off")       o.aim_off = true;
        else if (a == "--aim-kp") {
            o.aim_kp = next(o.aim_kp);
            o.aim_kp_yaw = o.aim_kp_pitch = o.aim_kp;   // 一次设两轴
        }
        else if (a == "--aim-kp-yaw")    o.aim_kp_yaw = next(o.aim_kp_yaw);
        else if (a == "--aim-kp-pitch")  o.aim_kp_pitch = next(o.aim_kp_pitch);
        else if (a == "--aim-deadband")  o.aim_deadband = next(o.aim_deadband);
        else if (a == "--aim-bias-yaw")  o.aim_bias_yaw = next(o.aim_bias_yaw);
        else if (a == "--aim-bias-pitch") o.aim_bias_pitch = next(o.aim_bias_pitch);
        else if (a == "--reticle-dx")    o.reticle_dx = nexti(o.reticle_dx);
        else if (a == "--reticle-dy")    o.reticle_dy = nexti(o.reticle_dy);
        else if (a == "--aim-stale")     o.aim_stale = next(o.aim_stale);
        else if (a == "--aim-assist")    o.aim_assist_on = true;
        else if (a == "--size")      { o.win_w = nexti(o.win_w); o.win_h = nexti(o.win_h); }
        else if (a == "--hz")        o.hz = next(o.hz);
        else if (a == "--selftest")  o.selftest = true;
        else { std::printf("未知参数: %s\n", a.c_str()); usage(argv[0]); return 2; }
    }

    if (o.selftest) return runSelftest(o);

    // ---- UDP ----
    UdpClient udp;
    if (!udp.open(o.host, o.ctrl_port)) {
        std::printf("无法建立到 %s:%d 的 UDP 通道\n", o.host.c_str(), o.ctrl_port);
        return 1;
    }

    // ---- 给 27RM 的命令口（U 键切模式）----
    UdpCmd rm27_cmd;
    if (rm27_cmd.open(o.host, o.rm27_cmd_port)) {
        std::printf("[mode] 27RM 命令口 %s:%d 就绪（U 键切 单板/反小陀螺）\n",
                    o.host.c_str(), o.rm27_cmd_port);
    } else {
        std::printf("[mode] 27RM 命令口 %s:%d 打不开，U 键不可用\n",
                    o.host.c_str(), o.rm27_cmd_port);
    }

    // ---- X11 ----
    Display* dpy = XOpenDisplay(nullptr);
    if (!dpy) {
        std::printf("打不开 X display（DISPLAY=%s）\n",
                    getenv("DISPLAY") ? getenv("DISPLAY") : "(空)");
        std::printf("当前会话是 Wayland 的话，本程序走 XWayland；"
                    "若仍失败请确认 XWayland 可用\n");
        return 1;
    }
    const int scr = DefaultScreen(dpy);
    const auto vis = DefaultVisual(dpy, scr);
    const int depth = DefaultDepth(dpy, scr);

    // 检查像素格式（视频直写 ZPixmap 需要常见 TrueColor）
    const bool truecolor = (vis->c_class == TrueColor) && depth >= 24 &&
                           vis->red_mask == 0xFF0000 &&
                           vis->green_mask == 0x00FF00 &&
                           vis->blue_mask == 0x0000FF;

    std::printf("[x11] depth=%d class=%s masks R=%#lx G=%#lx B=%#lx -> %s\n",
                depth,
                vis->c_class == TrueColor ? "TrueColor" : "非TrueColor",
                vis->red_mask, vis->green_mask, vis->blue_mask,
                truecolor ? "支持直写视频" : "不支持直写视频(只显示状态文字)");

    Window win = XCreateSimpleWindow(dpy, RootWindow(dpy, scr), 0, 0,
                                     static_cast<unsigned>(o.win_w),
                                     static_cast<unsigned>(o.win_h), 0,
                                     BlackPixel(dpy, scr), BlackPixel(dpy, scr));
    XStoreName(dpy, win, "gimbal_ctl");
    XSelectInput(dpy, win, ExposureMask | KeyPressMask | ButtonPressMask | ButtonReleaseMask |
                               PointerMotionMask | StructureNotifyMask |
                               FocusChangeMask);
    XMapWindow(dpy, win);

    Atom wm_delete = XInternAtom(dpy, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(dpy, win, &wm_delete, 1);

    XGCValues gcv;
    GC gc = XCreateGC(dpy, win, 0, &gcv);
    // 后备缓冲（双缓冲）：所有绘制先画到 Pixmap，最后一次性 XCopyArea 贴到窗口。
    // 不这么做的话每帧 XClearWindow→画图之间会露黑底，30fps 下就是"一闪一闪"。
    Pixmap back = XCreatePixmap(dpy, win, static_cast<unsigned>(o.win_w),
                                static_cast<unsigned>(o.win_h),
                                static_cast<unsigned>(depth));
    XSetForeground(dpy, gc, WhitePixel(dpy, scr));

    XFontStruct* font = XLoadQueryFont(dpy, "fixed");
    if (font) XSetFont(dpy, gc, font->fid);

    // ---- 鼠标控制：隐形光标（锁指针时看不见箭头，免得挡视野）----
    Cursor blank_cursor = None;
    {
        static char blank_bits[1] = {0};      // 1x1 全透明位图
        Pixmap pm = XCreateBitmapFromData(dpy, win, blank_bits, 1, 1);
        XColor dummy{};
        dummy.flags = DoRed | DoGreen | DoBlue;
        blank_cursor = XCreatePixmapCursor(dpy, pm, pm, &dummy, &dummy, 0, 0);
        XFreePixmap(dpy, pm);
    }
    // ---- 鼠标原始位移（XInput2）：FPS 式控制的正路 ----
    bool raw_motion = false;                  // XInput2 原始相对位移是否可用
    int xi_opcode = -1, xi_event = -1, xi_error = -1;
    if (o.use_raw_motion &&
        XQueryExtension(dpy, "XInputExtension", &xi_opcode, &xi_event, &xi_error)) {
        int major = 2, minor = 1;
        if (XIQueryVersion(dpy, &major, &minor) == Success) {
            unsigned char xi_mask[XIMaskLen(XI_LASTEVENT)] = {0};
            XISetMask(xi_mask, XI_RawMotion);
            XIEventMask ev_mask;
            ev_mask.deviceid = XIAllMasterDevices;
            ev_mask.mask_len = sizeof(xi_mask);
            ev_mask.mask     = xi_mask;
            if (XISelectEvents(dpy, DefaultRootWindow(dpy), &ev_mask, 1) == Success)
                raw_motion = true;
        }
    }
    std::printf("[mouse] 采样方式: %s\n",
                raw_motion ? "XInput2 原始位移（--raw-motion）"
                           : "指针位置差分 + 锁中心（默认，XWayland 下最稳）");

    bool mouse_on = (o.mode != "off");        // 旧的 grab/warp/auto 一律按 relative
    const double mouse_arm_init = nowS() + 0.25;   // 见下：启动静默期
    if (mouse_on) {
        if (o.mouse_lock && blank_cursor != None) XDefineCursor(dpy, win, blank_cursor);
        XWarpPointer(dpy, None, win, 0, 0, 0, 0, o.win_w / 2, o.win_h / 2);
    }

    // ---- 视频 ----
    MjpegView  view;
    uint64_t   shown_seq = 0;
    bool       video_on  = o.video;
    if (video_on) {
        view.start(o.host, o.http_port);
        std::printf("[video] 拉流 http://%s:%d/stream\n", o.host.c_str(), o.http_port);
    }

    // ---- 自瞄数据（27RM 广播到 UDP 40002）----
    AimView aim;
    bool    aim_ok = false;
    if (!o.aim_off) {
        aim_ok = aim.open(o.aim_port);
        if (aim_ok) {
            std::printf("[aim] 监听 UDP %d（27RM --aim-udp 广播；%s）\n", o.aim_port,
                        AimView::compiledIn() ? "48B/50Hz" : "本机没编进 aim_udp.hpp");
        } else {
            std::printf("[aim] 端口 %d 打开失败，自瞄数据不可用\n", o.aim_port);
        }
    }

    // ---- 状态 ----
    ControlTelemetry last_telem;
    bool    have_telem = false;
    double  rate[2]    = {0.0, 0.0};        // yaw, pitch（平滑后的 °/s）
    struct MouseSample { double t; double dx; double dy; };
    std::deque<MouseSample> mouse_hist;      // 最近 --mouse-window 内的指针位移
    bool    mouse_synced    = false;         // 位置差分兜底时的"首拍只同步"标志
    bool    return_to_win   = true;          // 指针在窗口内 = 可以把它拉回中心
    double  raw_acc_x = 0.0, raw_acc_y = 0.0; // XI_RawMotion 本拍累计位移
    double  raw_last_t = -1e9;               // 最后一次收到 raw 事件的时刻
    double  mouse_arm_t = mouse_arm_init;    // 静默到期时刻（启动/切回后先别动云台）
    double  mouse_px[2] = {0.0, 0.0};        // 上一拍估出来的 px/s（给状态栏显示）
    uint32_t seq       = 0;
    bool    hold_mode  = o.start_locked;
    bool    locked     = o.start_locked;   // 空格：锁死（云台失能 + 底盘拉低 STBY）
    bool    en_state   = true;
    uint8_t oneshot    = 0;                 // 本拍要带的一次性 flags
    bool    fire_hold_key = false;          // 本拍发射请求
    bool    fire_btn      = false;          // 鼠标左键（事件置位）
    bool    prev_focused  = false;          // 上一拍是否有焦点（防"聚焦点击"走火）
    int     fire_btn_up_ticks = 0;          // 连续多少拍"状态位说松开"（兜底清位用）
    bool    aim_btn       = false;          // 鼠标右键（事件置位）= 按住临时自瞄
    int     aim_btn_up_ticks = 0;

    // ---- 底盘状态（WASD 平滑后下发）----
    bool     chassis_en        = o.chassis_en;
    bool     chassis_estop     = false;
    bool     chassis_fast      = false;   // 快速档开关：按一下 Shift 切换
    bool     shift_prev        = false;   // 上一拍的 Shift 物理状态（上升沿检测用）
    bool     spin_mode         = false;   // 小陀螺：R 键切换
    double   spin_wz_lp        = 0.0;     // 实测自旋角速度的低通值 (mrad/s)
    double   spin_comp_until   = 0.0;     // 关掉小陀螺后继续补偿到这一刻（吃掉减速段）
    double   t_last_telem      = -1e9;    // 上次收到遥测的时刻（算新鲜度）
    bool     oneshot_chassis_clear = false;
    double   cvx = 0.0, cvy = 0.0, cwz = 0.0;   // 平滑后的 mm/s 与 mrad/s
    bool     aim_assist  = o.aim_assist_on;  // t 键：自瞄辅助（把云台往目标怼）
    AimState aim_state;             // 最新一帧自瞄
    bool     aim_have    = false;   // 有没有（含过期判断）
    double   aim_dps[2]  = {0.0, 0.0};  // 自瞄辅助给出的 yaw/pitch 指令
    double  lost_pkts = 0;
    uint32_t last_echo = 0;

    std::printf("========================================================\n");
    std::printf("  gimbal_ctl 云台/底盘控制\n");
    std::printf("  目标    : %s  UDP:%d  MJPEG:%d\n", o.host.c_str(), o.ctrl_port,
                o.http_port);
    std::printf("  云台    : 鼠标灵敏度 %.2f°/s per px/s（%s）  方向键 %.0f°/s  平滑 tau=%.0fms\n",
                o.sens, o.mouse_lock ? "指针锁定" : "不锁指针", o.gimbal_rate, o.tau * 1000);
    std::printf("  pitch   : 灵敏度 x%.2f（yaw 的 %d%%）  上限 %.0f°/s  软限位 %+.0f/%+.0f 度，"
                "离限位 %.1f 度内线性减速\n",
                o.pitch_scale, static_cast<int>(o.pitch_scale * 100 + 0.5),
                std::min(o.max_rate, o.pitch_max_rate), o.pitch_min, o.pitch_max, o.pitch_margin);
    std::printf("  输入    : 移动鼠标 = 转云台（右=右转 下=低头）+ 方向键叠加  m=开关鼠标\n");
    if (aim_ok) {
        std::printf("  自瞄    : UDP %d 收 27RM 48B/50Hz  辅助 t 键（%s）  kp yaw %.1f / pitch %.1f  死区 %.2f°\n",
                    o.aim_port, o.aim_assist_on ? "启动即开" : "默认关",
                    o.aim_kp_yaw, o.aim_kp_pitch, o.aim_deadband);
    }
    std::printf("  按键    : ↑/↓=云台俯仰  ←/→=云台左右  空格=停手  g=云台使能  j=底盘使能\n");
    std::printf("            W/S=前后  A/D=左右  Q/E=底盘转向  F=按住发射  x=急停  h=回中\n");
    std::printf("            z=归零  k=清速率  c=切敌方蓝/红  n=底盘里程计清零  v=视频\n");
    std::printf("            t=自瞄辅助  U=切单板/反小陀螺  空格=锁死/解除  Esc=退出\n");
    std::printf("  自瞄    : **按住鼠标右键** = 临时锁定最近装甲板（松手立刻交还鼠标）；t 是常开开关\n");
    if (o.start_locked)
        std::printf("  锁死    : 启动即锁死（云台失能 + 底盘 STBY 拉低），按空格解除\n");
    if (o.no_send)
        std::printf("  模式    : **只收不发**（--no-send）：不占用 40001，云台交给 27RM --gimbal-aim\n");
    std::printf("  发射    : 按住鼠标左键（窗口内 + 有焦点才响应，HOLD 时不响应）；F 键备用\n");
    std::printf("            按一下 Shift = 切换快速/普通模式（锁存，不用一直按）\n");
    std::printf("            R = 切换小陀螺（底盘自旋 + 云台反向站住）\n");
    std::printf("  底盘    : 普通档 平移 %.0f mm/s  转向 %.0f mrad/s\n",
                o.chassis_speed, o.chassis_turn);
    std::printf("            快速档 平移 %.0f mm/s  转向 %.0f mrad/s\n",
                o.chassis_speed_fast, o.chassis_turn_fast);
    std::printf("  小陀螺  : 自旋 %+.2f rad/s（%s）  云台补偿倍率 %+.2f\n",
                o.spin_rate_mradps / 1000.0,
                o.spin_rate_mradps >= 0 ? "逆时针" : "顺时针", o.spin_gain);
    std::printf("            vx 前进 / vy 左移 / wz 逆时针\n");
    std::printf("========================================================\n");

    const double dt_target = 1.0 / o.hz;
    auto         t_next    = std::chrono::steady_clock::now();
    auto         t_render  = std::chrono::steady_clock::now();   // 上次渲染时刻
    bool         running   = true;
    bool         video_first_done = false;
    int          video_frames = 0;
    auto         t_vstat  = std::chrono::steady_clock::now();

    while (running) {
        // ---------- 1. X 事件 ----------
        while (XPending(dpy)) {
            XEvent ev;
            XNextEvent(dpy, &ev);
            switch (ev.type) {
                case KeyPress: {
                    const KeySym ks = XLookupKeysym(&ev.xkey, 0);
                    if (ks == XK_Escape) running = false;
                    else if (ks == XK_space) {
                        locked    = !locked;
                        hold_mode = locked;      // 锁死时同时拒收所有输入
                        if (!locked) {
                            oneshot |= ctrl_flag::Enable;   // 解除后重新授权云台
                            chassis_estop = false;
                            chassis_en    = true;
                        }
                        std::printf("[lock] %s\n", locked
                            ? "锁死：云台失能 + 底盘拉低 STBY（再按空格解除）"
                            : "解除锁死：云台重新使能、底盘恢复");
                    }
                    else if (ks == XK_c) {
                        if (rm27_cmd.send('c'))
                            std::printf("[color] 已通知 27RM 切换 蓝/红（看状态栏 aim 行）\n");
                        else
                            std::printf("[color] 27RM 命令口没开（--27rm-cmd-port %d）\n",
                                        o.rm27_cmd_port);
                    }
                    else if (ks == XK_u) {
                        if (rm27_cmd.send('u'))
                            std::printf("[mode] 已通知 27RM 切换模式（看状态栏 aim 行）\n");
                        else
                            std::printf("[mode] 27RM 命令口没开（--27rm-cmd-port %d）\n",
                                        o.rm27_cmd_port);
                    }
                    else if (ks == XK_g) { en_state = !en_state;
                                           oneshot |= ctrl_flag::Enable; }
                    else if (ks == XK_j) { chassis_en = !chassis_en;
                                           if (chassis_en) chassis_estop = false;
                                           std::printf("[chassis] 使能=%d\n", chassis_en ? 1 : 0); }
                    else if (ks == XK_n) oneshot_chassis_clear = true;   // 底盘里程计清零（原 c，已被颜色切换占用）
                    else if (ks == XK_x) { oneshot |= ctrl_flag::EStop;
                                           chassis_estop = true;
                                           std::printf("[chassis] 急停（握手 e/f 恢复）\n"); }
                    else if (ks == XK_h) oneshot |= ctrl_flag::Home;
                    else if (ks == XK_z) oneshot |= ctrl_flag::Zero;
                    else if (ks == XK_k) { rate[0] = rate[1] = 0.0; }   // 清空当前速率（原 R 键）
                    else if (ks == XK_r) {                              // R = 小陀螺切换
                        spin_mode = !spin_mode;
                        spin_wz_lp = 0.0;                               // 复位补偿滤波器，免得上一次残留
                        // 关掉时底盘还要 0.33 s 才停转，这段时间继续补偿，否则相机会被甩一下
                        if (!spin_mode) spin_comp_until = nowS() + 0.6;
                        std::printf("[chassis] 小陀螺 %s（自旋 %+.2f rad/s，云台反向补偿）\n",
                                    spin_mode ? "开" : "关", o.spin_rate_mradps / 1000.0);
                    }
                    else if (ks == XK_m) {
                        mouse_on = !mouse_on;
                        mouse_hist.clear();
                        XDefineCursor(dpy, win,
                                      (mouse_on && o.mouse_lock && blank_cursor != None)
                                          ? blank_cursor : None);
                        if (mouse_on) {
                            mouse_synced = false;
                            mouse_arm_t  = nowS() + 0.25;
                            mouse_hist.clear();
                            raw_acc_x = raw_acc_y = 0.0;
                            if (o.mouse_lock)
                                XWarpPointer(dpy, None, win, 0, 0, 0, 0,
                                             o.win_w / 2, o.win_h / 2);
                        }
                        std::printf("[mouse] %s\n",
                                    mouse_on ? "开（指针锁在窗口中心，移动鼠标控云台）"
                                             : "关（指针归还，云台只用方向键）");
                    }
                    else if (ks == XK_t) {
                        aim_assist = !aim_assist;
                        std::printf("[aim] 自瞄辅助 %s（%s）\n", aim_assist ? "开" : "关",
                                    aim_assist ? "把云台按 aim_yaw/aim_pitch 怼过去"
                                               : "只显示数据，不动云台");
                    }
                    else if (ks == XK_v) {
                        video_on = !video_on;
                        if (video_on) view.start(o.host, o.http_port);
                        else          view.stop();
                    }
                    break;
                }
                case GenericEvent: {
                    const XGenericEventCookie* cookie = &ev.xcookie;
                    if (cookie->extension == xi_opcode && cookie->evtype == XI_RawMotion &&
                        XGetEventData(dpy, &ev.xcookie)) {
                        const auto* raw = static_cast<const XIRawEvent*>(ev.xcookie.data);
                        // raw_values = 未经指针加速的原始位移（手感更稳定）；
                        // 个别驱动不给 raw_values，就用加速后的值
                        const double* v = raw->raw_values ? raw->raw_values
                                                          : raw->valuators.values;
                        if (v) {
                            int idx = 0;
                            for (int bit = 0; bit < raw->valuators.mask_len * 8; ++bit) {
                                if (!XIMaskIsSet(raw->valuators.mask, bit)) continue;
                                if (bit == 0)      raw_acc_x += v[idx];
                                else if (bit == 1) raw_acc_y += v[idx];
                                ++idx;
                            }
                            raw_last_t = nowS();
                        }
                        XFreeEventData(dpy, &ev.xcookie);
                    }
                    break;
                }
                case ConfigureNotify: {
                    // 窗口被 WM 改尺寸后"中心"会变：不清历史的话，这份偏移会被
                    // 当成一次甩枪（实测启动时冒出 +95°/s 的假指令）。
                    const bool size_changed = (o.win_w != ev.xconfigure.width ||
                                               o.win_h != ev.xconfigure.height);
                    o.win_w = ev.xconfigure.width;    // 鼠标中心/视频居中都用它
                    o.win_h = ev.xconfigure.height;
                    if (size_changed) {
                        XFreePixmap(dpy, back);
                        back = XCreatePixmap(dpy, win,
                                             static_cast<unsigned>(o.win_w),
                                             static_cast<unsigned>(o.win_h),
                                             static_cast<unsigned>(depth));
                    }
                    if (size_changed && mouse_on) {
                        mouse_hist.clear();
                        if (o.mouse_lock)
                            XWarpPointer(dpy, None, win, 0, 0, 0, 0,
                                         o.win_w / 2, o.win_h / 2);
                    }
                    break;
                }
                case ButtonPress:
                    // ⚠️ 只认"上一拍就已经有焦点"时的左键：Alt+Tab 切回来之后那一下
                    //    点击是用来聚焦的，不该顺手打出一发水弹。
                    if (ev.xbutton.button == Button1 && prev_focused) fire_btn = true;
                    if (ev.xbutton.button == Button3 && prev_focused) aim_btn  = true;   // 右键=临时自瞄
                    break;
                case ButtonRelease:
                    if (ev.xbutton.button == Button1) { fire_btn = false; fire_btn_up_ticks = 0; }
                    if (ev.xbutton.button == Button3) { aim_btn  = false; aim_btn_up_ticks  = 0; }
                    break;
                case FocusOut:
                    fire_btn = false;      // 失焦立刻松手，别把水弹一直打着
                    fire_btn_up_ticks = 0;
                    aim_btn  = false;      // 失焦也退出临时自瞄
                    aim_btn_up_ticks = 0;
                    break;
                case ClientMessage:
                    if (static_cast<Atom>(ev.xclient.data.l[0]) == wm_delete)
                        running = false;
                    break;
                default:
                    break;
            }
        }

        // ---------- 1a2. 自瞄数据：非阻塞收包 ----------
        if (aim_ok) aim.poll();

        // ---------- 1b. 发射扳机：鼠标左键（按住连发）+ F 键（备用）----------
        // ⚠️ 用"查状态"而不是 ButtonPress 事件：事件会丢、Release 丢了就会卡在开火。
        //    而且必须同时满足「指针在本窗口内」+「窗口有焦点」，否则你在别的窗口
        //    点一下鼠标就把水弹打出去了。
        bool fire_mouse = false;
        bool aim_hold   = false;    // 按住右键 = 临时自瞄（此时鼠标不控云台）
        {
            Window r_ret = 0, c_ret = 0;
            int rx = 0, ry = 0, wx = 0, wy = 0;
            unsigned bmask = 0;
            XQueryPointer(dpy, win, &r_ret, &c_ret, &rx, &ry, &wx, &wy, &bmask);
            Window fwin = None;
            int    revert_to = 0;
            XGetInputFocus(dpy, &fwin, &revert_to);
            const bool ptr_in  = (wx >= 0 && wy >= 0 && wx < o.win_w && wy < o.win_h);
            const bool focused = (fwin == win) || o.ignore_focus;
            const bool mask_down = (bmask & Button1Mask) != 0;
            // 兜底清位：状态位连续 10 拍（≈100ms）都是"松开"，说明 Release 事件丢了
            if (fire_btn && !mask_down) {
                if (++fire_btn_up_ticks >= 10) { fire_btn = false; fire_btn_up_ticks = 0; }
            } else {
                fire_btn_up_ticks = 0;
            }
            fire_mouse = (fire_btn || mask_down) && ptr_in && focused;
            // 右键 = 按住临时自瞄：同样带兜底清位，防止 Release 丢了卡在自瞄
            const bool mask3 = (bmask & Button3Mask) != 0;
            if (aim_btn && !mask3) {
                if (++aim_btn_up_ticks >= 10) { aim_btn = false; aim_btn_up_ticks = 0; }
            } else {
                aim_btn_up_ticks = 0;
            }
            aim_hold = (aim_btn || mask3) && ptr_in && focused && !hold_mode;
            prev_focused = focused;   // 下一拍的 ButtonPress 要过这道门
            if (o.mouse_debug && (fire_btn || mask_down)) {
                static int dbg_f = 0;
                if (dbg_f++ < 20)
                    std::fprintf(stderr,
                                 "[mouse-dbg] FIRE ev=%d mask=%d ptr_in=%d focused=%d -> %d\n",
                                 static_cast<int>(fire_btn), static_cast<int>(mask_down),
                                 static_cast<int>(ptr_in), static_cast<int>(focused),
                                 static_cast<int>(fire_mouse));
            }
        }
        fire_hold_key = !hold_mode && (fire_mouse || keyDown(dpy, XK_f));

        // ---------- 1c. 鼠标：相对位移 -> 角速度（FPS 式）----------
        // 默认：**指针位置差分 + 每拍把指针拉回窗口中心**（FPS 的经典做法）。
        //   实测踩过的三个坑，全部在这里兜住：
        //   ① XWarpPointer 对还没 viewable 的窗口是空操作 -> 首拍只同步不计速度；
        //   ② Mutter 把 warp 当运动做加速，回落位置不是正中心 -> 跳变门限 + 启动静默期
        //      （不加的话，启动瞬间会往云台灌 114°/s 的假指令）；
        //   ③ 指针被点到别的窗口后不能继续拽回来 -> 只有指针在窗口内才接管。
        // --raw-motion 可切到 XInput2 的 XI_RawMotion（原生 Xorg 更干净；
        //   但这台 XWayland 上 warp 自己也会产生 raw 事件，会污染，所以默认不用）。
        double mvx = 0.0, mvy = 0.0;                        // px/s
        if (mouse_on) {
            const double tnow = nowS();
            const bool armed = tnow >= mouse_arm_t;   // 启动/切回后的静默期内不动云台
            const bool raw_live = raw_motion && (tnow - raw_last_t < 0.3);

            if (raw_live) {
                Window r_ret = 0, c_ret = 0;
                int qrx = 0, qry = 0, qwx = 0, qwy = 0;
                unsigned qm = 0;
                XQueryPointer(dpy, win, &r_ret, &c_ret, &qrx, &qry, &qwx, &qwy, &qm);
                Window focus_win = None;
                int    revert_to = 0;
                XGetInputFocus(dpy, &focus_win, &revert_to);
                return_to_win = (focus_win == win) && qwx >= 0 && qwy >= 0 &&
                                qwx < o.win_w && qwy < o.win_h;
                if (o.mouse_debug) {
                    static int dbg_raw = 0;
                    if (dbg_raw++ < 8)
                        std::fprintf(stderr, "[mouse-dbg] RAW d=(%+.1f,%+.1f)\n",
                                     raw_acc_x, raw_acc_y);
                }
                if (raw_acc_x != 0.0 || raw_acc_y != 0.0) {
                    mouse_hist.push_back({tnow, raw_acc_x, raw_acc_y});
                }
                raw_acc_x = 0.0;
                raw_acc_y = 0.0;
                mouse_synced = false;      // 走 raw 路径时不需要位置同步
            } else {
                Window root_ret = 0, child_ret = 0;
                int rx = 0, ry = 0, wx = 0, wy = 0;
                unsigned mask = 0;
                XQueryPointer(dpy, win, &root_ret, &child_ret, &rx, &ry, &wx, &wy, &mask);
                // ⚠️ 只有"窗口有焦点 且 指针在窗口内"才接管：
                //    失焦 = 用户去用别的程序了，这时候继续把指针往中心拽
                //    会让整个桌面没法用（只能靠 m 键解脱，太别扭）。
                Window focus_win = None;
                int    revert_to = 0;
                XGetInputFocus(dpy, &focus_win, &revert_to);
                const bool focused = (focus_win == win) || o.ignore_focus;
                // 焦点从"没焦点"变成"有焦点"：用户刚切回来，先把指针锚定、
                // 静默 150ms，免得切换瞬间的指针跳变被当成甩枪。
                static bool prev_focused = false;
                if (focused && !prev_focused) {
                    mouse_hist.clear();
                    mouse_synced = false;
                    mouse_arm_t  = tnow + 0.15;
                    raw_acc_x = raw_acc_y = 0.0;
                }
                prev_focused = focused;
                const bool ptr_inside = focused &&
                    wx >= 0 && wy >= 0 && wx < o.win_w && wy < o.win_h;
                const int cx = o.win_w / 2;
                const int cy = o.win_h / 2;
                if (o.mouse_debug && (!ptr_inside || std::abs(wx - o.win_w / 2) > 100)) {
                    static int dbg_i = 0;
                    if (dbg_i++ < 40)
                        std::fprintf(stderr,
                                     "[mouse-dbg] ptr=(%d,%d) focused=%d inside=%d\n",
                                     wx, wy, static_cast<int>(focus_win == win),
                                     static_cast<int>(ptr_inside));
                }
                if (!ptr_inside) {
                    mouse_hist.clear();
                    mouse_synced = false;
                    return_to_win = false;
                } else if (!mouse_synced) {
                    return_to_win = true;
                    // 启动 / 刚切回鼠标 / 刚回到窗口：本拍只同步，不计速度
                    mouse_hist.clear();
                    mouse_synced = true;
                } else {
                    return_to_win = true;
                    const double mdx = wx - cx;
                    const double mdy = wy - cy;
                    if (std::fabs(mdx) > o.win_w * 0.2 || std::fabs(mdy) > o.win_h * 0.2) {
                        mouse_hist.clear();          // 跳变（切窗口/启动）：丢掉
                    } else {
                        mouse_hist.push_back({tnow, mdx, mdy});
                    }
                }
            }

            if (o.mouse_lock && (raw_live || return_to_win))
                XWarpPointer(dpy, None, win, 0, 0, 0, 0, o.win_w / 2, o.win_h / 2);
            if (!armed) {                     // 静默期：清样本，指针照常摆正
                mouse_hist.clear();
                raw_acc_x = raw_acc_y = 0.0;
            }
            while (mouse_hist.size() > 1 &&
                   tnow - mouse_hist.front().t > o.mouse_window)
                mouse_hist.pop_front();
            const double span = tnow - mouse_hist.front().t;
            if (span > 1e-4) {
                double sx = 0.0, sy = 0.0;
                for (const auto& m : mouse_hist) { sx += m.dx; sy += m.dy; }
                mvx = sx / span;
                mvy = sy / span;
            }
            if (std::fabs(mvx) < o.deadzone) mvx = 0.0;
            if (std::fabs(mvy) < o.deadzone) mvy = 0.0;
            if (o.mouse_debug) {
                static int dbg_p = 0;
                if ((dbg_p++ < 5) || std::fabs(mvx) > 1.0 || std::fabs(mvy) > 1.0)
                    std::fprintf(stderr, "[mouse-dbg] px/s=(%+.0f,%+.0f) raw_live=%d\n",
                                 mvx, mvy, static_cast<int>(raw_live));
            }
        }

        // ---------- 2. 云台：鼠标（主控）+ 方向键（备用）一起算目标角速度 ----------
        double target_yaw   = 0.0;
        double target_pitch = 0.0;
        if (!hold_mode) {
            if (mouse_on && !aim_hold) {
                // 鼠标右移 = 枪口右转 = yaw 减小（← 才是 +yaw）；下移 = 低头
                // ⚠️ 按住右键（临时自瞄）时鼠标不控云台，松手立刻交还鼠标
                target_yaw   += -o.sens * mvx;
                target_pitch += (o.invert_y ? o.sens : -o.sens) * o.pitch_scale * mvy;
            }
            if (keyDown(dpy, XK_Left))  target_yaw   += o.gimbal_rate;
            if (keyDown(dpy, XK_Right)) target_yaw   -= o.gimbal_rate;
            if (keyDown(dpy, XK_Up))    target_pitch += o.gimbal_rate * o.pitch_scale;
            if (keyDown(dpy, XK_Down))  target_pitch -= o.gimbal_rate * o.pitch_scale;
        }

        // ---------- 2b. 自瞄辅助（t 键，默认关）----------
        // aim_yaw/aim_pitch 是"相机系里目标相对光轴的角误差"（右/上为正），
        // 所以就是最简单的比例控制：yaw 取负（向右转 = yaw 减小），pitch 取正。
        // ⚠️ 数据过期（> --aim-stale）或 valid=0 时立刻停手，不会拿旧角度乱怼。
        aim_dps[0] = aim_dps[1] = 0.0;
        aim_have = aim_ok && aim.fresh(nowS(), o.aim_stale);
        if (aim_have) aim.latest(aim_state);
        const bool assist_active = (aim_assist || aim_hold) && !hold_mode;
        if (assist_active && aim_have && aim_state.valid) {
            // 偏置：err = 实测aim角 − 期望aim角
            //   bias_pitch > 0 = 瞄目标下方（枪口压低 → 目标相对光轴上移 → aim_pitch 收敛到 +bias）
            //   bias_yaw   > 0 = 瞄目标右侧
            const double ey = aim_state.yaw_deg   - o.aim_bias_yaw;
            const double ep = aim_state.pitch_deg - o.aim_bias_pitch;
            if (std::fabs(ey) > o.aim_deadband) aim_dps[0] = -o.aim_kp_yaw   * ey;
            if (std::fabs(ep) > o.aim_deadband) aim_dps[1] =  o.aim_kp_pitch * ep;
            target_yaw   += aim_dps[0];
            target_pitch += aim_dps[1];
        }

        target_yaw   = std::max(-o.max_rate, std::min(o.max_rate, target_yaw));
        // pitch 单独再夹一道：gimbal 侧 pitch 轴上限就是 --pitch-rate（默认 120），
        // 客户端没必要给它超过这个的指令。
        const double pmax = std::min(o.max_rate, o.pitch_max_rate);
        target_pitch = std::max(-pmax, std::min(pmax, target_pitch));

        // ---------- 2c. pitch 软限位保护（别撞到机械限位）----------
        // ⚠️ gimbal 侧软限位 ±20° 一旦到头是 clampToLimits() **硬夹到 0**
        //    （axis.cpp:19-20），等于急停，对电机和机构都不友好。
        //    所以客户端在还差 --pitch-margin 度时就开始线性降速，平滑地贴住；
        //    同时认可 gimbal 反馈的 PitchLim 状态位（那个是权威的"已经贴限位"）。
        double pitch_guard = 1.0;
        const bool telem_fresh = have_telem && (nowS() - t_last_telem < 0.3);
        if (!o.no_pitch_guard && telem_fresh && target_pitch != 0.0) {
            const double ang  = last_telem.pitch_deg;
            const double marg = std::max(0.1, o.pitch_margin);
            if (target_pitch > 0.0) {
                pitch_guard = std::max(0.0, std::min(1.0, (o.pitch_max - ang) / marg));
            } else {
                pitch_guard = std::max(0.0, std::min(1.0, (ang - o.pitch_min) / marg));
            }
            // 状态位已经报"贴限位"：只放行往回走的指令
            if (last_telem.status & ctrl_status::PitchLim) {
                const bool near_max = ang >= (o.pitch_min + o.pitch_max) * 0.5;
                if ((near_max && target_pitch > 0.0) || (!near_max && target_pitch < 0.0)) {
                    pitch_guard = 0.0;
                }
            }
            target_pitch *= pitch_guard;
        }

        const double a = std::min(1.0, dt_target / std::max(1e-3, o.tau));
        rate[0] += (target_yaw   - rate[0]) * a;
        rate[1] += (target_pitch - rate[1]) * a;

        // ---------- 3b. 底盘：键盘 -> 平滑限幅（加速度限幅防打滑/冲击）----------
        // 按一下 Shift（左右均可）= 切换快速/普通档，锁存到再按一次为止；松开不退回。
        // 用「上一拍 vs 这一拍」的上升沿做切换：XQueryKeymap 轮询的是物理按键状态，
        // 长按只会保持 true，不会像 KeyPress 事件那样被 X11 自动重复刷爆。
        const bool shift_now = keyDown(dpy, XK_Shift_L) || keyDown(dpy, XK_Shift_R);
        if (shift_now && !shift_prev) {
            chassis_fast = !chassis_fast;
            std::printf("[chassis] 切到%s档：平移 %.0f mm/s / 转向 %.0f mrad/s\n",
                        chassis_fast ? "快速" : "普通",
                        chassis_fast ? o.chassis_speed_fast : o.chassis_speed,
                        chassis_fast ? o.chassis_turn_fast  : o.chassis_turn);
        }
        shift_prev = shift_now;

        const double spd  = chassis_fast ? o.chassis_speed_fast : o.chassis_speed;
        const double turn = chassis_fast ? o.chassis_turn_fast  : o.chassis_turn;

        double tvx = 0.0, tvy = 0.0, twz = 0.0;
        if (!hold_mode && !chassis_estop) {
            if (keyDown(dpy, XK_w))      tvx += spd;
            if (keyDown(dpy, XK_s))      tvx -= spd;
            if (keyDown(dpy, XK_a))      tvy += spd;    // +y = 左移
            if (keyDown(dpy, XK_d))      tvy -= spd;
            if (!spin_mode) {                           // 小陀螺时旋转交给自旋速率，Q/E 让位
                if (keyDown(dpy, XK_q))  twz += turn;   // Q = 左转（+wz 逆时针）
                if (keyDown(dpy, XK_e))  twz -= turn;   // E = 右转
            }
            if (spin_mode) twz = o.spin_rate_mradps;    // 小陀螺：恒定自旋（WASD 平移照常）
        }
        {
            // 平移 3000 mm/s²、转向 6 rad/s²：满速约 0.1~0.25 s
            const double max_lin = 3000.0 * dt_target;
            const double max_ang = 6000.0 * dt_target;
            auto slew = [](double& v, double t, double step) {
                const double d = t - v;
                v += std::max(-step, std::min(step, d));
            };
            slew(cvx, tvx, max_lin);
            slew(cvy, tvy, max_lin);
            slew(cwz, twz, max_ang);
        }

        // ---------- 3c. 小陀螺：yaw 反向补偿，让云台在世界系里不转 ----------
        // 底盘在世界系里转 +ω，yaw 电机就必须相对底盘转 −ω，相机才站得住。
        // 补偿以**指令值 cz 为基准**，实测陀螺只用来"修正"：
        //   实测与指令同号、且量级 ≥ 指令的 25% 时才采信（能吸掉打滑/掉速的误差）；
        //   否则退回指令值 —— 陀螺坏掉/轴映射不对时会恒报 0，
        //   无脑采信会让补偿直接变成 0，云台跟着车一起转（实机踩过）。
        double spin_comp_dps = 0.0;
        const bool comp_on = spin_mode || (nowS() < spin_comp_until);
        if (comp_on) {
            const bool fresh  = have_telem && (nowS() - t_last_telem < 0.3);
            const bool imu_ok = fresh &&
                ((last_telem.chassis_status & chassis_status::ImuOnline) != 0);
            double wz_src = cwz;
            if (imu_ok) {
                const double meas = last_telem.chassis_gyro_wz_mradps;
                if (meas * cwz > 0.0 && std::fabs(meas) >= 0.25 * std::fabs(cwz)) {
                    wz_src = meas;          // 实测可信：用它抵消打滑
                }
            }
            const double alp = std::min(1.0, dt_target / 0.03);   // 30 ms 低通压陀螺噪声
            spin_wz_lp += (wz_src - spin_wz_lp) * alp;
            spin_comp_dps = -o.spin_gain * spin_wz_lp * 0.001 * 57.29577951308232;  // rad/s -> °/s
        } else {
            spin_wz_lp = 0.0;
        }

        // ---------- 4. 组包发送 ----------
        ControlCommand cc;
        cc.seq       = ++seq;
        // 小陀螺时补偿一直生效（哪怕按了空格"停手"）：底盘自旋在减速的那 0.3 s 里
        // 仍然在转，yaw 必须继续抵着，相机才不会跟着甩一下。
        cc.yaw_dps   = (hold_mode ? 0.0 : rate[0]) + spin_comp_dps;
        cc.pitch_dps = hold_mode ? 0.0 : rate[1];
        cc.flags     = oneshot;
        oneshot      = 0;
        if (locked)          cc.flags |= ctrl_flag::Disable | ctrl_flag::Hold;
        else if (en_state)   cc.flags |= ctrl_flag::Enable;
        else                 cc.flags |= ctrl_flag::Disable;
        if (hold_mode)       cc.flags |= ctrl_flag::Hold;
        if (fire_hold_key)   cc.flags |= ctrl_flag::Fire;   // 按住 F 发射

        // ---- 底盘字段（坐标：vx 前进 / vy 左移 / wz 逆时针）----
        cc.chassis_vx_mmps   = cvx;
        cc.chassis_vy_mmps   = cvy;
        cc.chassis_wz_mradps = cwz;
        cc.chassis_flags     = 0;
        if (locked || chassis_estop)  cc.chassis_flags |= chs_flag::EStop;
        else if (chassis_en)          cc.chassis_flags |= chs_flag::Enable;
        if (oneshot_chassis_clear) {  cc.chassis_flags |= chs_flag::Clear;
                                      oneshot_chassis_clear = false; }
        if (!o.no_send) udp.send(cc);   // --no-send：只收不发（云台归 27RM --gimbal-aim）

        // ---------- 5. 收遥测 ----------
        ControlTelemetry t;
        while (udp.recv(t)) {
            if (last_echo && t.echo_seq > last_echo + 1)
                lost_pkts += static_cast<double>(t.echo_seq - last_echo - 1);
            last_echo = t.echo_seq;
            last_telem = t;
            have_telem = true;
            t_last_telem = nowS();      // 小陀螺补偿要判断遥测是否新鲜
        }

        // ---------- 6. 渲染（约 30fps，别挡控制）----------
        // ⚠️ 必须写成"距上次渲染超过 N ms 才渲染"。
        //    绝不能写 t_render += N ms 再比大小：循环每轮只过 10ms 却加 N ms，
        //    t_render 会飞出去，那个 if 就永远不成立（踩过）。
        //    16ms（≈60Hz）而不是 33ms：视频 30fps 时，33ms 的渲染节拍会和帧到达
        //    撞成拍频，实测"拉流 30fps 但只画出 17~24fps"。
        // 渲染条件：有新画面就画（跟到 30fps），否则最多 100ms 刷一次 HUD。
        // 这样每帧只画一次、不重画文字，省掉每秒上千次 X 请求（实测绘制 fps 从 24 提到 30）。
        const auto t_now = std::chrono::steady_clock::now();
        static auto t_last_draw = std::chrono::steady_clock::now();
        bool have_new_frame = false;
        MjpegView::Frame new_frame;
        if (video_on) have_new_frame = view.getLatest(new_frame, shown_seq);
        const bool time_to_draw =
            have_new_frame ||
            (std::chrono::duration<double>(t_now - t_last_draw).count() >= 0.1);
        if (time_to_draw && t_now - t_render >= std::chrono::milliseconds(8)) {
            t_render    = t_now;
            t_last_draw = t_now;


            XSetForeground(dpy, gc, BlackPixel(dpy, scr));
            XFillRectangle(dpy, back, gc, 0, 0,
                           static_cast<unsigned>(o.win_w),
                           static_cast<unsigned>(o.win_h));
            XSetForeground(dpy, gc, WhitePixel(dpy, scr));

            // 视频居中绘制
            if (video_on && truecolor) {
                MjpegView::Frame& f = new_frame;
                if (have_new_frame && f.width > 0) {
                    if (!video_first_done) {
                        video_first_done = true;
                        std::printf("[video] 首帧 %dx%d 已解码并绘制\n",
                                    f.width, f.height);
                    }
                    const int vx = std::max(0, (o.win_w - f.width) / 2);
                    const int vy = std::max(0, (o.win_h - f.height) / 2);
                    XImage* img = XCreateImage(dpy, vis, static_cast<unsigned>(depth),
                                               ZPixmap, 0,
                                               reinterpret_cast<char*>(f.bgrx.data()),
                                               static_cast<unsigned>(f.width),
                                               static_cast<unsigned>(f.height),
                                               32, f.width * 4);
                    if (img) {
                        XPutImage(dpy, back, gc, img, 0, 0, vx, vy,
                                  static_cast<unsigned>(f.width),
                                  static_cast<unsigned>(f.height));
                        ++video_frames;
                        img->data = nullptr;   // 别让 XDestroyImage 释放我们的 buffer
                        XDestroyImage(img);
                    }
                }
            }

            // ---- 准星：画在窗口正中 ----
            // 视频是按等比缩放居中贴的，所以窗口中心 = 光轴指向 = 该打的地方。
            // 指针被锁在中心且光标是隐形的，没有准星就完全不知道打哪。
            // 画法：先用黑色在 +1px 偏移处描一遍边，再叠亮色主线，白墙黑板上都看得清。
            if (!o.no_reticle) {
                const int cx  = o.win_w / 2 + o.reticle_dx;
                const int cy  = o.win_h / 2 + o.reticle_dy;
                const int r   = std::max(6, o.reticle_size);
                const int gap = std::max(2, r / 3);      // 中心留空，别挡住目标
                auto cross = [&](int off) {
                    XDrawLine(dpy, back, gc, cx - r + off, cy + off, cx - gap + off, cy + off);
                    XDrawLine(dpy, back, gc, cx + gap + off, cy + off, cx + r + off, cy + off);
                    XDrawLine(dpy, back, gc, cx + off, cy - r + off, cx + off, cy - gap + off);
                    XDrawLine(dpy, back, gc, cx + off, cy + gap + off, cx + off, cy + r + off);
                    XDrawArc(dpy, back, gc, cx - 2 + off, cy - 2 + off, 5, 5, 0, 360 * 64);
                };
                XSetForeground(dpy, gc, 0x000000);
                cross(1);
                XSetForeground(dpy, gc, 0x00FF00);   // 亮绿
                cross(0);
                XSetForeground(dpy, gc, WhitePixel(dpy, scr));   // 恢复文字颜色
            }

            // 状态文字
            char line[192];
            int  ty = 16;
            auto text = [&](const char* s, unsigned long color = 0) {
                if (color != 0) XSetForeground(dpy, gc, color);
                XDrawString(dpy, back, gc, 10, ty, s, static_cast<int>(std::strlen(s)));
                if (color != 0) XSetForeground(dpy, gc, WhitePixel(dpy, scr));
                ty += 15;
            };

            std::snprintf(line, sizeof(line), "gimbal_ctl -> %s:%d   [mouse + arrows]",
                          o.host.c_str(), o.ctrl_port);
            text(line);

            if (mouse_on) {
                std::snprintf(line, sizeof(line),
                              "mouse %+6.0f,%+6.0f px/s  sens %.2f  lock %s  (m=toggle)",
                              mouse_px[0], mouse_px[1], o.sens,
                              o.mouse_lock ? "on" : "off");
            } else {
                std::snprintf(line, sizeof(line), "mouse OFF (m=toggle) - arrows only");
            }
            text(line);

            mouse_px[0] = mvx;
            mouse_px[1] = mvy;
            if (o.no_send)
                text("** MONITOR / NO-SEND ** 只收不发（云台由 27RM --gimbal-aim 控制）");
            if (locked)        text("** LOCKED ** 云台失能 + 底盘失能（按空格解除）");
            else if (hold_mode) text("** HOLD ** (space to resume)");
            if (spin_mode) {
                std::snprintf(line, sizeof(line),
                              "** SPIN cmd %+.2f  act %+.2f rad/s ** yaw comp %+.0f deg/s (R)",
                              o.spin_rate_mradps / 1000.0, spin_wz_lp / 1000.0,
                              spin_comp_dps);
                text(line);
            }
            if (chassis_fast) {
                std::snprintf(line, sizeof(line),
                              "** FAST ** %.0f mm/s  %.1f rad/s (shift toggled)",
                              spd, turn / 1000.0);
                text(line);
            }
            if (fire_hold_key) text("** FIRE ** (left mouse / F held)");

            // 云台被失能/急停锁死时，Jetson 会把速率强制归零（连 F6 帧都不发），
            // 现场表现就是"yaw 完全不动"——必须在界面上喊出来，不然只能靠猜。
            if (have_telem && !(last_telem.status & ctrl_status::Enabled)) {
                text("** GIMBAL DISABLED/LOCKED ** 按 g 关再开(两下)重新授权");
            }

            if (have_telem) {
                std::snprintf(line, sizeof(line),
                              "yaw    cmd %+7.1f  act %+7.1f  out %+9.2f deg",
                              rate[0], last_telem.yaw_dps, last_telem.yaw_deg);
                text(line);
                std::snprintf(line, sizeof(line),
                              "pitch  cmd %+7.1f  act %+7.1f  out %+9.2f deg  "
                              "lim %+.0f/%+.0f  guard %3.0f%%",
                              rate[1], last_telem.pitch_dps, last_telem.pitch_deg,
                              o.pitch_min, o.pitch_max, pitch_guard * 100.0);
                text(line);
                if (pitch_guard < 0.999) {
                    std::snprintf(line, sizeof(line),
                                  "** PITCH LIMIT ** %.1f deg 内减速（限位 %+.0f/%+.0f，别硬顶）",
                                  last_telem.pitch_deg, o.pitch_min, o.pitch_max);
                    text(line);
                }
                std::snprintf(line, sizeof(line),
                              "motor %+7.1f / %+7.1f RPM  %5.2fV  %dC  status=0x%02X",
                              last_telem.yaw_rpm, last_telem.pitch_rpm,
                              last_telem.vbus_mv / 1000.0, last_telem.temp_c,
                              last_telem.status);
                text(line);

                const char* st = "OK";
                if (last_telem.status & ctrl_status::EStop)    st = "ESTOP";
                else if (last_telem.status & ctrl_status::Stall)    st = "STALL!";
                else if (last_telem.status & ctrl_status::Watchdog) st = "WATCHDOG!";
                else if (!(last_telem.status & ctrl_status::YawOk) ||
                         !(last_telem.status & ctrl_status::PitchOk)) st = "MOTOR LOST!";

                std::snprintf(line, sizeof(line),
                              "link %dHz lost %.0f  RTT %.1fms  %s  rx=%dHz",
                              static_cast<int>(o.hz), lost_pkts, udp.rttMs(), st,
                              last_telem.rx_hz);
                text(line);

                // ---- 底盘遥测 ----
                std::snprintf(line, sizeof(line),
                              "chassis pos %+7.0f,%+7.0f mm  yaw %+7.2f  cmd %+5.0f,%+5.0f mm/s",
                              last_telem.chassis_x_mm, last_telem.chassis_y_mm,
                              last_telem.chassis_yaw_deg, cvx, cvy);
                text(line);
                std::snprintf(line, sizeof(line),
                              "        wheel FL%+6.1f FR%+6.1f BL%+6.1f BR%+6.1f RPM",
                              last_telem.chassis_wheel_rpm[0], last_telem.chassis_wheel_rpm[1],
                              last_telem.chassis_wheel_rpm[2], last_telem.chassis_wheel_rpm[3]);
                text(line);
                std::snprintf(line, sizeof(line),
                              "        act %+5.0f,%+5.0f mm/s  gyro_yaw %+7.2f  chassis %s  %s",
                              last_telem.chassis_vx_mmps, last_telem.chassis_vy_mmps,
                              last_telem.chassis_gyro_yaw_deg,
                              last_telem.chassis_valid ? "ONLINE" : "OFFLINE",
                              (locked || chassis_estop) ? "[ESTOP]"
                                                         : (chassis_en ? "" : "[DISABLED]"));
                text(line);
            } else {
                text("waiting for telemetry ... (check gimbal running + --host)");
            }

            // ---- 自瞄（27RM UDP 40002）----
            if (aim_ok) {
                if (!aim_have) {
                    std::snprintf(line, sizeof(line),
                                  "aim 无数据 (27RM 没跑? %.0fHz lost %u) %s",
                                  aim.rateHz(), aim.lost(), aim.peer().c_str());
                } else if (!aim_state.valid) {
                    std::snprintf(line, sizeof(line),
                                  "aim 无可打目标  板%u  d %.2fm  %.0fHz lost %u",
                                  aim_state.plate_slot, aim_state.dist_m,
                                  aim.rateHz(), aim.lost());
                } else {
                    std::snprintf(line, sizeof(line),
                                  "aim %s 敌方%s 板%u%s  yaw %+6.2f pitch %+6.2f  d %.2fm  "
                                  "spin %+.0fdeg/s  %.0fHz lost %u",
                                  aim_state.spin_tracking ? "反小陀螺" : "单板",
                                  aim_state.color_id == 2 ? "红"
                                      : (aim_state.color_id == 1 ? "蓝" : "?"),
                                  aim_state.plate_slot,
                                  aim_state.facing ? "(朝向)" : "",
                                  aim_state.yaw_deg, aim_state.pitch_deg, aim_state.dist_m,
                                  aim_state.spin_dps, aim.rateHz(), aim.lost());
                }
                text(line);
                if (aim_assist || aim_hold) {
                    std::snprintf(line, sizeof(line),
                                  "** %s ** yaw %+6.1f pitch %+6.1f deg/s  %s",
                                  aim_hold ? "AIMING (右键按住)" : "AIM ASSIST",
                                  aim_dps[0], aim_dps[1],
                                  aim_hold ? "松开右键交还鼠标" : "(t=toggle)");
                    text(line, aim_hold ? 0x0060FFFFu : 0x00EBEBEBu);
                }
            } else if (!o.aim_off) {
                text("aim 接收未启动（端口打不开，或没编进 aim_udp.hpp）");
            }

            if (video_on) {
                std::snprintf(line, sizeof(line), "video %.1f fps %s",
                              view.fps(), view.connected() ? "connected" : "connecting...");
            } else {
                std::snprintf(line, sizeof(line), "video off (press v)");
            }
            text(line);

            XCopyArea(dpy, back, win, gc, 0, 0,
                      static_cast<unsigned>(o.win_w),
                      static_cast<unsigned>(o.win_h), 0, 0);
            XFlush(dpy);

            // 每秒打一行视频统计到 stdout（方便无头验证）
            const auto ts = std::chrono::steady_clock::now();
            const double dtv = std::chrono::duration<double>(ts - t_vstat).count();
            if (dtv >= 1.0) {
                std::printf("[video] 绘制 %.1f fps   拉流 %.1f fps\n",
                            video_frames / dtv, view.fps());
                video_frames = 0;
                t_vstat = ts;
                if (aim_ok) {
                    if (!aim_have) {
                        std::printf("[aim] 无数据  %.0fHz  lost %u  %s\n",
                                    aim.rateHz(), aim.lost(), aim.peer().c_str());
                    } else {
                        std::printf("[aim] %.0fHz lost %u valid=%d %s 板%u  yaw %+6.2f "
                                    "pitch %+6.2f  d %.2fm  spin %+.0f°/s%s\n",
                                    aim.rateHz(), aim.lost(), aim_state.valid ? 1 : 0,
                                    aim_state.spin_tracking ? "反小陀螺" : "单板",
                                    aim_state.plate_slot, aim_state.yaw_deg,
                                    aim_state.pitch_deg, aim_state.dist_m, aim_state.spin_dps,
                                    aim_assist ? "   [辅助开]" : "");
                    }
                }
            }
        }

        // ---------- 7. 定频 ----------
        t_next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(dt_target));
        const auto now = std::chrono::steady_clock::now();
        if (t_next > now) std::this_thread::sleep_until(t_next);
        else              t_next = now;
    }

    // ---- 收尾：明确发一帧停手，别让云台保持上一个速率 ----
    ControlCommand fin;
    fin.seq = ++seq; fin.yaw_dps = 0; fin.pitch_dps = 0;
    fin.flags = ctrl_flag::Hold | (en_state ? ctrl_flag::Enable : 0);
    if (!o.no_send) {
        for (int i = 0; i < 3; ++i) { udp.send(fin); usleep(2000); }
    }

    view.stop();
    aim.close();
    if (blank_cursor != None) XFreeCursor(dpy, blank_cursor);
    if (back != None) XFreePixmap(dpy, back);
    if (font) XFreeFont(dpy, font);
    XFreeGC(dpy, gc);
    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);
    std::printf("\n已发送停手指令并退出\n");
    return 0;
}
