/**
 * @file  main.cpp
 * @brief 27RM 自动瞄准视觉主程序（校内赛版，含反小陀螺目标跟踪）
 *
 * 功能：USB 摄像头（800x600 MJPG）→ 灯条检测 → 装甲板 PnP 位姿
 *       → 平滑滤波 →（反小陀螺）四板环模型目标跟踪 → 预测瞄准 + 终端输出。
 *
 * 输出内容：距离（m）、yaw/pitch（度，相机系）、装甲板中心坐标（mm）、
 *          板面法线与视线夹角（度）、重投影误差（px）、处理耗时与采集帧率；
 *          目标锁定后输出机器人中心/半径/转速与预测瞄准角。
 *
 * 用法：
 *   ./27RM                          相机实时（需要显示器）
 *   ./27RM --no-gui                 无窗口，只打印（适合 ssh 跑）
 *   ./27RM --frames 500             无窗口跑 500 帧后自动退出（测速用）
 *   ./27RM --bench                  只测采集帧率后退出（不用标定/检测）
 *   ./27RM --track-test             反小陀螺合成自检（不用相机/标定）
 *   ./27RM --save-frame F           抓一帧存 PNG 后退出
 */

#include "aim_filter.hpp"
#include "camera_calib.hpp"
#include "camera_usb.hpp"
#include "aim_udp.hpp"
#include "gimbal_cmd.hpp"
#include "light_bar.hpp"
#include "mjpeg_stream.hpp"
#include "target_tracker.hpp"

#include <opencv2/opencv.hpp>

// 客户端命令口（UDP）：--no-gui 时没有终端按 u，改用网络收
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <thread>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Options {
  std::string calib_path = "configs/camera.yaml";
  std::string lightbar_path = "configs/light_bar.yaml";
  std::string armor_path = "configs/armor.yaml";
  std::string runtime_path = "configs/camera_runtime.yaml";
  std::string target_path = "configs/target.yaml";
  int device = 0;
  int width = 800;
  int height = 600;
  std::string format;      // "mjpg"/"yuyv"/空（默认 MJPG 优先）
  bool no_gui = false;
  bool bench = false;      // --bench：只测采集帧率
  bool track_test = false; // --track-test：反小陀螺合成自检
  int stream_port = 8080;  // 图传 HTTP 端口（0=关闭）
  std::string stream_bind = "";  // 图传监听地址：空=所有网卡；可给 IP 或网卡名
  // 20 fps / q70 是实测稳定的组合（30fps+q85 = 20Mbps 会把 2.4G 塞爆 → 画面一顿一顿）
  int stream_fps = 30;     // 图传帧率（只影响发出去的图，不影响识别）
  std::string aim_udp = "192.168.2.255:40002";  // 自瞄 UDP 输出（off=关闭；子网广播）
  bool anti_spin = false;  // 反小陀螺模式（默认关；运行中按 U 切换）
  int  cmd_port = 40003;   // 客户端命令口（UDP，'u' = 切模式；0 = 关）
  int  stream_quality = 60;  // 图传 JPEG 质量 1~100（只影响发出去的图，不影响识别）
  bool stream_raw    = false; // --stream-raw：直发相机原始 MJPEG（零转码、无叠加框、最省 CPU）
  double stream_scale = 0.75; // --stream-scale：编码前缩放（0.75=600x450，实测 30fps 只需 1.6 Mbps）
  bool gimbal_aim = false;       // 闭环自瞄：27RM 直接控云台
  std::string gimbal_host = "127.0.0.1";  // 云台守护进程地址
  int gimbal_port = 40001;       // 云台守护进程 UDP 端口
  double gimbal_kp = 20.0;       // 角速度比例增益（°/s per °）
  double gimbal_yaw_max = 180.0;    // yaw 角速度上限（°/s）
  double gimbal_pitch_max = 120.0;  // pitch 角速度上限（°/s）
  double gimbal_deadband = 0.5;     // 瞄准死区（°）
  bool gimbal_invert_yaw = false;   // yaw 方向反了时置 true
  bool gimbal_invert_pitch = false; // pitch 方向反了时置 true
  int frames = 0;          // >0：跑完这么多帧自动退出
  std::string color = "blue";  // 敌方颜色 blue/red
  std::string save_frame;      // --save-frame：抓一帧存 PNG 后退出
};

void PrintUsage(const char* prog) {
  std::printf(
      "用法: %s [选项]\n"
      "  --calib FILE      标定文件（默认 configs/camera.yaml）\n"
      "  --lightbar FILE   检测参数（默认 configs/light_bar.yaml）\n"
      "  --armor FILE      装甲板尺寸（默认 configs/armor.yaml）\n"
      "  --runtime FILE    相机运行时参数（默认 configs/camera_runtime.yaml）\n"
      "  --target FILE     反小陀螺参数（默认 configs/target.yaml）\n"
      "  --device N        /dev/videoN（默认 0）\n"
      "  --width/--height  采集分辨率（默认 800x600，MJPG）\n"
      "  --format mjpg|yuyv 采集格式（默认 MJPG 优先；相机 MJPG 坏了时用 yuyv）\n"
      "  --no-gui          不开窗口（无显示器时用）\n"
      "  --bench           只测采集帧率后退出（不用标定/检测）\n"
      "  --track-test      反小陀螺合成自检（不用相机/标定）\n"
      "  --frames N        采集 N 帧后退出（测帧率/自动化用）\n"
      "  --color blue/red  敌方颜色（默认 blue）\n"
      "  --stream-port N   图传 HTTP 端口（默认 8080，0=关闭）\n"
      "  --stream-bind A   图传只在这个地址/网卡上监听（如 wlan0；默认所有网卡）\n"
      "  --stream-fps N    图传帧率（默认 30；只影响发出去的图，不影响识别）\n"
      "  --stream-quality Q 图传 JPEG 质量 1~100（默认 70；降它省带宽）\n"
      "  --stream-raw      图传直发相机原始 MJPEG（零转码、无检测框、最省 CPU）\n"
      "  --stream-scale F  编码前缩放图传画面（如 0.75 = 600x450；不影响识别）\n"
      "  --aim-udp A:P    自瞄数据 UDP 输出地址（默认 192.168.2.255:40002，off=关闭）\n"
      "  --anti-spin      启动即开启反小陀螺模式（默认关；运行中按 U 切换）\n"
      "  --cmd-port N     客户端命令口 UDP（默认 40003，0=关闭；发 'u' 切模式）\n"
      "  --gimbal-aim      闭环自瞄：27RM 直接控云台跟踪（需 gimbal 守护进程在跑）\n"
      "  --gimbal-host IP  云台守护进程地址（默认 127.0.0.1）\n"
      "  --gimbal-port N   云台守护进程 UDP 端口（默认 40001）\n"
      "  --gimbal-kp F     跟踪比例增益（默认 20.0，°/s per °）\n"
      "  --gimbal-yaw-max F / --gimbal-pitch-max F  角速度上限（默认 180/120 °/s）\n"
      "  --gimbal-deadband F 瞄准死区（默认 0.5°）\n"
      "  --gimbal-invert-yaw / --gimbal-invert-pitch  方向反了时翻转\n"
      "  --save-frame F    抓一帧存成 PNG 后退出（离线调参用）\n",
      prog);
}

bool ParseArgs(int argc, char** argv, Options* opt) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "参数 %s 缺少数值\n", name);
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--calib") {
      opt->calib_path = next("--calib");
    } else if (a == "--lightbar") {
      opt->lightbar_path = next("--lightbar");
    } else if (a == "--armor") {
      opt->armor_path = next("--armor");
    } else if (a == "--runtime") {
      opt->runtime_path = next("--runtime");
    } else if (a == "--target") {
      opt->target_path = next("--target");
    } else if (a == "--device") {
      opt->device = std::atoi(next("--device"));
    } else if (a == "--width") {
      opt->width = std::atoi(next("--width"));
    } else if (a == "--height") {
      opt->height = std::atoi(next("--height"));
    } else if (a == "--format") {
      opt->format = next("--format");
    } else if (a == "--no-gui") {
      opt->no_gui = true;
    } else if (a == "--bench") {
      opt->bench = true;
    } else if (a == "--track-test") {
      opt->track_test = true;
    } else if (a == "--frames") {
      opt->frames = std::atoi(next("--frames"));
    } else if (a == "--color") {
      opt->color = next("--color");
    } else if (a == "--stream-bind") {
      opt->stream_bind = next("--stream-bind");
    } else if (a == "--stream-port") {
      opt->stream_port = std::atoi(next("--stream-port"));
    } else if (a == "--stream-scale") {
      opt->stream_scale = std::atof(next("--stream-scale"));
      if (opt->stream_scale < 0.2) opt->stream_scale = 0.2;
      if (opt->stream_scale > 1.0) opt->stream_scale = 1.0;
    } else if (a == "--stream-raw") {
      opt->stream_raw = true;
    } else if (a == "--stream-quality") {
      opt->stream_quality = std::atoi(next("--stream-quality"));
      if (opt->stream_quality < 1) opt->stream_quality = 1;
      if (opt->stream_quality > 100) opt->stream_quality = 100;
    } else if (a == "--stream-fps") {
      opt->stream_fps = std::atoi(next("--stream-fps"));
    } else if (a == "--aim-udp") {
      opt->aim_udp = next("--aim-udp");
    } else if (a == "--cmd-port") {
      opt->cmd_port = std::atoi(next("--cmd-port"));
    } else if (a == "--anti-spin") {
      opt->anti_spin = true;
    } else if (a == "--gimbal-aim") {
      opt->gimbal_aim = true;
    } else if (a == "--gimbal-host") {
      opt->gimbal_host = next("--gimbal-host");
    } else if (a == "--gimbal-port") {
      opt->gimbal_port = std::atoi(next("--gimbal-port"));
    } else if (a == "--gimbal-kp") {
      opt->gimbal_kp = std::atof(next("--gimbal-kp"));
    } else if (a == "--gimbal-yaw-max") {
      opt->gimbal_yaw_max = std::atof(next("--gimbal-yaw-max"));
    } else if (a == "--gimbal-pitch-max") {
      opt->gimbal_pitch_max = std::atof(next("--gimbal-pitch-max"));
    } else if (a == "--gimbal-deadband") {
      opt->gimbal_deadband = std::atof(next("--gimbal-deadband"));
    } else if (a == "--gimbal-invert-yaw") {
      opt->gimbal_invert_yaw = true;
    } else if (a == "--gimbal-invert-pitch") {
      opt->gimbal_invert_pitch = true;
    } else if (a == "--save-frame") {
      opt->save_frame = next("--save-frame");
    } else if (a == "-h" || a == "--help") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      std::fprintf(stderr, "未知参数: %s\n", a.c_str());
      return false;
    }
  }
  return true;
}

const char* StateName(const AimOutput& out) {
  if (out.valid) return "锁定";
  if (out.measured) return "确认中";
  return "丢失";
}

/** 从 PnP 位姿取板面法线（模型 +x 轴 → 相机系单位向量） */
cv::Vec3d PoseNormal(const cv::Vec3d& rvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  cv::Vec3d n(R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0));
  const double len = cv::norm(n);
  return len > 1e-9 ? n / len : cv::Vec3d(0, 0, -1);
}

// ---------------------------------------------------------------
// 反小陀螺合成自检：无相机/标定，验证环模型跟踪与预测
// ---------------------------------------------------------------
int RunTrackTest() {
  std::printf("=== 反小陀螺合成自检 ===\n");
  std::printf("模拟旋转机器人：4 板环、R=150mm、转速 6 rad/s，"
              "每帧随机看到 1~2 块板（带噪声）\n\n");

  rm27::TargetTracker tracker;
  const double R_true = 150.0;
  const double w_true = 6.0;  // rad/s
  const cv::Vec3d c_true(0, 120, 2000);
  const double phi0 = 0.35;
  const double dt = 1.0 / 60.0;
  const int frames = 600;
  const double pred_delay = 0.04;
  cv::RNG rng(42);
  std::mt19937 gen(42);

  auto true_plates = [&](double t) {
    std::vector<cv::Vec3d> out;
    for (int k = 0; k < 4; ++k) {
      const double ang = phi0 + w_true * t + k * kPi / 2.0;
      out.emplace_back(c_true[0] + R_true * std::sin(ang), c_true[1],
                       c_true[2] + R_true * std::cos(ang));
    }
    return out;
  };

  double center_err = 0.0, w_err = 0.0, r_err = 0.0, pred_err = 0.0;
  int n_eval = 0;
  for (int i = 0; i < frames; ++i) {
    const double t = i * dt;
    // 生成观测（只保留朝向相机的板）
    std::vector<rm27::PlateObservation> obs_list;
    for (int k = 0; k < 4; ++k) {
      const double ang = phi0 + w_true * t + k * kPi / 2.0;
      const cv::Vec3d n(std::sin(ang), 0, std::cos(ang));
      const cv::Vec3d p = c_true + R_true * n;
      const double dist = cv::norm(p);
      const double face =
          -(n[0] * p[0] + n[2] * p[2]) / std::max(1e-6, dist);
      if (face < 0.25) continue;  // 背对相机的板看不到
      cv::Vec3d nz(n + cv::Vec3d(rng.gaussian(0.03), rng.gaussian(0.03),
                                 rng.gaussian(0.03)));
      nz /= cv::norm(nz);
      const cv::Vec3d pz(p[0] + rng.gaussian(8.0), p[1] + rng.gaussian(8.0),
                         p[2] + rng.gaussian(8.0));
      obs_list.push_back({pz, nz, 0.0});
    }
    // 随机保留 1~2 块
    std::shuffle(obs_list.begin(), obs_list.end(), gen);
    const int keep = std::min<int>(static_cast<int>(obs_list.size()),
                                   1 + static_cast<int>(rng.uniform(0, 2)));
    for (int k = 0; k < keep; ++k) {
      tracker.Update(obs_list[k], t);
    }

    if (i >= 100) {  // 预热后评估
      ++n_eval;
      center_err += cv::norm(tracker.center() - c_true);
      w_err += std::fabs(tracker.spin_rate_dps() - w_true * 180.0 / kPi);
      r_err += std::fabs(tracker.radius_mm() - R_true);
      // 预测误差：t+pred 时刻的板位置 vs 真值（最近匹配）
      const auto pred = tracker.Predict(t + pred_delay);
      const auto truth = true_plates(t + pred_delay);
      for (const auto& pc : pred) {
        double best = 1e18;
        for (const auto& tp : truth) {
          best = std::min(best, cv::norm(pc.center_mm - tp));
        }
        pred_err += best;
      }
    }
  }
  if (n_eval == 0) {
    std::printf("❌ 评估帧数为 0（模拟异常）\n");
    return 1;
  }
  center_err /= n_eval;
  w_err /= n_eval;
  r_err /= n_eval;
  pred_err /= (n_eval * 4);

  std::printf("中心误差: %.1f mm | 转速误差: %.1f °/s | 半径误差: %.1f mm | "
              "+40ms 预测板位置误差: %.1f mm\n",
              center_err, w_err, r_err, pred_err);
  const bool pass = center_err < 30.0 && w_err < 20.0 && r_err < 40.0 &&
                    pred_err < 40.0;
  std::printf("=== 反小陀螺自检%s ===\n", pass ? "通过 ✅" : "失败 ❌");
  return pass ? 0 : 1;
}

}  // namespace

namespace {

// 图传发布统计（每秒打印一次，判断"发不出去"还是"网络吞了"）
uint64_t g_stream_pub = 0;
uint64_t g_stream_bytes = 0;
uint64_t g_loop_count = 0;      // 主循环迭代次数（判断循环本身是不是被卡在 24Hz）

/// 外部命令（客户端 U 键）用的标志：主循环每拍取走一次
std::atomic<int> g_ext_cmd{-1};

/// 起一个 UDP 命令口线程：目前只支持 'u'（切 单板/反小陀螺）。
/// 为什么需要：27RM 用 --no-gui 后台跑时没有终端，按不了 u。
void startCmdListener(uint16_t port) {
  static std::atomic<bool> started{false};
  if (port == 0 || started.exchange(true)) return;

  std::thread([port] {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
      std::fprintf(stderr, "[cmd] socket 失败: %s\n", std::strerror(errno));
      return;
    }
    int one = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a{};
    a.sin_family      = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port        = htons(port);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) {
      std::fprintf(stderr, "[cmd] 绑定 UDP %u 失败: %s\n", port,
                   std::strerror(errno));
      ::close(fd);
      return;
    }
    std::printf("[cmd] 命令口 UDP %u 已开（'u'=切单板/反小陀螺  'c'=切蓝红  'r'/'b'=设为红/蓝）\n",
                port);
    for (;;) {
      char buf[64];
      sockaddr_in from{};
      socklen_t   flen = sizeof(from);
      const ssize_t n  = ::recvfrom(fd, buf, sizeof(buf), 0,
                                    reinterpret_cast<sockaddr*>(&from), &flen);
      if (n <= 0) continue;
      for (ssize_t i = 0; i < n; ++i) {
        const char c = buf[i];
        if (c == 'u' || c == 'U' || c == 'c' || c == 'C' ||
            c == 'r' || c == 'R' || c == 'b' || c == 'B') {
          g_ext_cmd.store(c == 'C' ? 'c' : (c == 'R' ? 'r' : (c == 'B' ? 'b' : c)));
        }
      }
      ::sendto(fd, "k", 1, 0, reinterpret_cast<sockaddr*>(&from), flen);  // 回执
    }
  }).detach();
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!ParseArgs(argc, argv, &opt)) {
    PrintUsage(argv[0]);
    return 2;
  }

  // 0) 反小陀螺合成自检（不需要相机/标定）
  if (opt.track_test) {
    return RunTrackTest();
  }

  // 0) 纯采集测速模式（不需要标定）
  if (opt.bench) {
    UsbCamera bcam;
    if (!opt.format.empty()) bcam.SetFormat(opt.format);
    if (!bcam.Open(opt.device, opt.width, opt.height, opt.runtime_path)) {
      return -1;
    }
    const int target = opt.frames > 0 ? opt.frames : 300;
    const auto t0 = std::chrono::steady_clock::now();
    uint64_t last = 0;
    int n = 0;
    auto last_frame_at = t0;
    while (n < target) {
      cv::Mat f;
      std::chrono::steady_clock::time_point ts;
      uint64_t seq = 0;
      const auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now - last_frame_at).count() > 2.0) {
        std::fprintf(stderr,
                     "\n⚠️ 2 秒收不到新帧，相机可能停流了（UVC 状态异常）\n"
                     "   处理：拔插摄像头，或重新枚举 USB 设备后再试一次\n");
        return -2;
      }
      if (!bcam.GetLatestFrame(&f, &ts, &seq)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      if (seq == last) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      last = seq;
      last_frame_at = now;
      ++n;
    }
    const double dt =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();
    std::printf("采集 %d 帧，用时 %.2fs → %.1f fps（相机实测窗口 %.1f fps）"
                " | %dx%d %s\n",
                n, dt, n / dt, bcam.MeasuredFps(), bcam.Width(), bcam.Height(),
                bcam.Fourcc().c_str());
    return 0;
  }

  // 1) 标定（必须）
  CameraCalib calib;
  if (!calib.load(opt.calib_path)) {
    std::fprintf(stderr,
                 "⚠️ 没有标定文件 %s。先做相机标定：\n"
                 "  ./capture_calib\n"
                 "  ./calibrate --cols 10 --rows 7 --square 15.0\n",
                 opt.calib_path.c_str());
    return -1;
  }

  // 2) 检测参数 + 装甲板尺寸 + 反小陀螺跟踪器
  rm27::LightBarDetector det(opt.lightbar_path);
  if (opt.color == "red") det.setEnemyColor("red");
  const ArmorSize armor = LoadArmorSize(opt.armor_path);
  rm27::TargetTracker tracker(opt.target_path);

  // 3) 相机
  UsbCamera cam;
  if (!opt.format.empty()) cam.SetFormat(opt.format);
  if (!cam.Open(opt.device, opt.width, opt.height, opt.runtime_path)) {
    return -1;
  }
  if (calib.imageSize() != cv::Size(cam.Width(), cam.Height())) {
    std::fprintf(stderr,
                 "⚠️ 标定分辨率 %dx%d 与当前采集 %dx%d 不一致！\n"
                 "   请在 %dx%d 下重新标定（./capture_calib + ./calibrate），"
                 "否则位姿会不准。\n",
                 calib.imageSize().width, calib.imageSize().height, cam.Width(),
                 cam.Height(), cam.Width(), cam.Height());
  }

  // 图传：MJPG 零转码转发；YUYV 降级时按需编码
  rm27::MjpegStreamer streamer(opt.stream_port, opt.stream_fps, opt.stream_bind);
  bool have_stream = false;
  if (opt.stream_port > 0) {
    have_stream = streamer.Start();
    if (have_stream) {
      std::printf(
          "图传: 控制端/浏览器打开 http://<jetson-ip>:%d/stream（%d fps）\n",
          opt.stream_port, opt.stream_fps);
    }
  }

  // 自瞄 UDP 输出（给控制端对接）
  startCmdListener(static_cast<uint16_t>(opt.cmd_port));   // 客户端 U 键用
  if (opt.stream_raw) cam.EnableRawJpeg(true);             // 图传直发原始 MJPEG

  rm27::AimUdpSender aim_sender;
  aim_sender.Start(opt.aim_udp);
  double last_aim_send = -1e9;

  // 闭环自瞄：27RM 直接给云台守护进程发角速度指令（驱动云台跟踪）
  rm27::GimbalCommandSender gimbal_sender;
  bool gimbal_aim_on = false;
  if (opt.gimbal_aim) {
    gimbal_aim_on = gimbal_sender.Start(opt.gimbal_host, opt.gimbal_port);
  }
  double last_gimbal_send = -1e9;

  if (!opt.no_gui && std::getenv("DISPLAY") == nullptr) {
    std::fprintf(stderr,
                 "❌ 没有检测到 DISPLAY：无图形环境。\n"
                 "   无头运行请加 --no-gui（只打印位姿）；\n"
                 "   要窗口就 ssh -Y 或先在 Jetson 桌面终端运行。\n");
    return 1;
  }
  const std::string win = "27RM";
  if (!opt.no_gui) {
    cv::namedWindow(win, cv::WINDOW_NORMAL);
  }

  PoseFilter filter;
  auto last_print = std::chrono::steady_clock::now();
  auto last_show = std::chrono::steady_clock::now();
  uint64_t last_seq = 0;
  uint64_t seen_seq = 0;
  uint64_t processed = 0;
  double detect_ms_ema = 0.0;
  double loop_fps_ema = 0.0;
  auto proc_start = std::chrono::steady_clock::now();
  auto last_proc_time = proc_start;
  const auto t0_steady = proc_start;

  std::printf("开始自瞄：q=退出 c=切颜色 s=截图 r=重载参数\n");

  while (true) {
    ++g_loop_count;
    cv::Mat frame;
    std::chrono::steady_clock::time_point ts;
    if (!cam.GetLatestFrame(&frame, &ts, &last_seq)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    // --save-frame：存一帧就走
    if (!opt.save_frame.empty()) {
      if (cv::imwrite(opt.save_frame, frame)) {
        std::printf("已保存 %s（%dx%d）\n", opt.save_frame.c_str(), frame.cols,
                    frame.rows);
      } else {
        std::fprintf(stderr, "保存 %s 失败\n", opt.save_frame.c_str());
      }
      break;
    }

    // 同一帧不重复处理（等新帧）
    if (last_seq == seen_seq) {
      if (!opt.no_gui) {
        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q') break;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      continue;
    }
    seen_seq = last_seq;

    const double t_now =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      t0_steady)
            .count();

    // ---- 检测（可能同帧看到 1~2 块板）+ 位姿 ----
    // 平时：PnP 锁「最近的装甲板」；按 U 切到反小陀螺模式（4 板环模型预测）
    const int64 t0 = cv::getTickCount();
    const auto targets = det.detectAll(frame);
    bool has_pose = false;
    ArmorPose best_pose;          // 最近的板
    rm27::ArmorTarget best_target;
    static double last_det_t = -1e9;
    static bool anti_spin = opt.anti_spin;  // 运行中按 U 切换
    // ---- 外部命令（客户端按键走 UDP 命令口）----
    switch (g_ext_cmd.exchange(-1)) {
      case 'u':                                   // 切 单板/反小陀螺
        anti_spin = !anti_spin;
        tracker.Reset();   // 切换模式后重新起跟，避免旧目标残留
        std::printf("[模式] 外部命令切换为 %s（%s）\n",
                    anti_spin ? "反小陀螺" : "单板锁定",
                    anti_spin ? "4 板环预测瞄准" : "PnP 锁最近的装甲板");
        break;
      case 'c': {                                 // 切 蓝 ↔ 红
        const std::string c =
            det.config().enemy_color == "blue" ? "red" : "blue";
        det.setEnemyColor(c);
        std::printf("[颜色] 外部命令切换为 %s（%s）\n", c.c_str(),
                    c == "red" ? "打红方" : "打蓝方");
        break;
      }
      case 'r':                                   // 直接设红
        det.setEnemyColor("red");
        std::printf("[颜色] 外部命令设为 red（打红方）\n");
        break;
      case 'b':                                   // 直接设蓝
        det.setEnemyColor("blue");
        std::printf("[颜色] 外部命令设为 blue（打蓝方）\n");
        break;
      default:
        break;
    }
    std::vector<ArmorPose> poses;
    for (const auto& target : targets) {
      if (auto p = calib.solvePose(target.corners, armor)) {
        poses.push_back(*p);
        last_det_t = t_now;
        if (!has_pose || p->distance_mm < best_pose.distance_mm) {
          has_pose = true;
          best_pose = *p;
          best_target = target;
        }
      }
    }
    // 反小陀螺模式才喂跟踪器；同时长时间无观测自动复位
    if (anti_spin) {
      for (const auto& p : poses) {
        tracker.Update(rm27::PlateObservation{p.tvec, PoseNormal(p.rvec),
                                              p.reproj_error_px},
                       t_now);
      }
      if (tracker.valid() && t_now - last_det_t > 0.8) {
        tracker.Reset();
      }
    }
    const double detect_ms =
        (cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency();
    detect_ms_ema = detect_ms_ema <= 0.0 ? detect_ms
                                         : 0.9 * detect_ms_ema + 0.1 * detect_ms;

    // ---- 图传发布速率（每秒一行）----
    {
      static double t_last_pub_log = -1e9;
      static uint64_t last_pub = 0, last_bytes = 0;
      static uint64_t last_loop = 0;
      if (t_now - t_last_pub_log >= 1.0) {
        const double dtl = t_now - t_last_pub_log;
        if (t_last_pub_log > 0 && have_stream) {
          const double fps_out = (g_stream_pub - last_pub) / dtl;
          const double kb_out =
              (last_pub == g_stream_pub) ? 0.0
              : static_cast<double>(g_stream_bytes - last_bytes) /
                    static_cast<double>(g_stream_pub - last_pub) / 1024.0;
          std::printf("[图传] 发布 %.1f fps  平均 %.1f KB/帧  采集 %.1f fps  主循环 %.1f Hz\n",
                      fps_out, kb_out, cam.MeasuredFps(),
                      static_cast<double>(g_loop_count - last_loop) / dtl);
        }
        t_last_pub_log = t_now;
        last_pub = g_stream_pub;
        last_bytes = g_stream_bytes;
        last_loop = g_loop_count;
      }
    }

    const auto now = std::chrono::steady_clock::now();
    const double dt =
        std::chrono::duration<double>(now - last_proc_time).count();
    last_proc_time = now;
    if (dt > 1e-6) {
      const double fps = 1.0 / dt;
      loop_fps_ema =
          loop_fps_ema <= 0.0 ? fps : 0.9 * loop_fps_ema + 0.1 * fps;
    }
    ++processed;

    const AimOutput& out =
        filter.Update(has_pose, best_pose.rvec, best_pose.tvec,
                      has_pose ? best_pose.reproj_error_px : 0.0);

    // ---- 瞄准选择 ----
    // 反小陀螺：跟踪器预测（t+前瞻）选板；平时：PnP 直接锁最近的板
    const double t_pred = t_now + tracker.config().predict_time_s;
    rm27::PredictedPlate aim_plate;
    bool has_aim = false;
    double aim_yaw = 0.0, aim_pitch = 0.0;
    bool aim_facing = false;
    if (anti_spin && tracker.SelectAimPlate(t_pred, &aim_plate)) {
      has_aim = true;
      CameraCalib::poseToYawPitch(aim_plate.center_mm, &aim_yaw, &aim_pitch);
      aim_facing = aim_plate.face_score > 0.3;
    } else if (has_pose) {
      has_aim = true;
      CameraCalib::poseToYawPitch(best_pose.tvec, &aim_yaw, &aim_pitch);
      const cv::Vec3d n = PoseNormal(best_pose.rvec);
      const double d = std::max(1e-6, cv::norm(best_pose.tvec));
      aim_facing =
          -(n[0] * best_pose.tvec[0] + n[2] * best_pose.tvec[2]) / d > 0.3;
    }

    // ---- 闭环自瞄：直接驱动云台跟踪（50Hz）----
    if (gimbal_aim_on && t_now - last_gimbal_send >= 0.02) {
      last_gimbal_send = t_now;
      double yaw_cmd = 0.0, pitch_cmd = 0.0;
      if (has_aim) {
        const double yerr = aim_yaw * 180.0 / kPi;   // 目标偏角（°）
        const double perr = aim_pitch * 180.0 / kPi;
        double y = opt.gimbal_kp * (opt.gimbal_invert_yaw ? -yerr : yerr);
        double p = opt.gimbal_kp * (opt.gimbal_invert_pitch ? -perr : perr);
        if (std::fabs(yerr) > opt.gimbal_deadband) {
          yaw_cmd = std::max(-opt.gimbal_yaw_max,
                             std::min(opt.gimbal_yaw_max, y));
        }
        if (std::fabs(perr) > opt.gimbal_deadband) {
          pitch_cmd = std::max(-opt.gimbal_pitch_max,
                               std::min(opt.gimbal_pitch_max, p));
        }
      }
      gimbal_sender.Send(yaw_cmd, pitch_cmd, true);
    }

    // ---- 自瞄 UDP 输出（50Hz）----
    if (aim_sender.enabled() && t_now - last_aim_send >= 0.02) {
      last_aim_send = t_now;
      rm27::AimPacket pkt{};
      pkt.valid = (has_pose || (anti_spin && tracker.valid())) ? 1 : 0;
      pkt.flags = 0;
      if (anti_spin && tracker.valid()) pkt.flags |= 0x01;  // 反小陀螺跟踪中
      if (has_aim && aim_facing) pkt.flags |= 0x02;         // 板面朝向相机
      pkt.plate_slot = (has_aim && anti_spin)
                           ? static_cast<uint8_t>(aim_plate.slot)
                           : 0;
      // reserved 字节：告诉控制端当前敌方颜色（1=蓝 2=红），老客户端忽略即可
      pkt.reserved =
          (det.config().enemy_color == "red") ? static_cast<uint8_t>(2)
                                              : static_cast<uint8_t>(1);
      if (has_aim) {
        pkt.aim_yaw_deg = static_cast<float>(aim_yaw * 180.0 / kPi);
        pkt.aim_pitch_deg = static_cast<float>(aim_pitch * 180.0 / kPi);
      } else if (out.valid) {
        pkt.aim_yaw_deg = static_cast<float>(out.yaw_deg);
        pkt.aim_pitch_deg = static_cast<float>(out.pitch_deg);
      }
      if (tracker.valid()) {
        pkt.spin_rate_dps = static_cast<float>(tracker.spin_rate_dps());
        pkt.center_x_mm = static_cast<int32_t>(tracker.center()[0]);
        pkt.center_y_mm = static_cast<int32_t>(tracker.center()[1]);
        pkt.center_z_mm = static_cast<int32_t>(tracker.center()[2]);
        pkt.radius_mm = static_cast<int32_t>(tracker.radius_mm());
      }
      pkt.dist_m = static_cast<float>(out.distance_m);
      aim_sender.Send(&pkt);
    }

    // ---- 终端打印（限速 10Hz）----
    const double since_print =
        std::chrono::duration<double>(now - last_print).count();
    if (since_print >= 0.1) {
      last_print = now;
      const char* mode_name = anti_spin ? "反小陀螺" : "单板";
      if (out.measured) {
        std::printf(
            "[%s] 距离 %.3fm | yaw %+.2f° | pitch %+.2f° | 位置(%+.0f,%+.0f,%+.0f)mm"
            " | 板偏角 %.1f° | 重投影 %.2fpx | 检测 %.1fms | 采集 %.0ffps | 处理 %.0ffps | 模式:%s\n",
            StateName(out), out.distance_m, out.yaw_deg, out.pitch_deg,
            out.tvec[0], out.tvec[1], out.tvec[2], out.plate_tilt_deg,
            out.reproj_error_px, detect_ms_ema, cam.MeasuredFps(),
            loop_fps_ema, mode_name);
      } else {
        std::printf("[%s] 无目标（灯条候选 %zu）| 检测 %.1fms | 采集 %.0ffps | 模式:%s\n",
                    StateName(out), det.bars().size(), detect_ms_ema,
                    cam.MeasuredFps(), mode_name);
      }
      if (anti_spin && tracker.valid()) {
        const cv::Vec3d c = tracker.center();
        std::printf(
            "  [反小陀螺] 中心(%+.0f,%+.0f,%+.0f)mm R=%.0fmm 转速%+.0f°/s | "
            "预测瞄准[板%d] yaw%+.2f° pitch%+.2f° (t+%.0fms)\n",
            c[0], c[1], c[2], tracker.radius_mm(), tracker.spin_rate_dps(),
            aim_plate.slot, aim_yaw * 180.0 / kPi, aim_pitch * 180.0 / kPi,
            tracker.config().predict_time_s * 1000.0);
      }
      std::fflush(stdout);
    }

    // ---- 画面覆盖层（GUI 与图传共用）：检测框 / 预测环 / 瞄准文字 ----
    const bool need_view =
        (!opt.no_gui) ||
        (have_stream && !opt.stream_raw && streamer.client_count() > 0);
    cv::Mat view;
    if (need_view) {
      view = frame.clone();
      std::optional<rm27::ArmorTarget> best_opt;
      if (has_pose) best_opt = best_target;
      rm27::LightBarDetector::drawDebug(view, det.bars(), best_opt);

      // 预测的整圈板位置（绿点 = 朝向相机）+ 环（仅反小陀螺模式）
      if (anti_spin && tracker.valid()) {
        const auto plates = tracker.Predict(t_pred);
        std::vector<cv::Point3f> ring3d;
        const cv::Vec3d cc = tracker.center();
        for (int i = 0; i <= 32; ++i) {
          const double a = 2.0 * kPi * i / 32.0;
          ring3d.emplace_back(
              static_cast<float>(cc[0] + tracker.radius_mm() * std::sin(a)),
              static_cast<float>(cc[1]),
              static_cast<float>(cc[2] + tracker.radius_mm() * std::cos(a)));
        }
        std::vector<cv::Point2f> ring2d;
        cv::projectPoints(ring3d, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0),
                          calib.K(), calib.D(), ring2d);
        std::vector<cv::Point> poly;
        for (const auto& p : ring2d) poly.emplace_back(p);
        if (poly.size() > 2) {
          cv::polylines(view, std::vector<std::vector<cv::Point>>{poly},
                        false, cv::Scalar(0, 160, 255), 1);
        }
        for (const auto& pl : plates) {
          const cv::Point3f p3(static_cast<float>(pl.center_mm[0]),
                               static_cast<float>(pl.center_mm[1]),
                               static_cast<float>(pl.center_mm[2]));
          std::vector<cv::Point2f> img;
          cv::projectPoints(std::vector<cv::Point3f>{p3}, cv::Vec3d(0, 0, 0),
                            cv::Vec3d(0, 0, 0), calib.K(), calib.D(), img);
          if (!img.empty()) {
            const bool facing = pl.face_score > 0.3;
            cv::circle(view, img[0], 6,
                       facing ? cv::Scalar(0, 255, 0)
                              : cv::Scalar(0, 160, 255),
                       -1);
          }
        }
      }

      // 画面中心十字：**必须按实际分辨率算**（以前写死 320/240 = 640x480 的中心，
      // 相机是 800x600 时偏 (-80,-60) 像素，看起来就像"有两个准星、自瞄锁偏了"）
      {
        const int ccx = view.cols / 2;
        const int ccy = view.rows / 2;
        const int cr  = 12;
        cv::line(view, cv::Point(ccx - cr, ccy), cv::Point(ccx + cr, ccy),
                 cv::Scalar(0, 128, 255), 1);
        cv::line(view, cv::Point(ccx, ccy - cr), cv::Point(ccx, ccy + cr),
                 cv::Scalar(0, 128, 255), 1);
      }
      char info[256];
      const char* mode_str = anti_spin ? "反小陀螺" : "单板";
      if (has_aim && anti_spin) {
        std::snprintf(
            info, sizeof(info),
            "%s [%s] D=%.2fm yaw=%+.1f pitch=%+.1f f=%.0ffps R=%.0f w=%+.0fdps",
            StateName(out), mode_str, out.distance_m, out.yaw_deg,
            out.pitch_deg, cam.MeasuredFps(), tracker.radius_mm(),
            tracker.spin_rate_dps());
      } else if (has_aim) {
        std::snprintf(info, sizeof(info),
                      "%s [%s] D=%.2fm yaw=%+.1f pitch=%+.1f f=%.0ffps",
                      StateName(out), mode_str, out.distance_m, out.yaw_deg,
                      out.pitch_deg, cam.MeasuredFps());
      } else {
        std::snprintf(info, sizeof(info), "%s [%s] no target f=%.0ffps",
                      StateName(out), mode_str, cam.MeasuredFps());
      }
      cv::putText(view, info, cv::Point(10, 26), cv::FONT_HERSHEY_SIMPLEX,
                  0.55,
                  out.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255),
                  1);
    }

    // ---- 图传（带每秒发布统计，方便现场确认是不是发不出去）----
    if (have_stream && opt.stream_raw) {
      // 直发相机原始 MJPEG：零转码、无叠加框，CPU 几乎不花（按 --stream-fps 限速）
      // 每帧都交给服务器；它内部按 stream_fps 挑最新帧发给客户端
      // （以前在这里按"每帧"检查 1/fps 的闸门，只能按帧周期整数倍触发，
      //   30fps 目标会被量化成 40ms 一帧 → 实测只有 23.8 fps）
      static uint64_t raw_seq = 0;
      static std::vector<uint8_t> raw;
      uint64_t seq = 0;
      if (cam.GetLatestRawJpeg(&raw, &seq) && seq != raw_seq) {
        raw_seq = seq;
        streamer.PublishJpeg(raw);
        g_stream_pub += 1;
        g_stream_bytes += raw.size();
      }
    } else if (have_stream && !view.empty()) {
      // 同上：每帧都编码发布，服务器按 stream_fps 挑最新帧
      {
        std::vector<uint8_t> enc_jpg;
        const cv::Mat* to_encode = &view;
        cv::Mat scaled;
        if (opt.stream_scale < 0.999) {
          cv::resize(view, scaled, cv::Size(), opt.stream_scale,
                     opt.stream_scale, cv::INTER_AREA);
          to_encode = &scaled;
        }
        cv::imencode(".jpg", *to_encode, enc_jpg,
                     {cv::IMWRITE_JPEG_QUALITY, opt.stream_quality});
        streamer.PublishJpeg(enc_jpg);
        g_stream_pub += 1;
        g_stream_bytes += enc_jpg.size();
        (void)t_now;
      }
    }

    // ---- GUI 显示 + 按键（限速 ~25Hz）----
    if (!opt.no_gui && !view.empty()) {
      const double since_show =
          std::chrono::duration<double>(now - last_show).count();
      if (since_show >= 0.04) {
        last_show = now;
        cv::imshow(win, view);
        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q') break;
        if (key == 'c') {
          const std::string c =
              det.config().enemy_color == "blue" ? "red" : "blue";
          det.setEnemyColor(c);
          std::printf("敌方颜色切换为 %s\n", c.c_str());
        }
        if (key == 'r') {
          det.loadConfig(opt.lightbar_path);
        }
        if (key == 'u') {
          anti_spin = !anti_spin;
          tracker.Reset();  // 切换模式后重新起跟，避免旧目标残留
          std::printf("[模式] 切换为 %s（%s）\n",
                      anti_spin ? "反小陀螺" : "单板锁定",
                      anti_spin ? "4 板环预测瞄准" : "PnP 锁最近的装甲板");
        }
        if (key == 's') {
          char name[64];
          std::snprintf(name, sizeof(name), "aim_%04u.jpg",
                        static_cast<unsigned>(processed));
          cv::imwrite(name, view);
          std::printf("已保存 %s\n", name);
        }
      }
    }

    if (opt.frames > 0 && processed >= static_cast<uint64_t>(opt.frames)) {
      break;
    }
  }

  if (gimbal_aim_on) {
    gimbal_sender.Send(0.0, 0.0, false);  // 退出前失能云台
  }

  const double total =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    proc_start)
          .count();
  std::printf("\n运行结束：处理 %llu 帧，用时 %.2fs（平均 %.1f fps）\n",
              static_cast<unsigned long long>(processed), total,
              processed / total);
  return 0;
}
