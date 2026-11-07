/**
 * @file  main.cpp
 * @brief 27RM 自动瞄准视觉主程序（校内赛版）
 *
 * 功能：USB 摄像头（640x480 MJPG，实测约 99fps）→ 灯条检测 → 装甲板 PnP 位姿
 *       → 平滑滤波 → 终端输出 + 调试可视化。
 *
 * 输出内容：距离（m）、yaw/pitch（度，相机系）、装甲板中心坐标（mm）、
 *          板面法线与视线夹角（度）、重投影误差（px）、处理耗时与采集帧率。
 *
 * 用法：
 *   ./27RM                          相机实时（需要显示器）
 *   ./27RM --no-gui                 无窗口，只打印（适合 ssh 跑）
 *   ./27RM --frames 500             无窗口跑 500 帧后自动退出（测速用）
 *   ./27RM --device 1               用 /dev/video1
 *   ./27RM --color red              检测红色灯条（默认蓝=打敌方蓝方）
 */

#include "aim_filter.hpp"
#include "camera_calib.hpp"
#include "camera_usb.hpp"
#include "light_bar.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

namespace {

struct Options {
  std::string calib_path = "configs/camera.yaml";
  std::string lightbar_path = "configs/light_bar.yaml";
  std::string armor_path = "configs/armor.yaml";
  std::string runtime_path = "configs/camera_runtime.yaml";
  int device = 0;
  int width = 800;
  int height = 600;
  std::string format;  // "mjpg"/"yuyv"/空（默认 MJPG 优先）
  bool no_gui = false;
  bool bench = false;          // --bench：只测采集帧率
  int frames = 0;  // >0：跑完这么多帧自动退出
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
      "  --device N        /dev/videoN（默认 0）\n"
      "  --width/--height  采集分辨率（默认 800x600，MJPG）\n"
      "  --format mjpg|yuyv 采集格式（默认 MJPG 优先；相机 MJPG 坏了时用 yuyv）\n"
      "  --no-gui          不开窗口（无显示器时用）\n"
      "  --bench           只测采集帧率后退出（不用标定/检测）\n"
      "  --frames N        采集 N 帧后退出（测帧率/自动化用）\n"
      "  --color blue/red  敌方颜色（默认 blue）\n"
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
    } else if (a == "--frames") {
      opt->frames = std::atoi(next("--frames"));
    } else if (a == "--color") {
      opt->color = next("--color");
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

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!ParseArgs(argc, argv, &opt)) {
    PrintUsage(argv[0]);
    return 2;
  }

  // 0) 纯采集测速模式（不需要标定）
  if (opt.bench) {
    UsbCamera bcam;
    if (!opt.format.empty()) bcam.SetFormat(opt.format);
    if (!bcam.Open(opt.device, opt.width, opt.height, opt.runtime_path)) return -1;
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
      // 防挂死：2 秒没新帧就报错退出（别让进程无限占着相机）
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
    const double dt = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - t0)
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

  // 2) 检测参数 + 装甲板尺寸
  rm27::LightBarDetector det(opt.lightbar_path);
  if (opt.color == "red") det.setEnemyColor("red");
  const ArmorSize armor = LoadArmorSize(opt.armor_path);

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

  std::printf("开始自瞄：q=退出 c=切颜色 s=截图 r=重载参数\n");

  while (true) {
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

    // ---- 检测 + 位姿 ----
    const int64 t0 = cv::getTickCount();
    auto target = det.detect(frame);
    bool has_pose = false;
    ArmorPose pose;
    if (target) {
      if (auto p = calib.solvePose(target->corners, armor)) {
        pose = *p;
        has_pose = true;
      }
    }
    const double detect_ms =
        (cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency();
    detect_ms_ema = detect_ms_ema <= 0.0 ? detect_ms
                                         : 0.9 * detect_ms_ema + 0.1 * detect_ms;

    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_proc_time).count();
    last_proc_time = now;
    if (dt > 1e-6) {
      const double fps = 1.0 / dt;
      loop_fps_ema =
          loop_fps_ema <= 0.0 ? fps : 0.9 * loop_fps_ema + 0.1 * fps;
    }
    ++processed;

    const AimOutput& out =
        filter.Update(has_pose, pose.rvec, pose.tvec, pose.reproj_error_px);

    // ---- 终端打印（限速 10Hz）----
    const double since_print =
        std::chrono::duration<double>(now - last_print).count();
    if (since_print >= 0.1) {
      last_print = now;
      if (out.measured) {
        std::printf(
            "[%s] 距离 %.3fm | yaw %+.2f° | pitch %+.2f° | 位置(%+.0f,%+.0f,%+.0f)mm"
            " | 板偏角 %.1f° | 重投影 %.2fpx | 检测 %.1fms | 采集 %.0ffps | 处理 %.0ffps\n",
            StateName(out), out.distance_m, out.yaw_deg, out.pitch_deg,
            out.tvec[0], out.tvec[1], out.tvec[2], out.plate_tilt_deg,
            out.reproj_error_px, detect_ms_ema, cam.MeasuredFps(),
            loop_fps_ema);
      } else {
        std::printf("[%s] 无目标（灯条候选 %zu）| 检测 %.1fms | 采集 %.0ffps\n",
                    StateName(out), det.bars().size(), detect_ms_ema,
                    cam.MeasuredFps());
      }
      std::fflush(stdout);
    }

    // ---- 可视化（限速 ~25Hz，别拖慢采集）----
    if (!opt.no_gui) {
      const double since_show =
          std::chrono::duration<double>(now - last_show).count();
      if (since_show >= 0.04) {
        last_show = now;
        cv::Mat view = frame.clone();
        rm27::LightBarDetector::drawDebug(view, det.bars(), target);
        cv::line(view, cv::Point(320 - 12, 240), cv::Point(320 + 12, 240),
                 cv::Scalar(0, 128, 255), 1);
        cv::line(view, cv::Point(320, 240 - 12), cv::Point(320, 240 + 12),
                 cv::Scalar(0, 128, 255), 1);
        char info[256];
        if (out.measured) {
          std::snprintf(
              info, sizeof(info),
              "%s D=%.2fm yaw=%+.1f pitch=%+.1f f=%.0ffps",
              StateName(out), out.distance_m, out.yaw_deg, out.pitch_deg,
              cam.MeasuredFps());
        } else {
          std::snprintf(info, sizeof(info), "%s no target f=%.0ffps",
                        StateName(out), cam.MeasuredFps());
        }
        cv::putText(view, info, cv::Point(10, 26), cv::FONT_HERSHEY_SIMPLEX,
                    0.55, out.valid ? cv::Scalar(0, 255, 0)
                                    : cv::Scalar(0, 200, 255),
                    1);
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

  const double total =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    proc_start)
          .count();
  std::printf("\n运行结束：处理 %llu 帧，用时 %.2fs（平均 %.1f fps）\n",
              static_cast<unsigned long long>(processed), total,
              processed / total);
  return 0;
}
