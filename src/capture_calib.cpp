/**
 * @file  capture_calib.cpp
 * @brief 相机标定图采集工具 —— 27RM 校内赛视觉（USB 摄像头）
 *
 * 流程：跑本工具 → 举棋盘格在画面里上下左右/远近/倾斜移动 →
 *       检测到就保存 → 攒 20~30 张 → 跑 ./calibrate 算内参。
 *
 * 模式：
 *   ./capture_calib                        图形界面（需要显示器）
 *   ./capture_calib --headless             无显示器：检测到就自动存
 *
 * 按键（图形界面）：
 *   空格 保存当前帧（默认要求检测到棋盘格） | f 强制保存 | d 删除上一张
 *   c 开关角点显示 | g 灰度预览 | q/ESC 退出
 */

#include "camera_usb.hpp"

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

#include <sys/stat.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Options {
  int device = 0;
  int cols = 10;            // 内角点列数（11x8 方格板）
  int rows = 7;             // 内角点行数
  float square_mm = 15.0f;  // 方格边长（打印后卡尺实测！）
  std::string dir = "calib_images";
  int width = 800;
  int height = 600;
  std::string format;  // "mjpg" / "yuyv" / 空=MJPG优先
  int fps = 120;            // 仅占位（帧率由相机自己跑 ≈99）
  bool headless = false;
  int interval_ms = 700;    // 无头模式：间隔多久尝试存一张
  int max_images = 40;      // 无头模式：最多存多少张
};

void PrintUsage(const char* prog) {
  std::printf(
      "用法: %s [选项]\n"
      "  --device N        /dev/videoN（默认 0）\n"
      "  --cols N          内角点列数（默认 10）\n"
      "  --rows N          内角点行数（默认 7）\n"
      "  --square F        方格边长 mm（默认 15.0，以卡尺实测为准）\n"
      "  --dir PATH        保存目录（默认 calib_images）\n"
      "  --format mjpg|yuyv 采集格式（默认 MJPG 优先；相机 MJPG 坏了时可用 yuyv 预览）\n"
      "  分辨率默认 800x600（--width/--height 可改）\n"
      "  --headless        无显示器模式：检测到棋盘格自动保存\n"
      "  --interval-ms N   无头模式保存间隔（默认 700ms）\n"
      "  --max N           无头模式最多保存张数（默认 40）\n",
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
    if (a == "--device") {
      opt->device = std::atoi(next("--device"));
    } else if (a == "--cols") {
      opt->cols = std::atoi(next("--cols"));
    } else if (a == "--rows") {
      opt->rows = std::atoi(next("--rows"));
    } else if (a == "--square") {
      opt->square_mm = static_cast<float>(std::atof(next("--square")));
    } else if (a == "--dir") {
      opt->dir = next("--dir");
    } else if (a == "--format") {
      opt->format = next("--format");
    } else if (a == "--headless") {
      opt->headless = true;
    } else if (a == "--interval-ms") {
      opt->interval_ms = std::atoi(next("--interval-ms"));
    } else if (a == "--max") {
      opt->max_images = std::atoi(next("--max"));
    } else if (a == "-h" || a == "--help") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      std::fprintf(stderr, "未知参数: %s\n", a.c_str());
      PrintUsage(argv[0]);
      return false;
    }
  }
  return true;
}

void Mkdirs(const std::string& path) {
  std::string cur;
  for (std::size_t i = 0; i <= path.size(); ++i) {
    if (i == path.size() || path[i] == '/') {
      if (!cur.empty()) {
        ::mkdir(cur.c_str(), 0755);
      }
    }
    if (i < path.size()) cur.push_back(path[i]);
  }
}

// 目录里已有多少张图（用于续号）
int CountExisting(const std::string& dir) {
  int n = 0;
  for (int i = 0; i < 1000; ++i) {
    char name[512];
    std::snprintf(name, sizeof(name), "%s/img_%03d.png", dir.c_str(), i);
    struct stat st;
    if (::stat(name, &st) == 0) {
      ++n;
    }
  }
  return n;
}

bool SaveImage(const std::string& dir, int* index, const cv::Mat& img) {
  char name[512];
  std::snprintf(name, sizeof(name), "%s/img_%03d.png", dir.c_str(), *index);
  if (!cv::imwrite(name, img)) {
    std::fprintf(stderr, "保存失败: %s\n", name);
    return false;
  }
  std::printf("已保存 %s（第 %d 张）\n", name, *index + 1);
  ++(*index);
  return true;
}

bool FindBoard(const cv::Mat& gray, int cols, int rows,
               std::vector<cv::Point2f>* corners) {
  // ⚠️ 经典检测器为主：Jetson 的 OpenCV 4.8 里 findChessboardCornersSB
  //    实测 800x600 要 381ms/帧（还会抛 getNeighbor 断言），完全没法用来预览；
  //    经典 findChessboardCorners 只要 ~8.5ms。
  std::vector<cv::Point2f> pts;
  const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE |
                    cv::CALIB_CB_FAST_CHECK;
  if (cv::findChessboardCorners(gray, cv::Size(cols, rows), pts, flags)) {
    *corners = pts;
    return true;
  }
  // 经典失败时再试一次 SB（某些模糊场景 SB 更稳），并吞掉异常
  pts.clear();
  try {
    if (cv::findChessboardCornersSB(gray, cv::Size(cols, rows), pts,
                                    cv::CALIB_CB_NORMALIZE_IMAGE)) {
      *corners = pts;
      return true;
    }
  } catch (const cv::Exception&) {
  }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!ParseArgs(argc, argv, &opt)) {
    return 2;
  }
  std::printf(
      "=== 27RM 标定图采集 ===\n"
      "棋盘格: %dx%d 内角点, 方格 %.1fmm（用卡尺实测打印结果后以此为准）\n",
      opt.cols, opt.rows, opt.square_mm);

  Mkdirs(opt.dir);
  int index = CountExisting(opt.dir);
  if (index > 0) {
    std::printf("目录 %s 已有 %d 张，继续从第 %d 张编号\n", opt.dir.c_str(),
                index, index + 1);
  }

  // 标定阶段用自动曝光（板子亮度均匀更容易检角点）。
  // UVC 的 exposure_auto：1=手动、3=自动（光圈优先），0/2 会 EINVAL。
  // 必须在开始取流之前设，流中改控制项会弄僵相机。
  UsbCamera::SetControl("/dev/video" + std::to_string(opt.device),
                        V4L2_CID_EXPOSURE_AUTO, 3);
  if (!opt.headless && std::getenv("DISPLAY") == nullptr) {
    std::fprintf(stderr,
                 "❌ 没有检测到 DISPLAY：当前终端没有图形环境。\n"
                 "   有外接屏：在 Jetson 桌面上运行；\n"
                 "   远程：用 ssh -Y 连过来再运行（或先 export DISPLAY=:0）；\n"
                 "   无头：加 --headless 自动保存（不需要显示器）\n");
    return 1;
  }

  UsbCamera cam;
  if (!cam.Open(opt.device, opt.width, opt.height, "")) {
    return -1;
  }

  const std::string win = "capture_calib";
  if (!opt.headless) {
    cv::namedWindow(win, cv::WINDOW_NORMAL);
  }

  bool show_corners = true;
  bool show_gray = false;
  auto last_tick = std::chrono::steady_clock::now();
  int saved_in_headless = 0;

  bool warned_opencv = false;
  auto loop_t0 = std::chrono::steady_clock::now();
  int loop_frames = 0;
  double loop_fps = 0.0;
  auto last_detect = std::chrono::steady_clock::now();
  std::vector<cv::Point2f> last_corners;
  bool last_found = false;
  while (true) {
    cv::Mat frame;
    std::chrono::steady_clock::time_point ts;
    if (!cam.GetLatestFrame(&frame, &ts)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    bool found = false;
    try {
      found = FindBoard(gray, opt.cols, opt.rows, &corners);
    } catch (const cv::Exception& e) {
      if (!warned_opencv) {
        std::fprintf(stderr,
                     "[capture_calib] ⚠️ 本帧检测抛 OpenCV 异常已忽略（进入下帧）：%s\n",
                     e.what());
        warned_opencv = true;
      }
      continue;
    }
    cv::Mat view = frame.clone();
    if (found && show_corners) {
      cv::drawChessboardCorners(view, cv::Size(opt.cols, opt.rows), corners,
                                found);
    }
    // 循环帧率（窗口实际刷新率）
    ++loop_frames;
    const auto loop_now = std::chrono::steady_clock::now();
    const double loop_dt =
        std::chrono::duration<double>(loop_now - loop_t0).count();
    if (loop_dt >= 1.0) {
      loop_fps = loop_frames / loop_dt;
      loop_frames = 0;
      loop_t0 = loop_now;
      std::printf("采集 %.0ffps | 窗口 %.0ffps | 检测状态 %s\n",
                  cam.MeasuredFps(), loop_fps,
                  found ? "检测到棋盘格" : "无");
      std::fflush(stdout);
    }
    char status[160];
    std::snprintf(status, sizeof(status),
                  "%s  cam %.0ffps  win %.0ffps  SPACE=save f=force d=del q=quit",
                  found ? "CHESSBOARD OK" : "no chessboard",
                  cam.MeasuredFps(), loop_fps);
    cv::putText(view, status, cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX,
                0.55, found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255),
                1);

    if (opt.headless) {
      // 无头：检测到就按间隔存图
      const auto now = std::chrono::steady_clock::now();
      const double dt =
          std::chrono::duration<double>(now - last_tick).count();
      if (found && dt * 1000.0 >= opt.interval_ms) {
        last_tick = now;
        if (SaveImage(opt.dir, &index, frame)) {
          ++saved_in_headless;
        }
        if (saved_in_headless >= opt.max_images) {
          std::printf("已达 %d 张上限，退出\n", opt.max_images);
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } else {
      cv::imshow(win, show_gray ? gray : view);
      const int key = cv::waitKey(10) & 0xFF;
      if (key == 27 || key == 'q') break;
      if (key == ' ' && found) {
        SaveImage(opt.dir, &index, frame);
      } else if (key == 'f') {
        SaveImage(opt.dir, &index, frame);
      } else if (key == 'd' && index > 0) {
        --index;
        char name[512];
        std::snprintf(name, sizeof(name), "%s/img_%03d.png", opt.dir.c_str(),
                      index);
        std::remove(name);
        std::printf("已删除 %s\n", name);
      } else if (key == 'c') {
        show_corners = !show_corners;
      } else if (key == 'g') {
        show_gray = !show_gray;
      }
    }
  }

  std::printf("\n共保存 %d 张标定图到 %s/\n下一步：\n"
              "  ./calibrate --dir %s --cols %d --rows %d --square %.2f\n",
              index, opt.dir.c_str(), opt.dir.c_str(), opt.cols, opt.rows,
              opt.square_mm);
  return 0;
}
