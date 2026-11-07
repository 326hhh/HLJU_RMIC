/**
 * @file  detect_test.cpp
 * @brief 灯条检测器验证工具（27RM 校内赛 - 小装甲板）
 *
 *  检测 → 4 角点 → PnP → 距离/yaw/pitch，一条龙验证。
 *
 * 模式：
 *   ./detect_test                          相机实时（c 切敌方颜色、r 重载参数）
 *   ./detect_test --image 照片.jpg         静态图交互
 *   ./detect_test --image 照片.jpg --once  跑一次就退出（脚本用）
 *   ./detect_test --selftest               合成图自检：已知位姿渲染 → 检测 → PnP
 */

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
#include <vector>

namespace {

// ---------------------------------------------------------------
// 合成渲染：按标定内参把已知位姿的装甲板画成发光灯条图
// ---------------------------------------------------------------
/** 按真实 3D 尺寸画一根灯条：条宽 bar_width_mm（毫米），
 *  端点严格落在灯条两端（圆管端弧会超出去）。 */
void DrawBar3D(cv::Mat& img, const cv::Mat& K, const cv::Mat& D,
               const cv::Vec3d& rvec, const cv::Vec3d& tvec, float y_center,
               float bar_width_mm, float bar_len_mm, const cv::Scalar& color) {
  const float hw = bar_width_mm / 2.f;
  const float hl = bar_len_mm / 2.f;
  const std::vector<cv::Point3f> quad{
      {0.f, y_center - hw, -hl}, {0.f, y_center - hw, hl},
      {0.f, y_center + hw, hl},  {0.f, y_center + hw, -hl}};
  std::vector<cv::Point2f> pts;
  cv::projectPoints(quad, rvec, tvec, K, D, pts);
  std::vector<cv::Point> poly;
  poly.reserve(4);
  for (const auto& p : pts) {
    poly.emplace_back(cvRound(p.x), cvRound(p.y));
  }
  cv::fillConvexPoly(img, poly, color);
}

cv::Mat RenderArmor(const cv::Mat& K, const cv::Mat& D,
                    const cv::Size& img_size, const cv::Vec3d& rvec,
                    const cv::Vec3d& tvec, const ArmorSize& size,
                    const std::string& color) {
  const double W = size.width_mm, L = size.height_mm;
  const std::vector<cv::Point3f> obj{
      {0.f, static_cast<float>(W / 2), static_cast<float>(-L / 2)},
      {0.f, static_cast<float>(W / 2), static_cast<float>(L / 2)},
      {0.f, static_cast<float>(-W / 2), static_cast<float>(L / 2)},
      {0.f, static_cast<float>(-W / 2), static_cast<float>(-L / 2)},
  };
  std::vector<cv::Point2f> pts;
  cv::projectPoints(obj, rvec, tvec, K, D, pts);

  cv::Mat img(img_size, CV_8UC3, cv::Scalar(10, 12, 16));
  cv::Mat noise(img_size, CV_8UC3);
  cv::randn(noise, cv::Scalar(0, 0, 0), cv::Scalar(6, 6, 6));
  img += noise;

  const cv::Scalar c = (color == "blue") ? cv::Scalar(230, 90, 30)
                                         : cv::Scalar(30, 60, 240);
  // 灯条按真实宽度 5.85mm 投影绘制（左条 y=+W/2，右条 y=-W/2）
  DrawBar3D(img, K, D, rvec, tvec, static_cast<float>(W / 2),
            static_cast<float>(size.bar_width_mm), static_cast<float>(L), c);
  DrawBar3D(img, K, D, rvec, tvec, static_cast<float>(-W / 2),
            static_cast<float>(size.bar_width_mm), static_cast<float>(L), c);

  // 干扰：两个小亮斑（检测器应忽略）
  cv::circle(img, cv::Point(160, 180), 4, cv::Scalar(180, 180, 180), -1);
  cv::circle(img, cv::Point(480, 400), 5, cv::Scalar(200, 200, 200), -1);

  cv::GaussianBlur(img, img, cv::Size(3, 3), 0.8);
  return img;
}

// ---------------------------------------------------------------
// 合成自检
// ---------------------------------------------------------------
int RunDetectSelftest(const cv::Mat& K, const cv::Mat& D,
                      const cv::Size& img_size, const ArmorSize& size,
                      rm27::LightBarDetector& det) {
  std::printf("=== 检测器合成自检 ===\n");
  std::printf("渲染已知位姿的发光装甲板 → 灯条检测 → PnP → 与真值对比\n\n");

  auto tvec_of = [](double yaw_deg, double pitch_deg, double d) {
    const double y = yaw_deg * CV_PI / 180.0,
                 p = pitch_deg * CV_PI / 180.0;
    return cv::Vec3d(d * std::sin(y) * std::cos(p), -d * std::sin(p),
                     d * std::cos(y) * std::cos(p));
  };
  auto rvec_of = [](double roll_deg, double yaw_deg, double pitch_deg) {
    const double r = roll_deg * CV_PI / 180.0,
                 y = yaw_deg * CV_PI / 180.0,
                 p = pitch_deg * CV_PI / 180.0;
    cv::Mat R = (cv::Mat_<double>(3, 3) << 0, 0, 1, 0, 1, 0, -1, 0, 0);
    R = R * (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 0, -1, 0, 1, 0);
    cv::Mat Rz = (cv::Mat_<double>(3, 3) << std::cos(r), -std::sin(r), 0,
                  std::sin(r), std::cos(r), 0, 0, 0, 1);
    cv::Mat Ry = (cv::Mat_<double>(3, 3) << std::cos(y), 0, std::sin(y), 0, 1,
                  0, -std::sin(y), 0, std::cos(y));
    cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, std::cos(p),
                  -std::sin(p), 0, std::sin(p), std::cos(p));
    const cv::Mat M = Rx * Ry * Rz * R;
    cv::Mat rv;
    cv::Rodrigues(M, rv);
    return cv::Vec3d(rv.at<double>(0), rv.at<double>(1), rv.at<double>(2));
  };

  struct Case {
    const char* name;
    cv::Vec3d tvec;
    cv::Vec3d rvec;
    cv::Vec3d expect;  // (距离m, yaw°, pitch°)
  };
  const std::vector<Case> cases{
      {"正前方 0.8m", tvec_of(0, 0, 800), rvec_of(0, 0, 0), {0.8, 0, 0}},
      {"正前方 1.5m", tvec_of(0, 0, 1500), rvec_of(0, 0, 0), {1.5, 0, 0}},
      {"右偏 20°, 1.0m", tvec_of(20, 0, 1000), rvec_of(0, 20, 0), {1.0, 20, 0}},
      {"上仰 10°, 1.5m", tvec_of(0, 10, 1500), rvec_of(0, 0, 10), {1.5, 0, 10}},
      {"右偏 15° 上仰 8°, 1.0m", tvec_of(15, 8, 1000), rvec_of(0, 15, 8),
       {1.0, 15, 8}},
      {"板内 roll 10°, 0.8m", tvec_of(0, 0, 800), rvec_of(10, 0, 0),
       {0.8, 0, 0}},
  };

  CameraCalib calib;
  calib.loadFromMats(K, D, img_size);

  bool all_pass = true;
  int idx = 0;
  double total_ms = 0.0;
  for (const auto& c : cases) {
    const cv::Mat img =
        RenderArmor(K, D, img_size, c.rvec, c.tvec, size, det.config().enemy_color);
    const int64 t0 = cv::getTickCount();
    auto armor = det.detect(img);
    total_ms += (cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency();

    cv::Mat dbg = img.clone();
    rm27::LightBarDetector::drawDebug(dbg, det.bars(), armor);
    char fname[64];
    std::snprintf(fname, sizeof(fname), "detect_case%d.jpg", idx);
    cv::imwrite(fname, dbg);

    bool pass = false;
    if (!armor) {
      std::printf("%-24s ❌ 未检测到目标（灯条候选 %zu 个）\n", c.name,
                  det.bars().size());
    } else {
      auto pose = calib.solvePose(armor->corners, size);
      if (!pose) {
        std::printf("%-24s ❌ 检测到但 PnP 失败\n", c.name);
      } else {
        double yaw = 0.0, pitch = 0.0;
        CameraCalib::poseToYawPitch(pose->tvec, &yaw, &pitch);
        const double d_err =
            std::fabs(pose->distance_mm / 1000.0 - c.expect[0]);
        const double y_err = std::fabs(yaw * 180.0 / CV_PI - c.expect[1]);
        const double p_err = std::fabs(pitch * 180.0 / CV_PI - c.expect[2]);
        // 28.05mm 小板在 640x480 下 1.5m 处灯条仅 ~9px、角点 1px 量化 →
        // 距离误差 ~5%。合成自检门限放宽到 8%（真实标定后卷尺验证为准）
        pass = d_err < 0.08 * c.expect[0] && y_err < 1.5 && p_err < 1.5;
        std::printf(
            "%-24s 距离 %.3fm(真 %.3f) | yaw %+.2f°(真 %+.2f) | pitch %+.2f°"
            "(真 %+.2f) | 重投影 %.2fpx | 灯条%zu %s\n",
            c.name, pose->distance_mm / 1000.0, c.expect[0],
            yaw * 180.0 / CV_PI, c.expect[1], pitch * 180.0 / CV_PI,
            c.expect[2], pose->reproj_error_px, det.bars().size(),
            pass ? "✅" : "❌");
        if (idx == 0) {
          const double W = size.width_mm, L = size.height_mm;
          const std::vector<cv::Point3f> obj{
              {0.f, (float)(W / 2), (float)(-L / 2)},
              {0.f, (float)(W / 2), (float)(L / 2)},
              {0.f, (float)(-W / 2), (float)(L / 2)},
              {0.f, (float)(-W / 2), (float)(-L / 2)}};
          std::vector<cv::Point2f> gt;
          cv::projectPoints(obj, c.rvec, c.tvec, K, D, gt);
          std::printf(
              "  真值: (%6.1f,%6.1f)(%6.1f,%6.1f)(%6.1f,%6.1f)(%6.1f,%6.1f)\n",
              gt[0].x, gt[0].y, gt[1].x, gt[1].y, gt[2].x, gt[2].y, gt[3].x,
              gt[3].y);
          std::printf(
              "  检测: (%6.1f,%6.1f)(%6.1f,%6.1f)(%6.1f,%6.1f)(%6.1f,%6.1f)\n",
              armor->corners[0].x, armor->corners[0].y, armor->corners[1].x,
              armor->corners[1].y, armor->corners[2].x, armor->corners[2].y,
              armor->corners[3].x, armor->corners[3].y);
        }
      }
    }
    all_pass = all_pass && pass;
    ++idx;
  }
  std::printf("\n平均每帧检测耗时 %.2f ms | 调试图 detect_case0~%d.jpg\n",
              total_ms / cases.size(), idx - 1);
  std::printf("=== 检测自检%s ===\n", all_pass ? "全部通过 ✅" : "有失败项 ❌");
  return all_pass ? 0 : 1;
}

void PrintUsage() {
  std::printf(
      "用法:\n"
      "  detect_test                         相机实时\n"
      "  detect_test --image 照片.jpg        静态图交互\n"
      "  detect_test --image 照片.jpg --once 跑一次退出（脚本用）\n"
      "  detect_test --selftest              合成图自检\n"
      "\n"
      "  其他: --calib 标定文件 | --lightbar 检测参数 | --armor 装甲板尺寸\n"
      "        --device N /dev/videoN（默认 0）\n"
      "        分辨率默认 800x600（--width/--height 可改）\n"
      "        --format mjpg|yuyv 采集格式（默认 MJPG 优先）\n"
      "\n"
      "按键: q/ESC 退出 | s 截图 | c 切换敌方颜色 | r 重载检测参数\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::string image_path, calib_path = "configs/camera.yaml",
                      lightbar_path = "configs/light_bar.yaml",
                      armor_path = "configs/armor.yaml";
  int device = 0;
  int width = 800;
  int height = 600;
  std::string format;  // "mjpg"/"yuyv"/空
  bool once = false, selftest = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--image" && i + 1 < argc) {
      image_path = argv[++i];
    } else if (a == "--calib" && i + 1 < argc) {
      calib_path = argv[++i];
    } else if (a == "--lightbar" && i + 1 < argc) {
      lightbar_path = argv[++i];
    } else if (a == "--armor" && i + 1 < argc) {
      armor_path = argv[++i];
    } else if (a == "--device" && i + 1 < argc) {
      device = std::atoi(argv[++i]);
    } else if (a == "--width" && i + 1 < argc) {
      width = std::atoi(argv[++i]);
    } else if (a == "--height" && i + 1 < argc) {
      height = std::atoi(argv[++i]);
    } else if (a == "--format" && i + 1 < argc) {
      format = argv[++i];
    } else if (a == "--once") {
      once = true;
    } else if (a == "--selftest") {
      selftest = true;
    } else if (a == "-h" || a == "--help") {
      PrintUsage();
      return 0;
    } else {
      std::fprintf(stderr, "未知参数: %s\n", a.c_str());
      PrintUsage();
      return 2;
    }
  }

  const ArmorSize size = LoadArmorSize(armor_path);
  rm27::LightBarDetector det(lightbar_path);
  const cv::Size img_size(640, 480);
  const cv::Mat K_synth = (cv::Mat_<double>(3, 3) << 480, 0, 320, 0, 480, 240,
                           0, 0, 1);
  const cv::Mat D_synth = cv::Mat::zeros(1, 5, CV_64F);

  if (selftest) {
    cv::Mat K = K_synth, D = D_synth;
    CameraCalib tmp;
    if (tmp.load(calib_path)) {
      K = tmp.K();
      D = tmp.D();
    } else {
      std::printf("[detect_test] 没有标定文件，自检使用合成内参 fx=480\n");
    }
    return RunDetectSelftest(K, D, img_size, size, det);
  }

  CameraCalib calib;
  if (!calib.load(calib_path)) {
    std::fprintf(stderr,
                 "⚠️ 没有 %s。请先：\n"
                 "  1) ./capture_calib 采集棋子格图\n"
                 "  2) ./calibrate 生成内参\n",
                 calib_path.c_str());
    return -1;
  }

  const bool use_camera = image_path.empty();
  UsbCamera cam;
  cv::Mat raw;

  if (use_camera) {
    if (!format.empty()) cam.SetFormat(format);
    if (!cam.Open(device, width, height, "configs/camera_runtime.yaml")) {
      std::fprintf(stderr, "相机初始化失败\n");
      return -1;
    }
  } else {
    raw = cv::imread(image_path);
    if (raw.empty()) {
      std::fprintf(stderr, "打不开图片: %s\n", image_path.c_str());
      return -1;
    }
  }

  if (use_camera && calib.imageSize() != cv::Size(cam.Width(), cam.Height())) {
    std::fprintf(stderr,
                 "⚠️ 标定分辨率 %dx%d 与当前采集 %dx%d 不一致，请先重新标定！\n",
                 calib.imageSize().width, calib.imageSize().height, cam.Width(),
                 cam.Height());
  }
  std::printf("敌方颜色顺序: 蓝(默认) → c 可切红 | q=退出 s=截图 r=重载\n");
  if (!once) {
    if (std::getenv("DISPLAY") == nullptr) {
      std::fprintf(stderr,
                   "❌ 没有检测到 DISPLAY：无图形环境。\n"
                   "   远程请 ssh -Y，或先 export DISPLAY=:0；\n"
                   "   静态图跑一次可用 --image xx.jpg --once（不需要窗口）\n");
      return 1;
    }
    cv::namedWindow("detect_test", cv::WINDOW_NORMAL);
  }

  int frame_idx = 0;
  while (true) {
    if (use_camera) {
      std::chrono::steady_clock::time_point ts;
      if (!cam.GetLatestFrame(&raw, &ts)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
    }
    const int64 t0 = cv::getTickCount();
    auto armor = det.detect(raw);
    const double dt_ms =
        (cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency();

    if (armor) {
      if (auto pose = calib.solvePose(armor->corners, size)) {
        double yaw = 0.0, pitch = 0.0;
        CameraCalib::poseToYawPitch(pose->tvec, &yaw, &pitch);
        std::printf(
            "小装甲 | 距离 %.3f m | yaw %+.2f° | pitch %+.2f° | 重投影 %.2fpx"
            " | 检测 %.1fms | 采集 %.0ffps\n",
            pose->distance_mm / 1000.0, yaw * 180.0 / CV_PI,
            pitch * 180.0 / CV_PI, pose->reproj_error_px, dt_ms,
            cam.MeasuredFps());
      }
    } else {
      std::printf("无目标（灯条候选 %zu）| 检测 %.1fms | 采集 %.0ffps\n",
                  det.bars().size(), dt_ms, cam.MeasuredFps());
    }

    if (!once) {
      cv::Mat view = raw.clone();
      rm27::LightBarDetector::drawDebug(view, det.bars(), armor);
      cv::putText(view, "q=quit s=save c=color r=reload", cv::Point(10, 28),
                  cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 1);
      cv::imshow("detect_test", view);
      const int key = cv::waitKey(5) & 0xFF;
      if (key == 27 || key == 'q') break;
      if (key == 'c') {
        const std::string c =
            det.config().enemy_color == "blue" ? "red" : "blue";
        det.setEnemyColor(c);
        std::printf("敌方颜色切换为 %s\n", c.c_str());
      }
      if (key == 'r') {
        det.loadConfig(lightbar_path);
      }
      if (key == 's') {
        char name[64];
        std::snprintf(name, sizeof(name), "detect_%04d.jpg", frame_idx);
        cv::imwrite(name, view);
        std::printf("已保存 %s\n", name);
      }
    }
    if (once) {
      break;
    }
    ++frame_idx;
  }
  return 0;
}
