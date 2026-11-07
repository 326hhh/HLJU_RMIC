/**
 * @file  pose_test.cpp
 * @brief 几何链路验证工具：4 角点 → PnP → 距离 / yaw / pitch（27RM 校内赛）
 *
 * 写检测器之前，先用它验证「标定 + 坐标约定 + PnP 解算」：
 * 把真的装甲板摆在相机前，鼠标点 4 个角，用卷尺/量角器核对输出。
 * 之后检测器出问题，就一定是检测的锅。
 *
 * 模式：
 *   ./pose_test                                    相机实时交互
 *   ./pose_test --image 照片.jpg                   图片交互
 *   ./pose_test --image 照片.jpg --corners x1,y1 x2,y2 x3,y3 x4,y4
 *                                                  无窗口直接算（脚本可用）
 *   ./pose_test --selftest                         合成数据自检（无需相机）
 *
 * 角点顺序约定：左下 → 左上 → 右上 → 右下
 */

#include "camera_calib.hpp"
#include "camera_usb.hpp"

#include <opencv2/opencv.hpp>

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace {

// ---------------------------------------------------------------
// 命令行选项
// ---------------------------------------------------------------
struct Options {
  std::string image_path;
  std::string calib_path = "configs/camera.yaml";
  std::string armor_path = "configs/armor.yaml";
  std::string runtime_path = "configs/camera_runtime.yaml";
  std::vector<cv::Point2f> corners;       // --corners，4 个 "x,y"
  double custom_w = 0.0, custom_l = 0.0;  // --size WxL（毫米）
  int device = 0;
  bool selftest = false;  // --selftest
};

void PrintUsage() {
  std::printf(
      "用法:\n"
      "  pose_test                                    相机交互：鼠标点 4 个角\n"
      "  pose_test --image 照片.jpg                   图片交互\n"
      "  pose_test --image 照片.jpg --corners x1,y1 x2,y2 x3,y3 x4,y4\n"
      "                                               无窗口直接算（点序：左下 左上 右上 右下）\n"
      "  pose_test --selftest                         合成数据自检（无需相机/显示器）\n"
      "\n"
      "  其他: --calib 标定文件(默认 configs/camera.yaml)\n"
      "        --armor 装甲板尺寸(默认 configs/armor.yaml)\n"
      "        --runtime 相机运行时参数(默认 configs/camera_runtime.yaml)\n"
      "        --size WxL 自定义矩形尺寸(毫米)，如 --size 67.09x28.05\n"
      "        --device N /dev/videoN（默认 0）\n"
      "\n"
      "相机模式按键: r 清除重选 | u 切换去畸变显示 | s 截图 | q/ESC 退出\n");
}

bool ParseArgs(int argc, char** argv, Options* opt) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--image" && i + 1 < argc) {
      opt->image_path = argv[++i];
    } else if (a == "--calib" && i + 1 < argc) {
      opt->calib_path = argv[++i];
    } else if (a == "--armor" && i + 1 < argc) {
      opt->armor_path = argv[++i];
    } else if (a == "--runtime" && i + 1 < argc) {
      opt->runtime_path = argv[++i];
    } else if (a == "--device" && i + 1 < argc) {
      opt->device = std::atoi(argv[++i]);
    } else if (a == "--size" && i + 1 < argc) {
      double w = 0.0, l = 0.0;
      if (std::sscanf(argv[++i], "%lfx%lf", &w, &l) != 2) {
        std::fprintf(stderr, "--size 格式错误: 要 WxL（毫米），如 67.09x28.05\n");
        return false;
      }
      opt->custom_w = w;
      opt->custom_l = l;
    } else if (a == "--selftest") {
      opt->selftest = true;
    } else if (a == "--corners") {
      for (; i + 1 < argc; ++i) {
        const std::string s = argv[i + 1];
        if (s.rfind("--", 0) == 0) break;  // 撞到下一个选项就停
        float x = 0.f, y = 0.f;
        if (std::sscanf(s.c_str(), "%f,%f", &x, &y) != 2) {
          std::fprintf(stderr, "角点格式错误: %s（要 x,y）\n", s.c_str());
          return false;
        }
        opt->corners.push_back(cv::Point2f(x, y));
      }
    } else if (a == "-h" || a == "--help") {
      PrintUsage();
      std::exit(0);
    } else {
      std::fprintf(stderr, "未知参数: %s\n", a.c_str());
      return false;
    }
  }
  if (!opt->corners.empty() && opt->corners.size() != 4) {
    std::fprintf(stderr, "--corners 必须正好 4 个点（左下 左上 右上 右下）\n");
    return false;
  }
  return true;
}

// ---------------------------------------------------------------
// 鼠标点角 / 绘制
// ---------------------------------------------------------------
struct ClickState {
  std::vector<cv::Point2f> pts;  // 显示窗口坐标系下的点
  bool dirty = false;            // 有新点/需重新解算
};

void OnMouse(int event, int x, int y, int, void* param) {
  if (event != cv::EVENT_LBUTTONDOWN) return;
  auto* st = static_cast<ClickState*>(param);
  if (st->pts.size() >= 4) return;
  st->pts.push_back(cv::Point2f(static_cast<float>(x), static_cast<float>(y)));
  st->dirty = true;
}

void DrawClicks(cv::Mat& view, const ClickState& st) {
  for (std::size_t i = 0; i < st.pts.size(); ++i) {
    cv::circle(view, st.pts[i], 5, cv::Scalar(0, 255, 255), -1);
    cv::putText(view, std::to_string(static_cast<int>(i) + 1),
                st.pts[i] + cv::Point2f(9, -9), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 255), 2);
    if (i > 0) {
      cv::line(view, st.pts[i - 1], st.pts[i], cv::Scalar(0, 255, 255), 1);
    }
  }
  if (st.pts.size() == 4) {
    cv::line(view, st.pts[3], st.pts[0], cv::Scalar(0, 255, 255), 1);
  }
}

/** 把显示窗口坐标换回原始（含畸变）图像坐标。 */
std::array<cv::Point2f, 4> ToOriginalCorners(
    const CameraCalib& calib, const std::array<cv::Point2f, 4>& view_pts,
    bool view_is_undistorted) {
  if (!view_is_undistorted) return view_pts;
  const cv::Mat& K = calib.K();
  const cv::Mat& D = calib.D();
  const double fx = K.at<double>(0, 0), fy = K.at<double>(1, 1);
  const double cx = K.at<double>(0, 2), cy = K.at<double>(1, 2);
  const double k1 = D.at<double>(0), k2 = D.at<double>(1);
  const double p1 = D.at<double>(2), p2 = D.at<double>(3);
  const double k3 = D.total() >= 5 ? D.at<double>(4) : 0.0;
  std::array<cv::Point2f, 4> out;
  for (std::size_t i = 0; i < 4; ++i) {
    const double x = (view_pts[i].x - cx) / fx;
    const double y = (view_pts[i].y - cy) / fy;
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2, r6 = r4 * r2;
    const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    const double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    out[i] = cv::Point2f(static_cast<float>(xd * fx + cx),
                         static_cast<float>(yd * fy + cy));
  }
  return out;
}

void DrawPoseAxes(cv::Mat& view, const CameraCalib& calib,
                  const ArmorPose& pose) {
  std::vector<cv::Point3f> axis{{0.f, 0.f, 0.f},
                                {60.f, 0.f, 0.f},
                                {0.f, 60.f, 0.f},
                                {0.f, 0.f, 60.f}};
  std::vector<cv::Point2f> im;
  cv::projectPoints(axis, pose.rvec, pose.tvec, calib.K(), calib.D(), im);
  cv::line(view, im[0], im[1], cv::Scalar(0, 0, 255), 2);  // x 朝前 红
  cv::line(view, im[0], im[2], cv::Scalar(0, 255, 0), 2);  // y 向左 绿
  cv::line(view, im[0], im[3], cv::Scalar(255, 0, 0), 2);  // z 向上 蓝
}

void SolveAndReport(const CameraCalib& calib,
                    const std::array<cv::Point2f, 4>& corners,
                    const ArmorSize& size, cv::Mat& view) {
  auto pose = calib.solvePose(corners, size);
  if (!pose) {
    std::printf(
        "  ❌ 解算失败：重投影超差/目标在相机后方？检查角点顺序"
        "（左下→左上→右上→右下）与装甲板尺寸\n");
    return;
  }
  double yaw = 0.0, pitch = 0.0;
  CameraCalib::poseToYawPitch(pose->tvec, &yaw, &pitch);
  std::printf(
      "  距离 %.3f m | yaw %+.2f° | pitch %+.2f° | 重投影 %.3f px | "
      "tvec=(%.0f, %.0f, %.0f) mm\n",
      pose->distance_mm / 1000.0, yaw * 180.0 / CV_PI, pitch * 180.0 / CV_PI,
      pose->reproj_error_px, pose->tvec[0], pose->tvec[1], pose->tvec[2]);
  DrawPoseAxes(view, calib, *pose);
}

// ---------------------------------------------------------------
// 合成自检：无需相机/显示器，验证坐标约定与解算闭环
// ---------------------------------------------------------------
int RunSelftest(const cv::Mat& K, const cv::Mat& D, const cv::Size& img_size,
                const ArmorSize& size) {
  std::printf("=== 合成自检 ===\n");
  std::printf("用内参把已知位姿的装甲板投影成 2D 角点，再解回来对比。\n");
  std::printf("图像 %dx%d | fx=%.2f fy=%.2f\n\n", img_size.width,
              img_size.height, K.at<double>(0, 0), K.at<double>(1, 1));

  const double W = size.width_mm, L = size.height_mm;
  const std::vector<cv::Point3f> obj{
      {0.f, static_cast<float>(W / 2), static_cast<float>(-L / 2)},   // 左下
      {0.f, static_cast<float>(W / 2), static_cast<float>(L / 2)},    // 左上
      {0.f, static_cast<float>(-W / 2), static_cast<float>(L / 2)},   // 右上
      {0.f, static_cast<float>(-W / 2), static_cast<float>(-L / 2)},  // 右下
  };

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
    cv::Vec3d tvec;    // mm
    cv::Vec3d rvec;    // 弧度
    cv::Vec3d expect;  // (距离m, yaw°, pitch°)
    bool expect_fail = false;
  };
  const std::vector<Case> cases{
      {"正前方 0.8m", tvec_of(0, 0, 800), rvec_of(0, 0, 0), {0.8, 0, 0}},
      {"正前方 1.5m", tvec_of(0, 0, 1500), rvec_of(0, 0, 0), {1.5, 0, 0}},
      {"右偏 20°, 1.0m", tvec_of(20, 0, 1000), rvec_of(0, 20, 0), {1.0, 20, 0}},
      {"上仰 10°, 2.0m", tvec_of(0, 10, 2000), rvec_of(0, 0, 10), {2.0, 0, 10}},
      {"右偏 15° 上仰 8°, 1.2m", tvec_of(15, 8, 1200), rvec_of(0, 15, 8),
       {1.2, 15, 8}},
      {"板内 roll 10°, 0.8m", tvec_of(0, 0, 800), rvec_of(10, 0, 0),
       {0.8, 0, 0}},
      {"【故意】交叉乱序", tvec_of(0, 0, 1000), rvec_of(0, 0, 0), {}, true},
  };

  // 用一个临时 CameraCalib 包装 K/D（solvePose 是纯 PnP）
  CameraCalib calib;
  calib.loadFromMats(K, D, img_size);

  bool all_pass = true;
  for (const auto& c : cases) {
    std::vector<cv::Point2f> img;
    cv::projectPoints(obj, c.rvec, c.tvec, K, D, img);
    std::array<cv::Point2f, 4> corners{img[0], img[1], img[2], img[3]};
    if (c.expect_fail) {
      // 交叉乱序：左下→右上→左上→右下（自交蝴蝶结，非刚体可达）
      corners = {img[0], img[2], img[1], img[3]};
    }
    auto pose = calib.solvePose(corners, size);
    bool pass = false;
    if (c.expect_fail) {
      pass = !pose.has_value();
      std::printf("%-28s %s\n", c.name,
                  pass ? "解算被拒（符合预期）✅" : "解算未被拒（不应通过）❌");
    } else if (pose) {
      double yaw = 0.0, pitch = 0.0;
      CameraCalib::poseToYawPitch(pose->tvec, &yaw, &pitch);
      const double d_err = std::fabs(pose->distance_mm / 1000.0 - c.expect[0]);
      const double y_err = std::fabs(yaw * 180.0 / CV_PI - c.expect[1]);
      const double p_err = std::fabs(pitch * 180.0 / CV_PI - c.expect[2]);
      pass = d_err < 0.05 * c.expect[0] && y_err < 1.5 && p_err < 1.5;
      std::printf(
          "%-28s 距离 %.3fm(真 %.3f) | yaw %+.2f°(真 %+.2f) | pitch %+.2f°"
          "(真 %+.2f) | 重投影 %.3fpx %s\n",
          c.name, pose->distance_mm / 1000.0, c.expect[0],
          yaw * 180.0 / CV_PI, c.expect[1], pitch * 180.0 / CV_PI, c.expect[2],
          pose->reproj_error_px, pass ? "✅" : "❌");
    } else {
      std::printf("%-28s 解算失败 ❌（不应失败）\n", c.name);
    }
    all_pass = all_pass && pass;
  }
  std::printf("\n=== 自检%s ===\n", all_pass ? "全部通过 ✅" : "有失败项 ❌");
  return all_pass ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!ParseArgs(argc, argv, &opt)) {
    PrintUsage();
    return 2;
  }

  const ArmorSize armor_default = LoadArmorSize(opt.armor_path);
  const bool has_custom = opt.custom_w > 0.0 && opt.custom_l > 0.0;
  ArmorSize size = armor_default;
  if (has_custom) {
    size.width_mm = opt.custom_w;
    size.height_mm = opt.custom_l;
  }

  // 标定：自检模式下允许没有 camera.yaml（用合成内参）
  CameraCalib calib;
  cv::Mat K_synth = (cv::Mat_<double>(3, 3) << 480, 0, 320, 0, 480, 240, 0, 0,
                     1);
  cv::Mat D_synth = cv::Mat::zeros(1, 5, CV_64F);
  if (opt.selftest) {
    cv::Mat K = K_synth, D = D_synth;
    if (calib.load(opt.calib_path)) {
      K = calib.K();
      D = calib.D();
    } else {
      std::printf("[pose_test] 没有标定文件，自检使用合成内参 fx=480\n");
    }
    return RunSelftest(K, D, cv::Size(640, 480), size);
  }
  if (!calib.load(opt.calib_path)) return -1;

  // ---------- 纯角点计算模式（无窗口） ----------
  if (opt.corners.size() == 4) {
    std::array<cv::Point2f, 4> corners{opt.corners[0], opt.corners[1],
                                       opt.corners[2], opt.corners[3]};
    cv::Mat dummy;
    SolveAndReport(calib, corners, size, dummy);
    if (!opt.image_path.empty()) {
      cv::Mat img = cv::imread(opt.image_path);
      if (!img.empty()) {
        for (int i = 0; i < 4; ++i) {
          cv::circle(img, opt.corners[i], 5, cv::Scalar(0, 255, 255), -1);
        }
        cv::imwrite("pose_result.jpg", img);
        std::printf("已画点存为 pose_result.jpg\n");
      }
    }
    return 0;
  }

  // ---------- 交互模式 ----------
  const bool use_camera = opt.image_path.empty();
  UsbCamera cam;
  cv::Mat raw;
  ClickState st;

  if (use_camera) {
    if (!cam.Open(opt.device, 800, 600, opt.runtime_path)) {
      std::fprintf(stderr, "相机初始化失败\n");
      return -1;
    }
    if (calib.imageSize() != cv::Size(cam.Width(), cam.Height())) {
      std::fprintf(stderr,
                   "⚠️ 标定分辨率 %dx%d 与当前采集 %dx%d 不一致，请先重新标定！\n",
                   calib.imageSize().width, calib.imageSize().height,
                   cam.Width(), cam.Height());
    }
    std::chrono::steady_clock::time_point ts;
    while (!cam.GetLatestFrame(&raw, &ts)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } else {
    raw = cv::imread(opt.image_path);
    if (raw.empty()) {
      std::fprintf(stderr, "打不开图片: %s\n", opt.image_path.c_str());
      return -1;
    }
  }

  std::printf("装甲板: %.2f x %.2f mm\n", size.width_mm, size.height_mm);
  std::printf("请按顺序点 4 个角: 左下 → 左上 → 右上 → 右下\n");
  std::printf("r=清除 u=切换去畸变显示 s=截图 q=退出\n");

  if (std::getenv("DISPLAY") == nullptr) {
    std::fprintf(stderr,
                 "❌ 没有检测到 DISPLAY：无图形环境。\n"
                 "   远程请 ssh -Y，或先 export DISPLAY=:0；\n"
                 "   纯计算可用 --corners x,y ... 或 --selftest（不需要窗口）\n");
    return 1;
  }
  cv::namedWindow("pose_test", cv::WINDOW_NORMAL);
  cv::setMouseCallback("pose_test", OnMouse, &st);

  bool show_undistort = false;
  int frame_idx = 0;

  while (true) {
    if (use_camera) {
      std::chrono::steady_clock::time_point ts;
      if (!cam.GetLatestFrame(&raw, &ts)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
    }
    cv::Mat view;
    if (show_undistort) {
      calib.undistort(raw, view);
    } else {
      raw.copyTo(view);
    }

    DrawClicks(view, st);
    cv::putText(view,
                "1:LB 2:LT 3:RT 4:RB | r=reset u=undistort s=save q=quit",
                cv::Point(10, 28), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 255, 0), 1);

    if (st.pts.size() == 4 && st.dirty) {
      std::array<cv::Point2f, 4> vp{st.pts[0], st.pts[1], st.pts[2], st.pts[3]};
      SolveAndReport(calib, ToOriginalCorners(calib, vp, show_undistort), size,
                     view);
      st.dirty = false;
    }

    cv::imshow("pose_test", view);
    const int key = cv::waitKey(5) & 0xFF;
    if (key == 27 || key == 'q') break;
    if (key == 'r') {
      st.pts.clear();
      st.dirty = true;
      std::printf("已清除，重新点 4 个角\n");
    }
    if (key == 'u') {
      show_undistort = !show_undistort;
      st.pts.clear();
      st.dirty = true;
      std::printf("显示切换为 %s（坐标系变了，角点已清空）\n",
                  show_undistort ? "去畸变图" : "原图");
    }
    if (key == 's') {
      char name[64];
      std::snprintf(name, sizeof(name), "pose_%04d.jpg", frame_idx);
      cv::imwrite(name, view);
      std::printf("已保存 %s\n", name);
    }
    ++frame_idx;
  }
  return 0;
}
