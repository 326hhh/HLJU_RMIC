#include "light_bar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

namespace rm27 {

// ------------------------------------------------------------------
// 配置加载
// ------------------------------------------------------------------
bool LightBarDetector::loadConfig(const std::string& yaml_path) {
  {
    std::ifstream f(yaml_path);
    if (!f.good()) {
      std::fprintf(stderr, "[LightBar] 参数文件不存在: %s，使用内置默认\n",
                   yaml_path.c_str());
      return false;
    }
  }
  cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::fprintf(stderr, "[LightBar] 打不开 %s，使用内置默认参数（敌方=蓝）\n",
                 yaml_path.c_str());
    return false;
  }
  auto readI = [&](const char* k, int& v) {
    if (!fs[k].isNone()) fs[k] >> v;
  };
  auto readD = [&](const char* k, double& v) {
    if (!fs[k].isNone()) fs[k] >> v;
  };
  auto readF = [&](const char* k, float& v) {
    if (!fs[k].isNone()) fs[k] >> v;
  };
  int tmp = 0;

  if (!fs["enemy_color"].isNone()) fs["enemy_color"] >> cfg_.enemy_color;

  readI("blue_h_min", cfg_.blue_h_min);
  readI("blue_h_max", cfg_.blue_h_max);
  readI("blue_s_min", cfg_.blue_s_min);
  readI("blue_v_min", cfg_.blue_v_min);
  readI("red_h_min_1", cfg_.red_h_min_1);
  readI("red_h_max_1", cfg_.red_h_max_1);
  readI("red_h_min_2", cfg_.red_h_min_2);
  readI("red_h_max_2", cfg_.red_h_max_2);
  readI("red_s_min", cfg_.red_s_min);
  readI("red_v_min", cfg_.red_v_min);
  readI("morph_kernel", cfg_.morph_kernel);
  readI("morph_iters", cfg_.morph_iters);

  readD("min_length", cfg_.min_length);
  readD("min_ratio", cfg_.min_ratio);
  readD("max_tilt_deg", cfg_.max_tilt_deg);
  readD("min_area", cfg_.min_area);
  readD("max_area", cfg_.max_area);
  readD("max_angle_diff_deg", cfg_.max_angle_diff_deg);
  readD("max_len_ratio", cfg_.max_len_ratio);
  readD("max_center_y_ratio", cfg_.max_center_y_ratio);
  readD("center_dist_ratio_min", cfg_.center_dist_ratio_min);
  readD("center_dist_ratio_max", cfg_.center_dist_ratio_max);

  if (!fs["use_pca"].isNone()) {
    fs["use_pca"] >> tmp;
    cfg_.use_pca = tmp != 0;
  }
  if (!fs["pca_required"].isNone()) {
    fs["pca_required"] >> tmp;
    cfg_.pca_required = tmp != 0;
  }
  readI("pass_optimize_lightbar_width", cfg_.pass_optimize_lightbar_width);
  readF("normalize_max_brightness", cfg_.normalize_max_brightness);
  readF("lightbar_min_mean_brightness", cfg_.lightbar_min_mean_brightness);
  readF("padding_scale", cfg_.padding_scale);
  readF("search_start_ratio", cfg_.search_start_ratio);
  readF("search_end_ratio", cfg_.search_end_ratio);
  fs.release();

  std::printf("[LightBar] %s 已加载: 敌方=%s PCA=%s\n", yaml_path.c_str(),
              cfg_.enemy_color.c_str(), cfg_.use_pca ? "开" : "关");
  return true;
}

// ------------------------------------------------------------------
// 轮廓 → 灯条（借鉴 jlu types.cpp LightBar(contour)）
// ------------------------------------------------------------------
LightBar LightBarDetector::fromContour(const std::vector<cv::Point>& contour) {
  LightBar bar;
  bar.rect = cv::minAreaRect(contour);

  // 用轮廓质心当中心（比矩形对角线交点稳）
  cv::Point2f c(0.f, 0.f);
  for (const auto& p : contour) {
    c += cv::Point2f(static_cast<float>(p.x), static_cast<float>(p.y));
  }
  bar.center = c / static_cast<float>(contour.size());

  // 4 角按 y 排序 → 上两点中点=top，下两点中点=bottom
  cv::Point2f p[4];
  bar.rect.points(p);
  std::sort(p, p + 4,
            [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
  bar.top = (p[0] + p[1]) * 0.5f;
  bar.bottom = (p[2] + p[3]) * 0.5f;
  bar.length = cv::norm(bar.top - bar.bottom);
  bar.width = cv::norm(p[0] - p[1]);

  cv::Point2f ax = bar.top - bar.bottom;
  const float n = cv::norm(ax);
  bar.axis = n > 0.f ? ax / n : cv::Point2f(0.f, -1.f);
  bar.tilt_deg = static_cast<float>(
      std::atan2(std::fabs(bar.axis.x), std::fabs(bar.axis.y)) * 180.0 / CV_PI);
  bar.signed_tilt =
      std::atan2(static_cast<double>(bar.axis.x),
                 -static_cast<double>(bar.axis.y));
  return bar;
}

// ------------------------------------------------------------------
// 颜色掩膜 → 灯条候选
// ------------------------------------------------------------------
std::vector<LightBar> LightBarDetector::findBars(const cv::Mat& bgr) {
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::Mat mask(hsv.size(), CV_8UC1, cv::Scalar(0));

  const bool blue =
      (cfg_.enemy_color == "blue" || cfg_.enemy_color == "Blue");
  if (blue) {
    cv::inRange(hsv, cv::Scalar(cfg_.blue_h_min, cfg_.blue_s_min, cfg_.blue_v_min),
                cv::Scalar(cfg_.blue_h_max, 255, 255), mask);
  } else {
    cv::Mat m1, m2;
    cv::inRange(hsv,
                cv::Scalar(cfg_.red_h_min_1, cfg_.red_s_min, cfg_.red_v_min),
                cv::Scalar(cfg_.red_h_max_1, 255, 255), m1);
    cv::inRange(hsv,
                cv::Scalar(cfg_.red_h_min_2, cfg_.red_s_min, cfg_.red_v_min),
                cv::Scalar(cfg_.red_h_max_2, 255, 255), m2);
    cv::bitwise_or(m1, m2, mask);
  }

  if (cfg_.morph_kernel > 0) {
    const cv::Mat k = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(cfg_.morph_kernel, cfg_.morph_kernel));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k, cv::Point(-1, -1),
                     cfg_.morph_iters);
  }
  mask_debug_ = mask.clone();

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  std::vector<LightBar> bars;
  for (const auto& contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < cfg_.min_area || area > cfg_.max_area) continue;

    LightBar bar = fromContour(contour);
    if (bar.length < cfg_.min_length) continue;
    if (bar.width < 1.f) continue;
    if (bar.length / bar.width < cfg_.min_ratio) continue;
    if (bar.tilt_deg > cfg_.max_tilt_deg) continue;
    bars.push_back(bar);
  }
  return bars;
}

// ------------------------------------------------------------------
// PCA 角点修正（借鉴 jlu lightbar_corrector.cpp）
// ------------------------------------------------------------------
std::optional<cv::Point2f> LightBarDetector::findCorner(
    const cv::Mat& gray, const LightBar& bar, const cv::Point2f& axis_dir,
    const cv::Point2f& centroid, float mean_val, bool toward_top) {
  const float L = bar.length;
  const float oper = toward_top ? 1.f : -1.f;
  const float dx = axis_dir.x * oper;
  const float dy = axis_dir.y * oper;

  std::vector<cv::Point2f> candidates;
  const int n = static_cast<int>(bar.width) - 2;
  const int half_n = static_cast<int>(std::round(n / 2.0));
  for (int i = -half_n; i <= half_n; ++i) {
    const float x0 = centroid.x + L * cfg_.search_start_ratio * dx +
                     static_cast<float>(i);
    const float y0 = centroid.y + L * cfg_.search_start_ratio * dy;

    cv::Point2f prev(x0, y0);
    cv::Point2f corner(x0, y0);
    float max_diff = 0.f;
    bool has_corner = false;
    for (float x = x0 + dx, y = y0 + dy;
         std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) <
         L * (cfg_.search_end_ratio - cfg_.search_start_ratio);
         x += dx, y += dy) {
      const cv::Point2f cur(x, y);
      if (cur.x < 0 || cur.x >= gray.cols || cur.y < 0 || cur.y >= gray.rows) {
        break;
      }
      // 沿轴从亮（灯条内）走向暗（背景），亮度骤降处 = 端点
      const float diff =
          static_cast<float>(gray.at<uchar>(static_cast<int>(prev.y),
                                            static_cast<int>(prev.x))) -
          static_cast<float>(gray.at<uchar>(static_cast<int>(cur.y),
                                            static_cast<int>(cur.x)));
      if (diff > max_diff &&
          static_cast<float>(gray.at<uchar>(static_cast<int>(prev.y),
                                            static_cast<int>(prev.x))) >
              mean_val) {
        max_diff = diff;
        corner = prev;
        has_corner = true;
      }
      prev = cur;
    }
    if (has_corner) {
      candidates.push_back(corner);
    }
  }
  if (candidates.empty()) {
    return std::nullopt;
  }
  cv::Point2f sum(0.f, 0.f);
  for (const auto& p : candidates) {
    sum += p;
  }
  return sum / static_cast<float>(candidates.size());
}

bool LightBarDetector::correctBar(const cv::Mat& gray, LightBar& bar) {
  if (bar.width <= static_cast<float>(cfg_.pass_optimize_lightbar_width)) {
    return false;
  }
  // 1) 扩大包围盒，留出搜索余量
  cv::Rect box = bar.rect.boundingRect();
  const int padx = static_cast<int>(box.width * cfg_.padding_scale);
  const int pady = static_cast<int>(box.height * cfg_.padding_scale);
  box.x -= padx;
  box.y -= pady;
  box.width += padx * 2;
  box.height += pady * 2;
  box.x = std::max(box.x, 0);
  box.y = std::max(box.y, 0);
  box.x = std::min(box.x, gray.cols - 1);
  box.y = std::min(box.y, gray.rows - 1);
  box.width = std::min(box.width, gray.cols - box.x);
  box.height = std::min(box.height, gray.rows - box.y);
  if (box.width <= 0 || box.height <= 0) {
    return false;
  }

  cv::Mat roi = gray(box);
  const float mean_val = static_cast<float>(cv::mean(roi)[0]);
  if (mean_val <= cfg_.lightbar_min_mean_brightness) {
    return false;  // 太暗，不是灯条
  }
  cv::Mat roi_f;
  roi.convertTo(roi_f, CV_32F);
  cv::normalize(roi_f, roi_f, 0, cfg_.normalize_max_brightness, cv::NORM_MINMAX);

  // 2) 矩分析求对称轴（jlu：从点云 PCA 换成矩分析，更快）
  const cv::Moments m = cv::moments(roi_f, false);
  if (m.m00 == 0.0) {
    return false;
  }
  cv::Point2f centroid(static_cast<float>(m.m10 / m.m00),
                       static_cast<float>(m.m01 / m.m00));
  centroid +=
      cv::Point2f(static_cast<float>(box.x), static_cast<float>(box.y));
  const double mu20 = m.mu20, mu11 = m.mu11, mu02 = m.mu02;
  if (mu20 == 0.0 && mu11 == 0.0 && mu02 == 0.0) {
    return false;
  }
  const double theta = 0.5 * std::atan2(2.0 * mu11, mu20 - mu02);
  cv::Point2f axis(static_cast<float>(std::cos(theta)),
                   static_cast<float>(std::sin(theta)));
  axis /= cv::norm(axis);
  if (axis.y > 0) {
    axis = -axis;  // 指向上方（图像坐标 y 向下）
  }

  // 3) 沿轴找两端角点（最大亮度下降处）
  auto top = findCorner(gray, bar, axis, centroid, mean_val, true);
  auto bottom = findCorner(gray, bar, axis, centroid, mean_val, false);
  if (!top || !bottom) {
    return false;
  }

  bar.center = centroid;
  bar.axis = axis;
  bar.top = *top;
  bar.bottom = *bottom;
  bar.length = cv::norm(bar.top - bar.bottom);
  bar.signed_tilt =
      std::atan2(static_cast<double>(bar.axis.x), -static_cast<double>(bar.axis.y));
  bar.pca_ok = true;
  return true;
}

// ------------------------------------------------------------------
// 灯条配对 → 装甲板
// ------------------------------------------------------------------
std::optional<ArmorTarget> LightBarDetector::pairBars(
    std::vector<LightBar>& bars) {
  ArmorTarget best;
  bool found = false;
  double best_score = 0.0;

  const int n = static_cast<int>(bars.size());
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      LightBar l = bars[i];
      LightBar r = bars[j];
      if (l.center.x > r.center.x) {
        std::swap(l, r);  // l = 左条, r = 右条
      }

      const float avg_len = (l.length + r.length) * 0.5f;
      const float dist = std::fabs(l.center.x - r.center.x);
      if (avg_len <= 0.f || dist <= 0.f) continue;

      // ① 倾角差
      const double angle_diff =
          std::fabs(l.signed_tilt - r.signed_tilt) * 180.0 / CV_PI;
      if (angle_diff > cfg_.max_angle_diff_deg) continue;

      // ② 长度比
      const float len_ratio =
          std::max(l.length, r.length) /
          std::max(1e-3f, std::min(l.length, r.length));
      if (len_ratio > cfg_.max_len_ratio) continue;

      // ③ 中心纵向差
      if (std::fabs(l.center.y - r.center.y) >
          cfg_.max_center_y_ratio * avg_len)
        continue;

      // ④ 中心距 / 平均灯条长（小装甲 ≈ 2.39，板面偏转会压小）
      const float dist_ratio = dist / avg_len;
      if (dist_ratio < cfg_.center_dist_ratio_min ||
          dist_ratio > cfg_.center_dist_ratio_max)
        continue;

      // ⑤ PCA 要求（默认关：灯条太小时无法精修，用矩形端点兜底）
      if (cfg_.use_pca && cfg_.pca_required && !(l.pca_ok && r.pca_ok))
        continue;

      const double score =
          static_cast<double>(avg_len) - std::fabs(l.center.y - r.center.y);
      if (!found || score > best_score) {
        found = true;
        best_score = score;
        best.left_light = l;
        best.right_light = r;
        best.center = (l.center + r.center) * 0.5f;
        // 角点顺序：板左下 → 板左上 → 板右上 → 板右下
        // ⚠️ 镜像：板子朝向相机时，板的左条出现在画面右侧，
        //    所以「板左下」= 画面右条的底端（r = 画面右条）
        best.corners = {r.bottom, r.top, l.top, l.bottom};
      }
    }
  }
  if (!found) {
    return std::nullopt;
  }
  return best;
}

// ------------------------------------------------------------------
// 主入口
// ------------------------------------------------------------------
std::optional<ArmorTarget> LightBarDetector::detect(const cv::Mat& bgr) {
  if (bgr.empty()) {
    return std::nullopt;
  }
  bars_ = findBars(bgr);
  if (cfg_.use_pca) {
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    for (auto& b : bars_) {
      correctBar(gray, b);
    }
  }
  return pairBars(bars_);
}

// ------------------------------------------------------------------
// 调试绘制
// ------------------------------------------------------------------
void LightBarDetector::drawDebug(cv::Mat& img,
                                 const std::vector<LightBar>& bars,
                                 const std::optional<ArmorTarget>& armor) {
  for (const auto& b : bars) {
    cv::line(img, b.bottom, b.top, cv::Scalar(0, 255, 255), 2);
    cv::circle(img, b.center, 3, cv::Scalar(0, 255, 255), -1);
  }
  if (!armor) {
    return;
  }
  const auto& a = *armor;
  for (int k = 0; k < 4; ++k) {
    cv::line(img, a.corners[k], a.corners[(k + 1) % 4], cv::Scalar(0, 255, 0),
             2);
  }
  for (const auto& p : a.corners) {
    cv::circle(img, p, 4, cv::Scalar(0, 255, 0), -1);
  }
  cv::putText(img, "SMALL", a.center + cv::Point2f(-30.f, -15.f),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
}

}  // namespace rm27
