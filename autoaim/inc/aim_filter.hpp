#ifndef RM27_AIM_FILTER_HPP_
#define RM27_AIM_FILTER_HPP_

/**
 * @file  aim_filter.hpp
 * @brief 位姿平滑 + 目标状态机 —— 27RM 校内赛视觉
 *
 * 作用：
 *   1. 对 tvec/距离/yaw/pitch 做指数滑动平均（EMA），压掉 PnP 抖动
 *   2. 单目标状态机：连续命中 N 帧才认定目标；连续丢 M 帧才判丢失
 *   3. 跳变门限：目标突然跳变（换板/误检）时重置滤波器
 *
 * 输出给控制/显示用的平滑量：距离、yaw、pitch、板面偏角、位置。
 */

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

struct AimConfig {
  double ema_alpha = 0.35;  // 0~1，越大越跟手、越小越平滑
  int acquire_hits = 3;     // 连续命中多少帧才判定目标有效
  int max_lost = 6;         // 连续丢失多少帧才判定目标丢失
  double jump_distance_m = 0.6;  // 单帧距离跳变门限（米）
  double jump_angle_deg = 40.0;  // 单帧 yaw/pitch 跳变门限（度）
};

struct AimOutput {
  bool valid = false;         // 状态机认为目标有效
  bool measured = false;      // 本帧是否解出有效位姿
  cv::Vec3d tvec{0, 0, 0};    // 平滑后的板中心位置（mm，相机系）
  double distance_m = 0.0;    // 平滑后距离（m）
  double yaw_deg = 0.0;       // 平滑后 yaw（度，右正）
  double pitch_deg = 0.0;     // 平滑后 pitch（度，上正）
  double plate_tilt_deg = 0.0;  // 板面法线与视线夹角（度，0=正对相机）
  double reproj_error_px = 0.0; // 本帧原始重投影误差
  int hits = 0;               // 连续命中帧数
  int lost = 0;               // 连续丢失帧数
};

class PoseFilter {
 public:
  explicit PoseFilter(const AimConfig& cfg = AimConfig()) : cfg_(cfg) {}

  void Reset() {
    have_ = false;
    hits_ = 0;
    lost_ = 0;
    out_ = AimOutput();
  }

  /**
   * 喂一帧结果。
   * @param has_pose 本帧是否解出位姿
   * @param rvec/tvec 本帧原始解（has_pose=false 时忽略）
   * @param reproj   本帧重投影误差
   * @return 平滑后的输出
   */
  const AimOutput& Update(bool has_pose, const cv::Vec3d& rvec,
                          const cv::Vec3d& tvec, double reproj) {
    out_.measured = has_pose;
    out_.reproj_error_px = reproj;

    if (has_pose) {
      ++hits_;
      lost_ = 0;

      // 计算原始观测量
      double yaw = std::atan2(tvec[0], tvec[2]);
      double pitch =
          std::atan2(-tvec[1], std::sqrt(tvec[0] * tvec[0] + tvec[2] * tvec[2]));
      const double dist_m = cv::norm(tvec) / 1000.0;

      // 跳变检测：与上一帧相比突变 → 重置（换目标/误检）
      const bool jump =
          have_ && (std::fabs(dist_m - out_.distance_m) > cfg_.jump_distance_m ||
                    std::fabs(yaw * 180.0 / CV_PI - out_.yaw_deg) >
                        cfg_.jump_angle_deg ||
                    std::fabs(pitch * 180.0 / CV_PI - out_.pitch_deg) >
                        cfg_.jump_angle_deg);

      if (!have_ || jump) {
        out_.tvec = tvec;
        out_.distance_m = dist_m;
        out_.yaw_deg = yaw * 180.0 / CV_PI;
        out_.pitch_deg = pitch * 180.0 / CV_PI;
      } else {
        const double a = cfg_.ema_alpha;
        out_.tvec[0] = (1.0 - a) * out_.tvec[0] + a * tvec[0];
        out_.tvec[1] = (1.0 - a) * out_.tvec[1] + a * tvec[1];
        out_.tvec[2] = (1.0 - a) * out_.tvec[2] + a * tvec[2];
        out_.distance_m = (1.0 - a) * out_.distance_m + a * dist_m;
        out_.yaw_deg = (1.0 - a) * out_.yaw_deg + a * (yaw * 180.0 / CV_PI);
        out_.pitch_deg =
            (1.0 - a) * out_.pitch_deg + a * (pitch * 180.0 / CV_PI);
      }

      // 板面法线与视线夹角：法线 n = R * (1,0,0)，视线方向 = -tvec/|tvec|
      cv::Mat R;
      cv::Rodrigues(rvec, R);
      const cv::Vec3d n(R.at<double>(0, 0), R.at<double>(1, 0),
                        R.at<double>(2, 0));
      const double dist_mm = cv::norm(tvec);
      const cv::Vec3d los(tvec[0] / dist_mm, tvec[1] / dist_mm,
                          tvec[2] / dist_mm);
      const double cos_tilt = -(n[0] * los[0] + n[1] * los[1] + n[2] * los[2]);
      // 用原始 rvec 计算偏角（避免平滑把角度混在一起）
      out_.plate_tilt_deg = std::acos(std::max(-1.0, std::min(1.0, cos_tilt))) *
                            180.0 / CV_PI;

      have_ = true;
    } else {
      if (have_) ++lost_;
    }

    out_.hits = hits_;
    out_.lost = lost_;
    if (hits_ >= cfg_.acquire_hits) {
      out_.valid = true;
    }
    if (have_ && lost_ > cfg_.max_lost) {
      // 丢失：清状态
      have_ = false;
      hits_ = 0;
      out_.valid = false;
    }
    return out_;
  }

  bool valid() const { return out_.valid; }
  const AimOutput& out() const { return out_; }

 private:
  AimConfig cfg_;
  bool have_ = false;
  int hits_ = 0;
  int lost_ = 0;
  AimOutput out_;
};

#endif  // RM27_AIM_FILTER_HPP_
