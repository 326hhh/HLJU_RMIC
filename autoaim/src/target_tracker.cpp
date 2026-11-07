#include "target_tracker.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>

namespace rm27 {

namespace {

constexpr double kPi = 3.14159265358979323846;

double WrapToPi(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

bool ReadDouble(const cv::FileStorage& fs, const char* key, double* out) {
  if (fs[key].isNone()) return false;
  fs[key] >> *out;
  return true;
}

bool ReadInt(const cv::FileStorage& fs, const char* key, int* out) {
  if (fs[key].isNone()) return false;
  fs[key] >> *out;
  return true;
}

}  // namespace

bool TargetTracker::loadConfig(const std::string& yaml_path) {
  std::ifstream f(yaml_path);
  if (!f.good()) {
    std::fprintf(stderr, "[Target] 参数文件不存在: %s，使用默认值\n",
                 yaml_path.c_str());
    return false;
  }
  cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::fprintf(stderr, "[Target] 打不开 %s，使用默认值\n", yaml_path.c_str());
    return false;
  }
  ReadDouble(fs, "radius_mm", &cfg_.radius_mm);
  ReadInt(fs, "armor_num", &cfg_.armor_num);
  ReadDouble(fs, "predict_time_s", &cfg_.predict_time_s);
  ReadDouble(fs, "max_spin_rate_dps", &cfg_.max_spin_rate_dps);
  ReadDouble(fs, "center_accel_std", &cfg_.center_accel_std);
  ReadDouble(fs, "phase_accel_std_deg", &cfg_.phase_accel_std_deg);
  ReadDouble(fs, "center_meas_std_mm", &cfg_.center_meas_std_mm);
  ReadDouble(fs, "phase_meas_std_deg", &cfg_.phase_meas_std_deg);
  ReadDouble(fs, "assoc_gate_mm", &cfg_.assoc_gate_mm);
  if (!fs["estimate_radius"].isNone()) {
    int v = 1;
    fs["estimate_radius"] >> v;
    cfg_.estimate_radius = (v != 0);
  }
  fs.release();
  if (cfg_.armor_num < 1) cfg_.armor_num = 4;
  radius_mm_ = cfg_.radius_mm;
  std::printf("[Target] %s 已加载: R=%.1fmm 板数=%d 前瞻=%.0fms\n",
              yaml_path.c_str(), cfg_.radius_mm, cfg_.armor_num,
              cfg_.predict_time_s * 1000.0);
  return true;
}

void TargetTracker::Reset() {
  valid_ = false;
  last_t_ = 0.0;
  radius_mm_ = cfg_.radius_mm;
  c_ = cv::Vec3d(0, 0, 0);
  v_ = cv::Vec3d(0, 0, 0);
  phi_ = 0.0;
  omega_ = 0.0;
  have_prev_obs_ = false;
  Pc_.release();
  Pp_.release();
}

void TargetTracker::Init(const PlateObservation& obs, double t_s) {
  cv::Vec3d n = obs.normal;
  n /= (cv::norm(n) + 1e-12);
  radius_mm_ = cfg_.radius_mm;
  c_ = obs.center_mm - radius_mm_ * n;
  v_ = cv::Vec3d(0, 0, 0);

  const cv::Vec3d d = obs.center_mm - c_;
  phi_ = std::atan2(d[0], d[2]);
  omega_ = 0.0;

  Pc_ = cv::Mat::zeros(6, 6, CV_64F);
  for (int i = 0; i < 6; ++i) Pc_.at<double>(i, i) = 1e6;
  Pp_ = cv::Mat::zeros(2, 2, CV_64F);
  Pp_.at<double>(0, 0) = std::pow(30.0 * kPi / 180.0, 2.0);   // 相位 30°
  Pp_.at<double>(1, 1) = std::pow(300.0 * kPi / 180.0, 2.0);  // 转速 300°/s

  last_t_ = t_s;
  valid_ = true;
  have_prev_obs_ = true;
  prev_obs_ = obs;
}

void TargetTracker::PredictState(double t_s) {
  const double dt = t_s - last_t_;
  if (dt <= 1e-9) return;

  // ---- 中心/速度（匀速模型）----
  cv::Mat x = (cv::Mat_<double>(6, 1) << c_[0], c_[1], c_[2], v_[0], v_[1],
               v_[2]);
  cv::Mat F = cv::Mat::eye(6, 6, CV_64F);
  F.at<double>(0, 3) = F.at<double>(1, 4) = F.at<double>(2, 5) = dt;
  const double sa2 = cfg_.center_accel_std * cfg_.center_accel_std;
  const double a = std::pow(dt, 4.0) / 4.0, b = std::pow(dt, 3.0) / 2.0,
               cq = dt * dt;
  cv::Mat Q = cv::Mat::zeros(6, 6, CV_64F);
  for (int i = 0; i < 3; ++i) {
    Q.at<double>(i, i) = a * sa2;
    Q.at<double>(i, i + 3) = Q.at<double>(i + 3, i) = b * sa2;
    Q.at<double>(i + 3, i + 3) = cq * sa2;
  }
  x = F * x;
  Pc_ = F * Pc_ * F.t() + Q;
  c_ = cv::Vec3d(x.at<double>(0), x.at<double>(1), x.at<double>(2));
  v_ = cv::Vec3d(x.at<double>(3), x.at<double>(4), x.at<double>(5));

  // ---- 相位/转速（匀速模型）----
  cv::Mat x2 = (cv::Mat_<double>(2, 1) << phi_, omega_);
  cv::Mat F2 = (cv::Mat_<double>(2, 2) << 1.0, dt, 0.0, 1.0);
  const double sw2 =
      std::pow(cfg_.phase_accel_std_deg * kPi / 180.0, 2.0);
  cv::Mat Q2 = (cv::Mat_<double>(2, 2) << a * sw2, b * sw2, b * sw2, cq * sw2);
  x2 = F2 * x2;
  Pp_ = F2 * Pp_ * F2.t() + Q2;
  phi_ = x2.at<double>(0);
  omega_ = x2.at<double>(1);
  last_t_ = t_s;
}

void TargetTracker::Update(const PlateObservation& obs, double t_s) {
  cv::Vec3d n = obs.normal;
  const double nn = cv::norm(n);
  if (nn < 1e-6) return;
  n /= nn;

  if (!valid_) {
    Init(obs, t_s);
    return;
  }
  double dt = t_s - last_t_;
  if (dt < -1e-6 || dt > 2.0) {  // 时间跳变/长时间断流：重新起跟
    Reset();
    Init(obs, t_s);
    return;
  }
  if (dt > 1e-9) PredictState(t_s);

  // ---- 双板同时可见时在线估计半径（同帧内第二条观测）----
  if (cfg_.estimate_radius && have_prev_obs_ && std::abs(dt) < 1e-6) {
    cv::Vec3d n_prev = prev_obs_.normal;
    n_prev /= (cv::norm(n_prev) + 1e-12);
    const cv::Vec3d dn = n - n_prev;
    const double dn2 = dn.dot(dn);
    if (dn2 > 0.09) {  // 两板法线夹角 > 17°
      const cv::Vec3d dp = obs.center_mm - prev_obs_.center_mm;
      const double r_est = dn.dot(dp) / dn2;
      if (std::isfinite(r_est) && r_est > 0.5 * cfg_.radius_mm &&
          r_est < 2.0 * cfg_.radius_mm) {
        radius_mm_ = 0.8 * radius_mm_ + 0.2 * r_est;
      }
    }
  }

  // ---- 关联：观测板中心 vs 预测板中心 ----
  const double step = 2.0 * kPi / cfg_.armor_num;
  int slot = 0;
  double best_d = 1e18;
  for (int k = 0; k < cfg_.armor_num; ++k) {
    const double ang = phi_ + k * step;
    const cv::Vec3d pc = c_ + radius_mm_ * AngleToDir(ang);
    const double d = cv::norm(obs.center_mm - pc);
    if (d < best_d) {
      best_d = d;
      slot = k;
    }
  }
  have_prev_obs_ = true;
  prev_obs_ = obs;
  if (best_d > cfg_.assoc_gate_mm) {
    last_t_ = t_s;
    return;  // 关联失败：只用预测，不更新
  }

  // ---- 中心位置卡尔曼更新 ----
  {
    const cv::Vec3d c_meas = obs.center_mm - radius_mm_ * n;
    cv::Mat H = cv::Mat::zeros(3, 6, CV_64F);
    H.at<double>(0, 0) = H.at<double>(1, 1) = H.at<double>(2, 2) = 1.0;
    cv::Mat Rm = cv::Mat::eye(3, 3, CV_64F) *
                 (cfg_.center_meas_std_mm * cfg_.center_meas_std_mm);
    cv::Mat x = (cv::Mat_<double>(6, 1) << c_[0], c_[1], c_[2], v_[0], v_[1],
                 v_[2]);
    cv::Mat z = (cv::Mat_<double>(3, 1) << c_meas[0], c_meas[1], c_meas[2]);
    cv::Mat S = H * Pc_ * H.t() + Rm;
    cv::Mat K = Pc_ * H.t() * S.inv();
    x = x + K * (z - H * x);
    Pc_ = (cv::Mat::eye(6, 6, CV_64F) - K * H) * Pc_;
    c_ = cv::Vec3d(x.at<double>(0), x.at<double>(1), x.at<double>(2));
    v_ = cv::Vec3d(x.at<double>(3), x.at<double>(4), x.at<double>(5));
  }

  // ---- 相位卡尔曼更新（残差按 π 回绕）----
  {
    const cv::Vec3d dvec = obs.center_mm - c_;
    const double alpha = std::atan2(dvec[0], dvec[2]);
    const double theta_pred = phi_ + slot * step;
    const double r = WrapToPi(alpha - theta_pred);
    cv::Mat H = (cv::Mat_<double>(1, 2) << 1.0, 0.0);
    cv::Mat Rm = cv::Mat::eye(1, 1, CV_64F) *
                 std::pow(cfg_.phase_meas_std_deg * kPi / 180.0, 2.0);
    cv::Mat S = H * Pp_ * H.t() + Rm;
    cv::Mat K = Pp_ * H.t() / S.at<double>(0, 0);  // 2x1
    phi_ += K.at<double>(0, 0) * r;
    omega_ += K.at<double>(1, 0) * r;
    const double max_w = cfg_.max_spin_rate_dps * kPi / 180.0;
    if (omega_ > max_w) omega_ = max_w;
    if (omega_ < -max_w) omega_ = -max_w;
    Pp_ = (cv::Mat::eye(2, 2, CV_64F) - K * H) * Pp_;
  }
  last_t_ = t_s;
}

std::vector<PredictedPlate> TargetTracker::Predict(double t_s) const {
  std::vector<PredictedPlate> out;
  if (!valid_) return out;

  const double dt = std::max(0.0, t_s - last_t_);
  const cv::Vec3d c = c_ + v_ * dt;
  const double phi = phi_ + omega_ * dt;
  const double step = 2.0 * kPi / cfg_.armor_num;
  const double dist_c = cv::norm(c);

  for (int k = 0; k < cfg_.armor_num; ++k) {
    PredictedPlate p;
    p.slot = k;
    p.angle_rad = phi + k * step;
    p.normal = AngleToDir(p.angle_rad);
    p.center_mm = c + radius_mm_ * p.normal;
    const double dn = cv::norm(p.center_mm);
    double face = 0.0;
    if (dn > 1e-6) {
      face = -(p.normal[0] * p.center_mm[0] + p.normal[2] * p.center_mm[2]) /
             dn;
    }
    p.face_score = face;
    out.push_back(p);
  }
  (void)dist_c;
  return out;
}

bool TargetTracker::SelectAimPlate(double t_s, PredictedPlate* out) const {
  if (!valid_ || !out) return false;
  const std::vector<PredictedPlate> plates = Predict(t_s);
  if (plates.empty()) return false;
  const PredictedPlate* best = &plates[0];
  for (const auto& p : plates) {
    if (p.face_score > best->face_score) best = &p;
  }
  *out = *best;
  return true;
}

}  // namespace rm27
