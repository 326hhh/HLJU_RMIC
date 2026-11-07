#ifndef RM27_TARGET_TRACKER_HPP_
#define RM27_TARGET_TRACKER_HPP_

/**
 * @file  target_tracker.hpp
 * @brief 旋转机器人（小陀螺）目标跟踪 —— 27RM「反小陀螺」
 *
 * 模型：敌方机器人身上有 armor_num 块装甲板排成环形（默认 4 块），
 *       中心 = c，半径 R，相位 φ（绕竖直轴），转速 ω。
 *       每块板中心 = c + R * dir(φ + k * 2π/armor_num)。
 *
 * 观测：任意时刻检测到 1 块（或同时 2 块）装甲板即可：
 *   - 由板中心 p 与板面法线 n（指向板外侧 = 远离机器人中心）反推
 *     机器人中心：c = p - R * n（R 已知或双板在线估计）
 *   - 由板的角度获得相位 φ；相位随时间变化率即转速 ω
 *
 * 输出：Predict(t) 给出 t 时刻整圈 4 块板的预测位置；
 *       SelectAimPlate(t) 选出最适合瞄准的板（板面朝向相机 + 接近视线中心）。
 *
 * 滤波器：中心位置/速度用 6 维线性卡尔曼（匀速模型），
 *         相位/转速用 2 维卡尔曼（相位残差按 π 回绕）。
 */

#include <opencv2/opencv.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace rm27 {

/** 装甲板观测（相机坐标系，mm）：板中心 + 板面法线（单位向量，指向板外侧） */
struct PlateObservation {
  cv::Vec3d center_mm{0, 0, 0};
  cv::Vec3d normal{0, 0, 0};  // 使用前会归一化
  double reproj_error_px = 0.0;
};

/** 环模型参数（configs/target.yaml） */
struct TargetConfig {
  double radius_mm = 150.0;             // 机器人中心到板中心的水平距离（实测！）
  int armor_num = 4;                    // 装甲板数量
  double predict_time_s = 0.04;         // 预测前瞻（补偿处理/击发延迟）
  double max_spin_rate_dps = 900.0;     // 最大转速（°/s）
  double center_accel_std = 40.0;       // 中心平动的加速度噪声（mm/s²）
  double phase_accel_std_deg = 120.0;   // 相位角加速度噪声（°/s²）
  double center_meas_std_mm = 12.0;     // 中心位置观测噪声（mm）
  double phase_meas_std_deg = 3.0;      // 相位观测噪声（°）
  double assoc_gate_mm = 400.0;         // 观测与预测板位置的关联门限（mm）
  bool estimate_radius = true;          // 同时看到 2 块板时在线估计半径
};

/** 一块被预测的装甲板（相机坐标系，mm） */
struct PredictedPlate {
  cv::Vec3d center_mm{0, 0, 0};  // 板中心
  cv::Vec3d normal{0, 0, 0};     // 板面法线（指向板外侧）
  double angle_rad = 0.0;        // 绕环的绝对相位（弧度）
  int slot = 0;                  // 板序号（0..armor_num-1）
  double face_score = 0.0;       // 板面朝向相机的程度（越大越正对相机）
};

class TargetTracker {
 public:
  TargetTracker() = default;
  explicit TargetTracker(const std::string& yaml_path) { loadConfig(yaml_path); }

  bool loadConfig(const std::string& yaml_path);

  /**
   * 输入一帧检测结果。同一时刻可多次调用（例如同时看到两块板）。
   * @param obs 板中心与法线（相机系，mm）
   * @param t_s 时间戳（秒，单调递增）
   */
  void Update(const PlateObservation& obs, double t_s);

  /** 预测 t_s 时刻整圈板的位姿（t_s < 最近更新时刻时按当前状态输出） */
  std::vector<PredictedPlate> Predict(double t_s) const;

  /**
   * 选出最适合瞄准的板：板面朝相机 + 最接近视线中心。
   * @return 找到返回 true，out 写入预测板
   */
  bool SelectAimPlate(double t_s, PredictedPlate* out) const;

  bool valid() const { return valid_; }
  cv::Vec3d center() const { return c_; }
  cv::Vec3d center_velocity() const { return v_; }
  double radius_mm() const { return radius_mm_; }
  double spin_rate_dps() const { return omega_ * 180.0 / CV_PI; }
  double phase_deg() const {
    return std::fmod(phi_ * 180.0 / CV_PI + 3600.0, 360.0);
  }
  const TargetConfig& config() const { return cfg_; }

  void Reset();

 private:
  void Init(const PlateObservation& obs, double t_s);
  void PredictState(double t_s);  // 从 last_t_ 推进到 t_s

  /** 环上相位 angle 对应的水平方向单位向量（camera x/z 平面） */
  cv::Vec3d AngleToDir(double angle) const {
    return cv::Vec3d(std::sin(angle), 0.0, std::cos(angle));
  }

  TargetConfig cfg_;
  bool valid_ = false;
  double last_t_ = 0.0;
  double radius_mm_ = 150.0;

  // 状态：中心/速度、相位/转速
  cv::Vec3d c_{0, 0, 0};
  cv::Vec3d v_{0, 0, 0};
  double phi_ = 0.0;   // 绝对相位（未回绕，随时间增长）
  double omega_ = 0.0; // rad/s

  // 协方差
  cv::Mat Pc_;  // 6x6
  cv::Mat Pp_;  // 2x2

  // 同帧内上一条观测（用于双板估半径）
  bool have_prev_obs_ = false;
  PlateObservation prev_obs_;
};

}  // namespace rm27

#endif  // RM27_TARGET_TRACKER_HPP_
