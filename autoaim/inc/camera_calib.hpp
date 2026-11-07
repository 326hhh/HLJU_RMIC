#ifndef RM27_CAMERA_CALIB_HPP_
#define RM27_CAMERA_CALIB_HPP_

/**
 * @file  camera_calib.hpp
 * @brief 相机标定数据（内参/畸变）加载 + 装甲板 PnP 位姿解算 —— 27RM
 *
 * 配置来源：configs/camera.yaml（由 ./calibrate 生成）
 *
 * 典型用法：
 *   CameraCalib calib;
 *   if (!calib.load("configs/camera.yaml")) return -1;
 *   cv::Mat undist;
 *   calib.undistort(frame, undist);
 *   // 装甲板四角点（左下、左上、右上、右下）→ 位姿
 *   std::array<cv::Point2f, 4> corners{lb, lt, rt, rb};
 *   if (auto pose = calib.solvePose(corners, armor_size)) {
 *       double yaw, pitch;
 *       CameraCalib::poseToYawPitch(pose->tvec, yaw, pitch);
 *   }
 */

#include <opencv2/opencv.hpp>

#include <array>
#include <optional>
#include <string>

/** 装甲板物理尺寸（毫米）：width = 两灯条中心间距，height = 灯条长度 */
struct ArmorSize {
  double width_mm = 67.40;   // 两灯条中心距（实测，见 configs/armor.yaml）
  double height_mm = 28.05;  // 灯条长度
  double bar_width_mm = 5.85;  // 灯条宽度（检测门限/合成图用）
};

/** 位姿解算结果（相机坐标系，单位 mm） */
struct ArmorPose {
  cv::Vec3d rvec{0, 0, 0};       // 旋转向量（模型 → 相机）
  cv::Vec3d tvec{0, 0, 0};       // 装甲板中心位置
  double distance_mm = 0.0;      // 到装甲板中心的距离
  double reproj_error_px = 0.0;  // 重投影误差（质量指标）
};

class CameraCalib {
 public:
  CameraCalib() = default;
  explicit CameraCalib(const std::string& yaml_path) { load(yaml_path); }

  /** 加载标定文件；成功返回 true（失败会打印原因） */
  bool load(const std::string& yaml_path);

  /** 直接设置内参/畸变（自检/合成用） */
  bool loadFromMats(const cv::Mat& K, const cv::Mat& D,
                    const cv::Size& image_size);

  bool valid() const { return valid_; }
  const cv::Mat& K() const { return camera_matrix_; }
  const cv::Mat& D() const { return dist_coeffs_; }
  cv::Size imageSize() const { return image_size_; }

  /** 每帧去畸变（内部缓存映射表，尺寸不变时不用重建） */
  void undistort(const cv::Mat& src, cv::Mat& dst);

  /** 重投影误差门限（像素）：超过则视为无效解，默认 8.0 */
  void setMaxReprojError(double px) { max_reproj_err_px_ = px; }

  /**
   * 四个角点 → 装甲板位姿
   * 内部流程：solvePnPGeneric(IPPE) 取回全部候选解 → 筛掉
   * 「非有限值」「z<=0（相机后方）」「重投影超差」→ 取重投影最小者；
   * 若 IPPE 数值退化（全被筛掉），自动退回 SOLVEPNP_ITERATIVE。
   * @param corners 图像坐标（原图，含畸变），顺序：左下、左上、右上、右下
   * @param size    装甲板物理尺寸
   * @return 成功返回位姿；全部候选被筛掉时返回 std::nullopt
   */
  std::optional<ArmorPose> solvePose(const std::array<cv::Point2f, 4>& corners,
                                     const ArmorSize& size) const;

  /** 位置 → 云台角度（弧度）：yaw 向右为正、pitch 向上为正 */
  static void poseToYawPitch(const cv::Vec3d& tvec, double* yaw, double* pitch) {
    *yaw = std::atan2(tvec[0], tvec[2]);
    *pitch = std::atan2(-tvec[1], std::sqrt(tvec[0] * tvec[0] +
                                           tvec[2] * tvec[2]));
  }

 private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Size image_size_{0, 0};
  bool valid_ = false;
  double max_reproj_err_px_ = 8.0;

  cv::Mat map1_, map2_;  // 去畸变映射表缓存
};

/** 从 armor.yaml 读取装甲板尺寸（读失败时保留默认值并打警告） */
ArmorSize LoadArmorSize(const std::string& yaml_path);

#endif  // RM27_CAMERA_CALIB_HPP_
