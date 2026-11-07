#include "camera_calib.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

ArmorSize LoadArmorSize(const std::string& yaml_path) {
  ArmorSize size;
  {
    std::ifstream f(yaml_path);
    if (!f.good()) {
      std::fprintf(stderr, "[ArmorSize] %s 不存在，使用默认尺寸\n",
                   yaml_path.c_str());
      return size;
    }
  }
  cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::fprintf(stderr, "[ArmorSize] 打不开 %s，使用默认尺寸\n",
                 yaml_path.c_str());
    return size;
  }
  if (!fs["width_mm"].isNone()) fs["width_mm"] >> size.width_mm;
  if (!fs["height_mm"].isNone()) fs["height_mm"] >> size.height_mm;
  if (!fs["bar_width_mm"].isNone()) fs["bar_width_mm"] >> size.bar_width_mm;
  std::printf("[ArmorSize] %s 已加载: 中心距 W=%.2fmm 灯条长 L=%.2fmm 灯条宽=%.2fmm\n",
              yaml_path.c_str(), size.width_mm, size.height_mm,
              size.bar_width_mm);
  return size;
}

bool CameraCalib::load(const std::string& yaml_path) {
  {
    std::ifstream f(yaml_path);
    if (!f.good()) {
      std::fprintf(stderr, "[CameraCalib] 配置文件不存在: %s\n",
                   yaml_path.c_str());
      return false;
    }
  }
  cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::fprintf(stderr, "[CameraCalib] 打不开配置文件: %s\n",
                 yaml_path.c_str());
    return false;
  }
  fs["camera_matrix"] >> camera_matrix_;
  fs["dist_coeffs"] >> dist_coeffs_;
  int w = 0, h = 0;
  fs["image_width"] >> w;
  fs["image_height"] >> h;
  fs.release();

  if (camera_matrix_.empty() || dist_coeffs_.empty()) {
    std::fprintf(stderr, "[CameraCalib] %s 缺少 camera_matrix 或 dist_coeffs\n",
                 yaml_path.c_str());
    return false;
  }
  image_size_ = cv::Size(w, h);
  valid_ = true;
  std::printf("[CameraCalib] %s 已加载: fx=%.2f fy=%.2f cx=%.2f cy=%.2f "
              "(%dx%d)\n",
              yaml_path.c_str(), camera_matrix_.at<double>(0, 0),
              camera_matrix_.at<double>(1, 1), camera_matrix_.at<double>(0, 2),
              camera_matrix_.at<double>(1, 2), w, h);
  return true;
}

bool CameraCalib::loadFromMats(const cv::Mat& K, const cv::Mat& D,
                               const cv::Size& image_size) {
  if (K.empty() || D.empty()) {
    return false;
  }
  camera_matrix_ = K.clone();
  dist_coeffs_ = D.clone();
  image_size_ = image_size;
  valid_ = true;
  std::printf("[CameraCalib] 已设置合成内参: fx=%.2f fy=%.2f (%dx%d)\n",
              camera_matrix_.at<double>(0, 0),
              camera_matrix_.at<double>(1, 1), image_size.width,
              image_size.height);
  return true;
}

void CameraCalib::undistort(const cv::Mat& src, cv::Mat& dst) {  if (!valid_) {
    src.copyTo(dst);
    return;
  }
  if (map1_.empty() || map1_.size() != src.size()) {
    cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_, cv::Mat(),
                                camera_matrix_, src.size(), CV_32FC1, map1_,
                                map2_);
  }
  cv::remap(src, dst, map1_, map2_, cv::INTER_LINEAR);
}

std::optional<ArmorPose> CameraCalib::solvePose(
    const std::array<cv::Point2f, 4>& corners, const ArmorSize& size) const {
  if (!valid_) {
    return std::nullopt;
  }

  // 3D 模型点：x 朝前（装甲板法线），y 向左，z 向上；左下起顺时针
  const double W = size.width_mm;   // 两灯条中心间距
  const double L = size.height_mm;  // 灯条长度
  const std::vector<cv::Point3f> obj_points{
      {0.f, static_cast<float>(W / 2), static_cast<float>(-L / 2)},   // 左下
      {0.f, static_cast<float>(W / 2), static_cast<float>(L / 2)},    // 左上
      {0.f, static_cast<float>(-W / 2), static_cast<float>(L / 2)},   // 右上
      {0.f, static_cast<float>(-W / 2), static_cast<float>(-L / 2)},  // 右下
  };
  const std::vector<cv::Point2f> img_points{corners[0], corners[1],
                                            corners[2], corners[3]};

  // 单个候选解的验收：有限值 → z>0（相机前方）→ 原图重投影门限
  auto tryAccept = [&](const cv::Mat& rv,
                       const cv::Mat& tv) -> std::optional<ArmorPose> {
    const double z = tv.at<double>(2);
    if (!std::isfinite(z) || z <= 0.0) {
      return std::nullopt;  // 镜像解常落在相机后方；NaN 直接丢
    }
    std::vector<cv::Point2f> proj;
    cv::projectPoints(obj_points, rv, tv, camera_matrix_, dist_coeffs_, proj);
    double err = 0.0;
    for (std::size_t k = 0; k < proj.size(); ++k) {
      const double dx = proj[k].x - img_points[k].x;
      const double dy = proj[k].y - img_points[k].y;
      if (!std::isfinite(dx) || !std::isfinite(dy)) {
        return std::nullopt;
      }
      err += std::sqrt(dx * dx + dy * dy);
    }
    err /= static_cast<double>(proj.size());
    if (err > max_reproj_err_px_) {
      return std::nullopt;
    }
    ArmorPose pose;
    pose.rvec =
        cv::Vec3d(rv.at<double>(0), rv.at<double>(1), rv.at<double>(2));
    pose.tvec =
        cv::Vec3d(tv.at<double>(0), tv.at<double>(1), tv.at<double>(2));
    pose.reproj_error_px = err;
    pose.distance_mm = cv::norm(pose.tvec);
    return pose;
  };

  // 路径 A：IPPE 全候选（平面靶标标准做法，一次拿到两个解便于筛镜像）
  std::vector<cv::Mat> rvecs, tvecs;
  const int n = cv::solvePnPGeneric(obj_points, img_points, camera_matrix_,
                                    dist_coeffs_, rvecs, tvecs, false,
                                    cv::SOLVEPNP_IPPE);
  std::optional<ArmorPose> best;
  for (int i = 0; i < n; ++i) {
    auto p = tryAccept(rvecs[i], tvecs[i]);
    if (p && (!best || p->reproj_error_px < best->reproj_error_px)) {
      best = p;
    }
  }
  if (best) {
    return best;
  }

  // 路径 B：IPPE 数值退化（OpenCV 4.x 在「完美轴对称矩形」上返回 NaN、
  // 「畸变+板内旋转」上返回垃圾解）时退回 ITERATIVE，两条退化情形都稳。
  cv::Mat rv, tv;
  if (cv::solvePnP(obj_points, img_points, camera_matrix_, dist_coeffs_, rv, tv,
                   false, cv::SOLVEPNP_ITERATIVE)) {
    return tryAccept(rv, tv);
  }
  return std::nullopt;
}
