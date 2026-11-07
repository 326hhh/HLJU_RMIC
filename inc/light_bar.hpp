#ifndef RM27_LIGHT_BAR_HPP_
#define RM27_LIGHT_BAR_HPP_

/**
 * @file  light_bar.hpp
 * @brief 传统 CV 灯条检测器（27RM 校内赛 - 小装甲板）
 *
 * 管线：
 *   1. HSV 颜色掩膜（敌方颜色：默认蓝）→ 闭运算 → 轮廓
 *   2. 轮廓 → LightBar（minAreaRect + 质心 + 上下端点 + 倾角）
 *   3. PCA 角点修正（灰度矩求对称轴 + 亮度梯度找端点；失败时退回矩形端点）
 *   4. 左右配对（倾角差/长度比/中心高度差/中心距比）
 *   5. 输出 4 角点：左下 → 左上 → 右上 → 右下
 *      （与 CameraCalib::solvePose 的模型点顺序一致）
 *
 * 校内赛只有小装甲板（灯条 L=28.05mm、宽 5.85mm），不分类大小。
 */

#include <opencv2/opencv.hpp>

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace rm27 {

/** 检测参数（configs/light_bar.yaml 可覆盖） */
struct LightBarConfig {
  // ---- 颜色（OpenCV HSV：H∈[0,180]，S/V∈[0,255]）----
  std::string enemy_color = "blue";
  int blue_h_min = 100, blue_h_max = 140, blue_s_min = 60, blue_v_min = 60;
  int red_h_min_1 = 0, red_h_max_1 = 12;  // 红色 H 跨两端
  int red_h_min_2 = 168, red_h_max_2 = 180;
  int red_s_min = 60, red_v_min = 60;

  // ---- 形态学 ----
  int morph_kernel = 3;
  int morph_iters = 1;

  // ---- 灯条筛选 ----
  double min_length = 5.0;    // 最小灯条长度（px）。小灯条 28.05mm，
                              // 1.5m 处约 9px，别设太大
  double min_ratio = 1.4;     // 长/宽最小比（阈值化后掩膜会变宽，别设高）
  double max_tilt_deg = 45.0; // 与竖直方向最大夹角
  double min_area = 8.0;      // 最小灯条轮廓面积（px）。28.05mm 灯条 1.5m
                              // 处只有 ~9x2px，面积门限别设 20 以上
  double max_area = 200000.0;

  // ---- 配对 ----
  double max_angle_diff_deg = 15.0;  // 两灯条倾角差
  double max_len_ratio = 2.0;        // 两灯条长度比上限
  double max_center_y_ratio = 0.6;   // 中心纵向差 / 平均长度
  // 中心距 / 平均长度（小装甲 W/L = 67.09/28.05 ≈ 2.39；
  // 板面大角度偏转时投影会把它压小）
  double center_dist_ratio_min = 1.2;
  double center_dist_ratio_max = 3.2;

  // ---- PCA 角点修正（jlu lightbar_corrector 同款参数）----
  bool use_pca = true;
  bool pca_required = false;  // true=两侧灯条必须 PCA 成功才配对
  int pass_optimize_lightbar_width = 2;
  float normalize_max_brightness = 25.0f;
  float lightbar_min_mean_brightness = 0.0f;
  float padding_scale = 0.07f;
  float search_start_ratio = 0.4f;
  float search_end_ratio = 0.6f;
};

/** 一根灯条 */
struct LightBar {
  cv::RotatedRect rect;  // 轮廓最小外接矩形
  cv::Point2f center;    // 质心（比矩形对角线交点稳）
  cv::Point2f top, bottom;  // 端点（图像坐标，y 向下）
  float length = 0.f;       // 端点距离
  float width = 0.f;        // 矩形短边
  cv::Point2f axis;         // bottom→top 单位方向
  float tilt_deg = 0.f;     // 与竖直方向夹角（无符号）
  double signed_tilt = 0.0; // 有符号倾角（正 = 向右倾）
  bool pca_ok = false;      // PCA 修正成功（端点已精修）
};

/** 一块完整装甲板（校内赛只有小装甲） */
struct ArmorTarget {
  LightBar left_light, right_light;
  std::array<cv::Point2f, 4> corners;  // 左下 左上 右上 右下
  cv::Point2f center;
};

class LightBarDetector {
 public:
  LightBarDetector() = default;
  explicit LightBarDetector(const std::string& yaml_path) {
    loadConfig(yaml_path);
  }

  /** 加载参数；文件不存在时保留默认值并打印警告 */
  bool loadConfig(const std::string& yaml_path);

  /**
   * 检测一帧，返回最优目标（多个候选时取灯条平均长度最大者）。
   * @param bgr 原始帧（BGR）
   * @return 没找到时 std::nullopt
   */
  std::optional<ArmorTarget> detect(const cv::Mat& bgr);

  /**
   * 检测一帧，返回所有通过门限的装甲板（用于反小陀螺：同帧可能看到 1~2 块）。
   * @param bgr 原始帧（BGR）
   * @return 空的 vector 表示没找到
   */
  std::vector<ArmorTarget> detectAll(const cv::Mat& bgr);

  const LightBarConfig& config() const { return cfg_; }
  const std::vector<LightBar>& bars() const { return bars_; }
  const cv::Mat& mask() const { return mask_debug_; }

  /** 调试绘制：灯条（黄）、配对矩形与角点（绿） */
  static void drawDebug(cv::Mat& img, const std::vector<LightBar>& bars,
                        const std::optional<ArmorTarget>& armor);

  /** 切换敌方颜色（现场调试用） */
  void setEnemyColor(const std::string& color) { cfg_.enemy_color = color; }

 private:
  std::vector<LightBar> findBars(const cv::Mat& bgr);
  std::optional<ArmorTarget> pairBars(std::vector<LightBar>& bars);
  std::vector<ArmorTarget> pairAllBars(std::vector<LightBar>& bars);

  /** 从轮廓构造灯条（借鉴 jlu types.cpp LightBar(contour)） */
  static LightBar fromContour(const std::vector<cv::Point>& contour);

  /** PCA 角点修正（借鉴 jlu lightbar_corrector）；成功返回 true */
  bool correctBar(const cv::Mat& gray, LightBar& bar);
  std::optional<cv::Point2f> findCorner(const cv::Mat& gray,
                                        const LightBar& bar,
                                        const cv::Point2f& axis_dir,
                                        const cv::Point2f& centroid,
                                        float mean_val, bool toward_top);

  LightBarConfig cfg_;
  std::vector<LightBar> bars_;
  cv::Mat mask_debug_;
};

}  // namespace rm27

#endif  // RM27_LIGHT_BAR_HPP_
