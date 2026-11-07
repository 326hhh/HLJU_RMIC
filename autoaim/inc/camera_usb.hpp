#ifndef RM27_CAMERA_USB_HPP_
#define RM27_CAMERA_USB_HPP_

/**
 * @file  camera_usb.hpp
 * @brief USB(UVC/V4L2) 摄像头封装 —— 27RM 校内赛视觉
 *
 * ⚠️ 为什么不直接用 cv::VideoCapture：
 *   这个 UVC 相机（0c45:6368，640x480 MJPG）用 OpenCV 的 V4L2 后端取流
 *   容易被卡住（读几帧后 select() timeout 直到 USB 重新枚举），而直接
 *   V4L2 ioctl（v4l2-ctl 同款 mmap 方式）稳定 99fps。所以这里自己写。
 *
 * 设计：
 *   - 直采 V4L2：S_FMT(MJPG) → REQBUFS 4 个 mmap 缓冲 → STREAMON
 *   - 后台线程 poll + DQBUF → cv::imdecode → 只保留最新一帧（自动丢旧帧）
 *   - 曝光/白平衡等 V4L2 控制项在【开始流之前】设置，避免流中改控制项
 *     把 UVC 弄僵
 *   - 实测帧率统计（1 秒滑动窗口）
 *
 * 典型用法：
 *   UsbCamera cam;
 *   cam.Open(0, 640, 480, "configs/camera_runtime.yaml");
 *   cv::Mat img; std::chrono::steady_clock::time_point ts;
 *   while (cam.GetLatestFrame(&img, &ts)) { ... }
 */

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

class UsbCamera {
 public:
  UsbCamera() = default;
  ~UsbCamera();

  UsbCamera(const UsbCamera&) = delete;
  UsbCamera& operator=(const UsbCamera&) = delete;

  /**
   * 打开相机并启动采集线程。
   * @param device_index   /dev/videoN 的 N
   * @param width/height   目标分辨率（相机不支持时退回最近的可选）
   * @param runtime_yaml   相机运行时参数文件（曝光/白平衡等）；空串=不设置。
   *                       ⚠️ 这些参数在开始取流之前设置（避免流中改控制项）
   * @return 成功返回 true
   */
  bool Open(int device_index, int width, int height,
            const std::string& runtime_yaml = std::string());
  void Close();
  bool IsOpen() const { return running_; }

  std::string DevicePath() const { return device_path_; }

  /**
   * 取最新一帧（BGR）。
   * @param[out] frame  帧数据（拷贝）
   * @param[out] ts     该帧被读出的时刻（steady_clock）
   * @param[out] seq    帧序号（可用于判断是否为新帧）
   * @return true 表示至少取到过一帧
   */
  bool GetLatestFrame(cv::Mat* frame, std::chrono::steady_clock::time_point* ts,
                      uint64_t* seq = nullptr);

  /** 最近 1 秒窗口的实测帧率 */
  double MeasuredFps() const { return fps_.load(); }

  /** 实际生效的分辨率 / 格式（打开后查询） */
  int Width() const { return actual_width_; }
  int Height() const { return actual_height_; }
  std::string Fourcc() const { return actual_fourcc_; }

  /** 应用运行时参数（必须在开始取流前调用，Open 内部会自动调用一次） */
  bool ApplyRuntimeConfig(const std::string& yaml_path);

  /** 打开前强制采集格式："mjpg" / "yuyv"（空=默认 MJPG 优先） */
  void SetFormat(const std::string& fmt);

  // ---- V4L2 控制项（静态，可对任意 /dev/videoN 使用）----
  static bool SetControl(const std::string& device_path, uint32_t cid,
                         int32_t value);
  static bool GetControl(const std::string& device_path, uint32_t cid,
                         int32_t* value);

 private:
  struct Buf {
    void* start = nullptr;
    size_t length = 0;
  };

  bool ConfigureFormat(int width, int height);
  bool StartStream();
  void CaptureLoop();
  void StopStream();
  // 等连续拿到 min_frames 帧（超时 ms）；返回是否达标
  bool WaitFrames(int min_frames, int timeout_ms);

  int fd_ = -1;
  std::string device_path_;
  std::vector<Buf> bufs_;  // mmap 缓冲
  int actual_width_ = 0;
  int actual_height_ = 0;
  std::string actual_fourcc_;
  bool is_mjpg_ = true;
  uint32_t force_format_ = 0;  // 0=自动（MJPG 优先），或 V4L2_PIX_FMT_*
  bool streaming_ = false;

  std::thread thread_;
  std::atomic<bool> running_{false};

  mutable std::mutex mutex_;
  cv::Mat latest_;
  std::chrono::steady_clock::time_point latest_ts_;
  uint64_t latest_seq_ = 0;

  std::atomic<double> fps_{0.0};
};

#endif  // RM27_CAMERA_USB_HPP_
