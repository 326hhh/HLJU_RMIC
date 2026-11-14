#ifndef RM27_MJPEG_STREAM_HPP_
#define RM27_MJPEG_STREAM_HPP_

/**
 * @file  mjpeg_stream.hpp
 * @brief 内嵌 MJPEG-over-HTTP 图传服务器 —— 27RM
 *
 * 27RM 占着相机运行时，控制端（gimbal_ctl / 浏览器）无法用别的程序拉流，
 * 所以直接在 27RM 里起一个小 HTTP 服务：把相机原始 MJPEG 帧按 30fps 发给
 * 客户端（零转码）。
 *
 * 用法（gimbal_ctl 默认就是拉 http://<jetson>:8080/stream）：
 *   rm27::MjpegStreamer s(8080, 30);
 *   s.Start();
 *   // 主循环每来一帧：
 *   s.PublishJpeg(raw_jpeg_bytes);   // 只保留最新一帧；客户端自动限速 30fps
 *
 * 浏览器直接打开 http://<jetson>:8080/stream 也能看。
 */

#include <cstdint>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rm27 {

class MjpegStreamer {
 public:
  /// @param bind 监听地址：空串/"0.0.0.0" = 所有网卡；
  ///             也可以是 IPv4 或**网卡名**（如 "wlan0"），只在该网卡上监听
  MjpegStreamer(int port, int fps, std::string bind = "");
  ~MjpegStreamer();

  MjpegStreamer(const MjpegStreamer&) = delete;
  MjpegStreamer& operator=(const MjpegStreamer&) = delete;

  /** 启动 HTTP 服务；端口被占用/创建失败返回 false（不影响主程序） */
  bool Start();
  void Stop();

  /** 发布一帧 MJPEG 字节（调用方负责只传最新帧），内部拷贝并通知客户端 */
  void PublishJpeg(const std::vector<uint8_t>& jpeg);

  int client_count() const { return clients_.load(); }

 private:
  void AcceptLoop();
  void ClientLoop(int fd);
  bool WriteAll(int fd, const void* data, size_t len);
  void CloseClient(int fd);

  int         port_ = 8080;
  int         fps_ = 30;
  std::string bind_;   ///< 监听地址（空 = INADDR_ANY；可给 IP 或网卡名）
  int listen_fd_ = -1;
  std::atomic<bool> running_{false};
  std::atomic<int> clients_{0};

  std::thread accept_thread_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::shared_ptr<std::vector<uint8_t>> latest_;
  uint64_t seq_ = 0;

  std::mutex clients_mtx_;
  std::vector<int> client_fds_;
};

}  // namespace rm27

#endif  // RM27_MJPEG_STREAM_HPP_
