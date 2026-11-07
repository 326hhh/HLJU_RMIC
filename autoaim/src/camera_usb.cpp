#include "camera_usb.hpp"

#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

namespace {

constexpr int kBufferCount = 4;

// V4L2 ioctl 包装：EINTR 重试
int Xioctl(int fd, unsigned long request, void* arg) {
  int r = 0;
  do {
    r = ::ioctl(fd, request, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

std::string FourccToString(uint32_t f) {
  char s[5] = {0};
  s[0] = static_cast<char>(f & 0xFF);
  s[1] = static_cast<char>((f >> 8) & 0xFF);
  s[2] = static_cast<char>((f >> 16) & 0xFF);
  s[3] = static_cast<char>((f >> 24) & 0xFF);
  return s;
}

bool ReadInt(const cv::FileStorage& fs, const char* key, int* out) {
  if (fs[key].isNone()) return false;
  fs[key] >> *out;
  return true;
}

// 文件是否存在（避免 cv::FileStorage 对 /dev/null 之类抛异常）
bool FileExists(const std::string& path) {
  std::ifstream f(path);
  return f.good();
}

}  // namespace

UsbCamera::~UsbCamera() { Close(); }

bool UsbCamera::Open(int device_index, int width, int height,
                     const std::string& runtime_yaml) {
  Close();
  device_path_ = "/dev/video" + std::to_string(device_index);

  fd_ = ::open(device_path_.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) {
    std::fprintf(stderr, "[UsbCamera] 打不开 %s: %s\n", device_path_.c_str(),
                 std::strerror(errno));
    return false;
  }

  // 1) 查询能力
  v4l2_capability cap{};
  if (Xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
    std::fprintf(stderr, "[UsbCamera] VIDIOC_QUERYCAP 失败\n");
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    std::fprintf(stderr, "[UsbCamera] %s 不支持视频采集\n",
                 device_path_.c_str());
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // 2) 运行时参数（含 format 覆盖；此时还没开始取流，安全）
  if (!runtime_yaml.empty()) {
    ApplyRuntimeConfig(runtime_yaml);
  }

  // 3) 设格式 + 启动取流；相机 MJPG 档起不来时自动降级 YUYV（同分辨率）
  //    这台 Sonix 相机的 640x480 MJPG（120fps 档）会偶发「首帧后停流」，
  //    只有断电复位能恢复；自动降级能让程序先跑起来（30fps），不影响标定内参。
  const bool auto_format = (force_format_ == 0);
  bool ok = false;
  if (!ConfigureFormat(width, height) || !StartStream()) {
    std::fprintf(stderr, "[UsbCamera] 启动取流失败\n");
    Close();
    return false;
  }
  ok = WaitFrames(5, 2000);
  if (!ok && auto_format && is_mjpg_) {
    std::fprintf(stderr,
                 "[UsbCamera] ⚠️ MJPG 模式 2s 内拿不到 5 帧（相机该档卡死），"
                 "自动降级为 YUYV 同分辨率...\n");
    Close();
    device_path_ = "/dev/video" + std::to_string(device_index);
    fd_ = ::open(device_path_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
      std::fprintf(stderr, "[UsbCamera] 重开 %s 失败\n", device_path_.c_str());
      return false;
    }
    force_format_ = V4L2_PIX_FMT_YUYV;
    if (!ConfigureFormat(width, height) || !StartStream()) {
      Close();
      return false;
    }
    ok = WaitFrames(5, 2000);
    if (!ok) {
      std::fprintf(stderr,
                   "[UsbCamera] ❌ YUYV 也出不了帧：相机需要物理拔插断电复位\n");
      Close();
      return false;
    }
  } else if (!ok) {
    std::fprintf(stderr,
                 "[UsbCamera] ❌ 1.5s 内无帧：相机需要物理拔插断电复位\n");
    Close();
    return false;
  }
  return true;
}

bool UsbCamera::WaitFrames(int min_frames, int timeout_ms) {
  auto t0 = std::chrono::steady_clock::now();
  cv::Mat tmp;
  std::chrono::steady_clock::time_point ts;
  uint64_t seq = 0, last = 0;
  int count = 0;
  while (true) {
    if (GetLatestFrame(&tmp, &ts, &seq)) {
      if (seq != last) {
        last = seq;
        ++count;
        if (count >= min_frames) return true;
      }
    }
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - t0)
            .count() > timeout_ms) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool UsbCamera::ConfigureFormat(int width, int height) {
  // 优先用 force_format_（runtime yaml 指定），否则 MJPG 优先：
  // 640x480 只有 MJPG 能跑 ~99fps（YUYV 只有 30fps）
  uint32_t ordered[2] = {V4L2_PIX_FMT_MJPEG, V4L2_PIX_FMT_YUYV};
  if (force_format_ == V4L2_PIX_FMT_YUYV) {
    ordered[0] = V4L2_PIX_FMT_YUYV;
    ordered[1] = V4L2_PIX_FMT_MJPEG;
  }
  for (int k = 0; k < 2; ++k) {
    const uint32_t pixfmt = ordered[k];
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = static_cast<uint32_t>(width);
    fmt.fmt.pix.height = static_cast<uint32_t>(height);
    fmt.fmt.pix.pixelformat = pixfmt;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (Xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
      continue;
    }
    if (fmt.fmt.pix.pixelformat != pixfmt) {
      continue;  // 驱动改回了别的格式
    }
    actual_width_ = static_cast<int>(fmt.fmt.pix.width);
    actual_height_ = static_cast<int>(fmt.fmt.pix.height);
    actual_fourcc_ = FourccToString(pixfmt);
    is_mjpg_ = (pixfmt == V4L2_PIX_FMT_MJPEG);
    std::printf("[UsbCamera] 格式: %dx%d %s %s\n", actual_width_,
                actual_height_, actual_fourcc_.c_str(),
                is_mjpg_ ? "（MJPG）" : "（YUYV）");
    return true;
  }
  return false;
}

bool UsbCamera::StartStream() {
  v4l2_requestbuffers req{};
  req.count = kBufferCount;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (Xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
    std::fprintf(stderr, "[UsbCamera] VIDIOC_REQBUFS 失败: %s\n",
                 std::strerror(errno));
    return false;
  }
  if (req.count < 2) {
    std::fprintf(stderr, "[UsbCamera] 驱动给的内存缓冲太少 (%u)\n", req.count);
    return false;
  }

  bufs_.resize(req.count);
  for (unsigned i = 0; i < req.count; ++i) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (Xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      std::fprintf(stderr, "[UsbCamera] VIDIOC_QUERYBUF 失败\n");
      return false;
    }
    bufs_[i].length = buf.length;
    bufs_[i].start = ::mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd_, buf.m.offset);
    if (bufs_[i].start == MAP_FAILED) {
      std::fprintf(stderr, "[UsbCamera] mmap 失败\n");
      bufs_[i].start = nullptr;
      return false;
    }
    if (Xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      std::fprintf(stderr, "[UsbCamera] VIDIOC_QBUF 失败\n");
      return false;
    }
  }

  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (Xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    std::fprintf(stderr, "[UsbCamera] VIDIOC_STREAMON 失败: %s\n",
                 std::strerror(errno));
    return false;
  }
  streaming_ = true;

  running_ = true;
  thread_ = std::thread(&UsbCamera::CaptureLoop, this);
  return true;
}

void UsbCamera::CaptureLoop() {
  auto window_start = std::chrono::steady_clock::now();
  int window_count = 0;
  while (running_.load()) {
    struct pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int pr = ::poll(&pfd, 1, 100);
    if (pr <= 0) {
      continue;  // 超时/被打断：继续等（相机空闲时正常）
    }

    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (Xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
      continue;
    }
    if (buf.index >= bufs_.size() || bufs_[buf.index].start == nullptr) {
      continue;
    }

    const auto now = std::chrono::steady_clock::now();
    cv::Mat frame;
    auto* data = static_cast<uint8_t*>(bufs_[buf.index].start);
    const int bytes = static_cast<int>(buf.bytesused);
    if (is_mjpg_) {
      // UVC 的 MJPG 帧本身就是完整 JPEG，直接解码
      if (bytes > 4) {
        const cv::Mat jpg(1, bytes, CV_8UC1, data);
        frame = cv::imdecode(jpg, cv::IMREAD_COLOR);
      }
    } else if (bytes >= actual_width_ * 2) {
      const int rows = bytes / (actual_width_ * 2);
      cv::Mat yuv(rows, actual_width_, CV_8UC2, data);
      cv::cvtColor(yuv, frame, cv::COLOR_YUV2BGR_YUYV);
    }
    if (!frame.empty()) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ = frame;  // 丢旧保新：下游只拿最新
      latest_ts_ = now;
      ++latest_seq_;
    }
    Xioctl(fd_, VIDIOC_QBUF, &buf);

    // 1 s 窗口实测帧率
    ++window_count;
    const auto dt = std::chrono::duration<double>(now - window_start).count();
    if (dt >= 1.0) {
      fps_.store(static_cast<double>(window_count) / dt);
      window_start = now;
      window_count = 0;
    }
  }
}

void UsbCamera::StopStream() {
  if (fd_ >= 0 && streaming_) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    Xioctl(fd_, VIDIOC_STREAMOFF, &type);
    streaming_ = false;
  }
  for (auto& b : bufs_) {
    if (b.start) {
      ::munmap(b.start, b.length);
      b.start = nullptr;
    }
  }
  bufs_.clear();
}

void UsbCamera::Close() {
  if (!running_.exchange(false)) {
    StopStream();
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    return;
  }
  if (thread_.joinable()) {
    thread_.join();
  }
  StopStream();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool UsbCamera::GetLatestFrame(cv::Mat* frame,
                               std::chrono::steady_clock::time_point* ts,
                               uint64_t* seq) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (latest_.empty()) {
    return false;
  }
  if (frame) *frame = latest_.clone();
  if (ts) *ts = latest_ts_;
  if (seq) *seq = latest_seq_;
  return true;
}

// ------------------------------------------------------------------
// V4L2 控制项
// ------------------------------------------------------------------
bool UsbCamera::SetControl(const std::string& device_path, uint32_t cid,
                           int32_t value) {
  const int fd = ::open(device_path.c_str(), O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    std::fprintf(stderr, "[UsbCamera] 打开 %s 失败: %s\n", device_path.c_str(),
                 std::strerror(errno));
    return false;
  }
  v4l2_control ctrl{};
  ctrl.id = cid;
  ctrl.value = value;
  const bool ok = Xioctl(fd, VIDIOC_S_CTRL, &ctrl) == 0;
  if (!ok) {
    std::fprintf(stderr, "[UsbCamera] 设置控制项 0x%08x=%d 失败: %s\n", cid,
                 value, std::strerror(errno));
  }
  ::close(fd);
  return ok;
}

bool UsbCamera::GetControl(const std::string& device_path, uint32_t cid,
                           int32_t* value) {
  const int fd = ::open(device_path.c_str(), O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    return false;
  }
  v4l2_control ctrl{};
  ctrl.id = cid;
  const bool ok = Xioctl(fd, VIDIOC_G_CTRL, &ctrl) == 0;
  if (ok && value) *value = ctrl.value;
  ::close(fd);
  return ok;
}

void UsbCamera::SetFormat(const std::string& fmt) {
  if (fmt == "yuyv" || fmt == "YUYV") {
    force_format_ = V4L2_PIX_FMT_YUYV;
  } else if (fmt == "mjpg" || fmt == "MJPG") {
    force_format_ = V4L2_PIX_FMT_MJPEG;
  } else {
    force_format_ = 0;
  }
}

bool UsbCamera::ApplyRuntimeConfig(const std::string& yaml_path) {
  if (!FileExists(yaml_path)) {
    std::fprintf(stderr, "[UsbCamera] %s 不存在，跳过运行时参数\n",
                 yaml_path.c_str());
    return false;
  }
  cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::fprintf(stderr, "[UsbCamera] 打不开 %s，跳过运行时参数\n",
                 yaml_path.c_str());
    return false;
  }
  if (!fs["format"].isNone()) {
    std::string f;
    fs["format"] >> f;
    if (f == "yuyv" || f == "YUYV") {
      force_format_ = V4L2_PIX_FMT_YUYV;
    } else if (f == "mjpg" || f == "MJPG") {
      force_format_ = V4L2_PIX_FMT_MJPEG;
    }
    std::printf("[UsbCamera] 强制格式: %s\n", f.c_str());
  }
  int v = 0;
  struct Item {
    const char* key;
    uint32_t cid;
  };
  const Item items[] = {
      {"exposure_auto", V4L2_CID_EXPOSURE_AUTO},
      {"exposure_absolute_100us", V4L2_CID_EXPOSURE_ABSOLUTE},
      {"white_balance_auto", V4L2_CID_AUTO_WHITE_BALANCE},
      {"white_balance_temperature", V4L2_CID_WHITE_BALANCE_TEMPERATURE},
      {"brightness", V4L2_CID_BRIGHTNESS},
      {"contrast", V4L2_CID_CONTRAST},
      {"saturation", V4L2_CID_SATURATION},
      {"gamma", V4L2_CID_GAMMA},
      {"sharpness", V4L2_CID_SHARPNESS},
  };
  for (const auto& it : items) {
    if (ReadInt(fs, it.key, &v)) {
      SetControl(device_path_, it.cid, v);
    }
  }
  fs.release();
  std::printf("[UsbCamera] 运行时参数已应用（取流前）: %s\n", yaml_path.c_str());
  return true;
}
