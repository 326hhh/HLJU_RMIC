#include "mjpeg_stream.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <cstdio>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

namespace rm27 {

MjpegStreamer::MjpegStreamer(int port, int fps, std::string bind)
    : port_(port), fps_(fps), bind_(std::move(bind)) {
  if (fps_ < 1) fps_ = 30;
  if (fps_ > 60) fps_ = 60;
}

MjpegStreamer::~MjpegStreamer() { Stop(); }

bool MjpegStreamer::Start() {
  if (running_.load()) return true;
  listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ < 0) {
    std::fprintf(stderr, "[图传] socket 失败: %s\n", std::strerror(errno));
    return false;
  }
  int one = 1;
  ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  // 监听地址：空 = 所有网卡；否则解析成 IPv4（支持网卡名，DHCP 换 IP 也不怕）
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (!bind_.empty() && bind_ != "0.0.0.0") {
    uint32_t ip_be = 0;
    if (::inet_pton(AF_INET, bind_.c_str(), &ip_be) == 1) {
      addr.sin_addr.s_addr = ip_be;
    } else {
      // 当成网卡名，用 getifaddrs 找它的 IPv4
      struct ifaddrs* ifs = nullptr;
      if (::getifaddrs(&ifs) == 0) {
        for (struct ifaddrs* it = ifs; it; it = it->ifa_next) {
          if (!it->ifa_addr || it->ifa_addr->sa_family != AF_INET) continue;
          if (bind_ != it->ifa_name) continue;
          addr.sin_addr =
              reinterpret_cast<struct sockaddr_in*>(it->ifa_addr)->sin_addr;
          ip_be = addr.sin_addr.s_addr;
          break;
        }
        ::freeifaddrs(ifs);
      }
      if (ip_be == 0) {
        std::fprintf(stderr,
                     "[图传] 找不到地址/网卡 '%s'，退回所有网卡监听\n", bind_.c_str());
      }
    }
    if (addr.sin_addr.s_addr != htonl(INADDR_ANY)) {
      char buf[INET_ADDRSTRLEN] = {0};
      ::inet_ntop(AF_INET, &addr.sin_addr, buf, sizeof(buf));
      std::printf("[图传] 只监听 %s:%d（%s）\n", buf, port_, bind_.c_str());
    }
  }
  addr.sin_port = htons(static_cast<uint16_t>(port_));
  if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) <
      0) {
    std::fprintf(stderr, "[图传] 绑定 :%d 失败（端口被占用？）: %s\n", port_,
                 std::strerror(errno));
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  if (::listen(listen_fd_, 8) < 0) {
    std::fprintf(stderr, "[图传] listen 失败: %s\n", std::strerror(errno));
    ::close(listen_fd_);
    listen_fd_ = -1;
    return false;
  }
  running_ = true;
  accept_thread_ = std::thread(&MjpegStreamer::AcceptLoop, this);
  std::printf("[图传] MJPEG 服务已启动: http://<jetson-ip>:%d/stream （%d fps）\n",
              port_, fps_);
  return true;
}

void MjpegStreamer::Stop() {
  if (!running_.exchange(false)) return;
  if (listen_fd_ >= 0) {
    ::close(listen_fd_);
    listen_fd_ = -1;
  }
  {
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for (int fd : client_fds_) ::close(fd);
    client_fds_.clear();
  }
  clients_ = 0;
  cv_.notify_all();
  if (accept_thread_.joinable()) accept_thread_.join();
}

void MjpegStreamer::PublishJpeg(const std::vector<uint8_t>& jpeg) {
  if (jpeg.size() < 4) return;
  std::lock_guard<std::mutex> lk(mtx_);
  latest_ = std::make_shared<std::vector<uint8_t>>(jpeg);
  ++seq_;
  cv_.notify_all();
}

void MjpegStreamer::AcceptLoop() {
  while (running_.load()) {
    sockaddr_in cli{};
    socklen_t len = sizeof(cli);
    const int fd =
        ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&cli), &len);
    if (fd < 0) {
      if (errno == EINTR) continue;
      if (!running_.load()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    {
      std::lock_guard<std::mutex> lk(clients_mtx_);
      client_fds_.push_back(fd);
    }
    ++clients_;
    std::thread(&MjpegStreamer::ClientLoop, this, fd).detach();
  }
}

bool MjpegStreamer::WriteAll(int fd, const void* data, size_t len) {
  const auto* p = static_cast<const uint8_t*>(data);
  while (len > 0) {
    const ssize_t n = ::send(fd, p, len, MSG_NOSIGNAL);
    if (n <= 0) {
      if (n < 0 && errno == EINTR) continue;
      return false;
    }
    p += n;
    len -= static_cast<size_t>(n);
  }
  return true;
}

void MjpegStreamer::CloseClient(int fd) {
  {
    std::lock_guard<std::mutex> lk(clients_mtx_);
    for (size_t i = 0; i < client_fds_.size(); ++i) {
      if (client_fds_[i] == fd) {
        client_fds_.erase(client_fds_.begin() + i);
        break;
      }
    }
  }
  ::close(fd);
  --clients_;
}

void MjpegStreamer::ClientLoop(int fd) {
  const char* hdr =
      "HTTP/1.0 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Cache-Control: no-cache\r\n"
      "Pragma: no-cache\r\n"
      "Connection: close\r\n\r\n";
  if (!WriteAll(fd, hdr, std::strlen(hdr))) {
    CloseClient(fd);
    return;
  }

  const double period = 1.0 / fps_;
  uint64_t last_seq = 0;
  auto next_send = std::chrono::steady_clock::now();
  char part_hdr[128];

  while (running_.load()) {
    std::shared_ptr<std::vector<uint8_t>> jpg;
    uint64_t seq = 0;
    {
      std::unique_lock<std::mutex> lk(mtx_);
      cv_.wait_for(lk, std::chrono::milliseconds(200), [&] {
        return !running_.load() || (latest_ && seq_ != last_seq);
      });
      if (!running_.load()) break;
      if (!latest_ || seq_ == last_seq) continue;
      jpg = latest_;
      seq = seq_;
    }

    // 限速：保证客户端最多 fps 帧/秒（相机原始帧率可能更高，直接丢旧帧）
    const auto now = std::chrono::steady_clock::now();
    if (now < next_send) {
      std::this_thread::sleep_until(next_send);
    }
    const int n = std::snprintf(part_hdr, sizeof(part_hdr),
                                "--frame\r\nContent-Type: image/jpeg\r\n"
                                "Content-Length: %zu\r\n\r\n",
                                jpg->size());
    if (!WriteAll(fd, part_hdr, static_cast<size_t>(n)) ||
        !WriteAll(fd, jpg->data(), jpg->size()) ||
        !WriteAll(fd, "\r\n", 2)) {
      break;  // 客户端断开
    }
    last_seq = seq;
    next_send = std::chrono::steady_clock::now() +
                std::chrono::milliseconds(static_cast<int>(period * 1000.0));
  }
  CloseClient(fd);
}

}  // namespace rm27
