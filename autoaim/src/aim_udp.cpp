#include "aim_udp.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace rm27 {

namespace {

/// 按网卡的 地址 & 掩码 算广播地址（`--aim-udp auto` 用）。
/// @param ifname 网卡名（空 = 优先 wlan0，没有再挑第一块带 IFF_BROADCAST 的）
/// @param out    输出：网络字节序的广播地址
/// @param used   输出：实际用的网卡名 + 算出来的点分地址（打日志用）
bool ifaceBroadcast(const std::string& ifname, uint32_t* out, std::string* used,
                    std::string* text) {
  struct ifaddrs* ifs = nullptr;
  if (::getifaddrs(&ifs) != 0) return false;
  bool ok = false;
  for (int pass = 0; pass < 2 && !ok; ++pass) {
    // pass 0：只找 wlan0（默认）；pass 1：任选一块能广播的网卡
    for (struct ifaddrs* it = ifs; it && !ok; it = it->ifa_next) {
      if (!it->ifa_addr || it->ifa_addr->sa_family != AF_INET) continue;
      if (!(it->ifa_flags & IFF_UP) || !(it->ifa_flags & IFF_BROADCAST)) continue;
      const std::string name = ifname.empty() ? std::string("wlan0") : ifname;
      if (pass == 0 && name != it->ifa_name) continue;
      const uint32_t ip =
          reinterpret_cast<struct sockaddr_in*>(it->ifa_addr)->sin_addr.s_addr;
      uint32_t mask = 0xFFFFFFFFu;
      if (it->ifa_netmask)
        mask = reinterpret_cast<struct sockaddr_in*>(it->ifa_netmask)
                   ->sin_addr.s_addr;
      *out  = (ip & mask) | ~mask;
      *used = it->ifa_name;
      char buf[INET_ADDRSTRLEN] = {0};
      ::inet_ntop(AF_INET, out, buf, sizeof(buf));
      *text = buf;
      ok    = true;
    }
  }
  ::freeifaddrs(ifs);
  return ok;
}

}  // namespace

AimUdpSender::~AimUdpSender() { Stop(); }

bool AimUdpSender::Start(const std::string& target) {
  Stop();
  if (target.empty() || target == "off" || target == "0") {
    std::printf("[自瞄UDP] 已关闭\n");
    return false;
  }

  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    std::fprintf(stderr, "[自瞄UDP] socket 失败: %s\n", std::strerror(errno));
    return false;
  }
  int one = 1;
  ::setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));

  // 逗号分隔多目标：有线网段广播 + 热点网段广播可以同时发
  //   --aim-udp "192.168.2.255:40002,10.42.0.255:40002"
  targets_.clear();
  size_t begin = 0;
  while (begin <= target.size()) {
    const size_t comma = target.find(',', begin);
    std::string item = target.substr(
        begin, comma == std::string::npos ? std::string::npos : comma - begin);
    // 去掉首尾空白
    while (!item.empty() && (item.front() == ' ' || item.front() == '\t'))
      item.erase(item.begin());
    while (!item.empty() && (item.back() == ' ' || item.back() == '\t'))
      item.pop_back();

    if (!item.empty()) {
      std::string host = item;
      int port = 40002;
      // auto / auto@网卡：按网卡掩码算广播地址（换网段/换路由器不用改命令行）
      if (item == "auto" || item.rfind("auto@", 0) == 0) {
        const std::string ifn =
            (item == "auto") ? std::string() : item.substr(5);
        uint32_t ip_be = 0;
        std::string used, text;
        if (ifaceBroadcast(ifn, &ip_be, &used, &text)) {
          Target t;
          t.ip_be   = ip_be;
          t.port_be = htons(40002);
          targets_.push_back(t);
          std::printf("[自瞄UDP] auto -> %s:40002（网卡 %s 的广播地址）\n",
                      text.c_str(), used.c_str());
        } else {
          std::fprintf(stderr,
                       "[自瞄UDP] auto 失败：找不到可用网卡（%s）\n",
                       ifn.empty() ? "默认 wlan0" : ifn.c_str());
        }
        if (comma == std::string::npos) break;
        begin = comma + 1;
        continue;
      }
      const auto pos = item.rfind(':');
      if (pos != std::string::npos) {
        host = item.substr(0, pos);
        port = std::atoi(item.c_str() + pos + 1);
      }
      if (host.empty()) host = "255.255.255.255";
      if (port <= 0 || port > 65535) port = 40002;

      Target t;
      t.port_be = htons(static_cast<uint16_t>(port));
      if (::inet_pton(AF_INET, host.c_str(), &t.ip_be) != 1) {
        std::fprintf(stderr, "[自瞄UDP] 跳过非法地址: %s（用 IP）\n", host.c_str());
      } else {
        targets_.push_back(t);
        std::printf("[自瞄UDP] 自瞄数据输出到 %s:%d\n", host.c_str(), port);
      }
    }
    if (comma == std::string::npos) break;
    begin = comma + 1;
  }

  if (targets_.empty()) {
    std::fprintf(stderr, "[自瞄UDP] 没有合法目标，发送端不启动\n");
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  std::printf("[自瞄UDP] 共 %zu 个目标，50 Hz，48 字节/包\n", targets_.size());
  return true;
}

void AimUdpSender::Stop() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void AimUdpSender::Send(AimPacket* pkt) {
  if (fd_ < 0 || !pkt) return;
  static uint32_t seq = 0;
  pkt->magic = kAimMagic;
  pkt->seq = ++seq;
  for (const Target& t : targets_) {
    sockaddr_in to{};
    to.sin_family      = AF_INET;
    to.sin_addr.s_addr = t.ip_be;
    to.sin_port        = t.port_be;
    ::sendto(fd_, pkt, sizeof(*pkt), 0, reinterpret_cast<sockaddr*>(&to),
             sizeof(to));
  }
}

}  // namespace rm27
