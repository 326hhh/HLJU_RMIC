#include "gimbal_cmd.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace rm27 {

namespace {

constexpr uint32_t kCtrlMagic = 0x4D454341u;  // 与 control_link.hpp 一致
constexpr uint8_t kFlagEnable = 0x01;
constexpr uint8_t kFlagDisable = 0x40;

void PutU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
void PutU32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}
void PutI16(uint8_t* p, int16_t v) {
  PutU16(p, static_cast<uint16_t>(v));
}

int16_t ClampCdps(double dps) {
  const double v = std::round(dps * 100.0);
  if (v > 32767.0) return 32767;
  if (v < -32768.0) return -32768;
  return static_cast<int16_t>(v);
}

}  // namespace

GimbalCommandSender::~GimbalCommandSender() { Stop(); }

bool GimbalCommandSender::Start(const std::string& host, int port) {
  Stop();
  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    std::fprintf(stderr, "[云台指令] socket 失败: %s\n", std::strerror(errno));
    return false;
  }
  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_port = htons(static_cast<uint16_t>(port));
  if (::inet_pton(AF_INET, host.c_str(), &to.sin_addr) != 1) {
    std::fprintf(stderr, "[云台指令] 非法地址: %s\n", host.c_str());
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  addr_be_ = to.sin_addr.s_addr;
  port_be_ = to.sin_port;
  std::printf("[云台指令] 闭环自瞄已开启 → UDP %s:%d（%s）\n", host.c_str(),
              port, "使能+角速度跟踪");
  return true;
}

void GimbalCommandSender::Stop() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void GimbalCommandSender::Send(double yaw_dps, double pitch_dps,
                               bool enable) {
  if (fd_ < 0) return;
  uint8_t b[24] = {0};
  PutU32(&b[0], kCtrlMagic);
  PutU32(&b[4], ++seq_);
  PutI16(&b[8], ClampCdps(yaw_dps));
  PutI16(&b[10], ClampCdps(pitch_dps));
  b[12] = enable ? kFlagEnable : kFlagDisable;
  uint8_t chk = 0;
  for (int i = 0; i < 13; ++i) chk ^= b[i];
  b[13] = chk;
  // 底盘字段全 0（不控制底盘）
  b[20] = 0;
  uint8_t chk2 = 0;
  for (int i = 14; i < 21; ++i) chk2 ^= b[i];
  b[21] = chk2;

  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_addr.s_addr = addr_be_;
  to.sin_port = port_be_;
  ::sendto(fd_, b, sizeof(b), 0, reinterpret_cast<sockaddr*>(&to), sizeof(to));
}

}  // namespace rm27
