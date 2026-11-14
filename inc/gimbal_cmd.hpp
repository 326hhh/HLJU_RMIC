#ifndef RM27_GIMBAL_CMD_HPP_
#define RM27_GIMBAL_CMD_HPP_

/**
 * @file  gimbal_cmd.hpp
 * @brief 云台指令 UDP 发送 —— 27RM 闭环自瞄用
 *
 * 27RM 检测到装甲板 → 算出瞄准偏差（yaw/pitch 角误差）→ 换算成角速度 →
 * 按 gimbal 守护进程的线协议（~/gimbal/include/gimbal/link/control_link.hpp）
 * 发 UDP 指令，驱动云台转过去跟踪目标。
 *
 * 上行报文（24 字节，小端，与 control_link.hpp 逐字节一致）：
 *   off size type  field
 *    0   4   u32  magic        = 0x4D454341
 *    4   4   u32  seq
 *    8   2   i16  yaw_cdps     yaw 角速度，0.01°/s
 *   10   2   i16  pitch_cdps   pitch 角速度，0.01°/s
 *   12   1   u8   flags        bit0=Enable, bit6=Disable
 *   13   1   u8   chk          bytes 0..12 XOR
 *   14   2   i16  vx_mmps      = 0
 *   16   2   i16  vy_mmps      = 0
 *   18   2   i16  wz_mradps    = 0
 *   20   1   u8   chassis_flags = 0
 *   21   1   u8   chk2         bytes 14..20 XOR
 *   22   2   u16  pad          = 0
 */

#include <cstdint>
#include <string>

namespace rm27 {

class GimbalCommandSender {
 public:
  GimbalCommandSender() = default;
  ~GimbalCommandSender();

  GimbalCommandSender(const GimbalCommandSender&) = delete;
  GimbalCommandSender& operator=(const GimbalCommandSender&) = delete;

  /** 打开到云台守护进程的 UDP 通道（默认 127.0.0.1:40001） */
  bool Start(const std::string& host, int port);
  void Stop();
  bool enabled() const { return fd_ >= 0; }

  /**
   * 发送一帧云台角速度指令。
   * @param yaw_dps    yaw 角速度（°/s，向右为正）
   * @param pitch_dps  pitch 角速度（°/s，抬头为正）
   * @param enable     true=使能云台；false=失能（Disable）
   */
  void Send(double yaw_dps, double pitch_dps, bool enable);

 private:
  int fd_ = -1;
  uint32_t addr_be_ = 0;
  uint16_t port_be_ = 0;
  uint32_t seq_ = 0;
};

}  // namespace rm27

#endif  // RM27_GIMBAL_CMD_HPP_
