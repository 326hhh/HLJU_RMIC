#ifndef RM27_AIM_UDP_HPP_
#define RM27_AIM_UDP_HPP_

/**
 * @file  aim_udp.hpp
 * @brief 自瞄数据 UDP 输出 —— 给控制端（gimbal_ctl / 自制控制程序）对接用
 *
 * 27RM 把当前瞄准结果打包成 48 字节小端报文，按 50 Hz 发到指定地址
 * （默认子网广播 192.168.2.255:40002）。控制端解析后即可驱动云台。
 *
 * 报文格式（48 字节，全部小端）：
 *   off size type   字段
 *    0   4   u32   magic = 0x524D3732 ("27RM")
 *    4   4   u32   seq            递增序号
 *    8   4   f32   aim_yaw_deg    预测瞄准 yaw（°），向右为正（相机系）
 *   12   4   f32   aim_pitch_deg  预测瞄准 pitch（°），向上为正
 *   16   4   f32   dist_m         目标板距离（米）
 *   20   4   f32   spin_rate_dps  反小陀螺转速（°/s，0=不转）
 *   24   4   i32   center_x_mm    机器人中心 x（mm，相机系，跟踪无效时为 0）
 *   28   4   i32   center_y_mm
 *   32   4   i32   center_z_mm
 *   36   4   i32   radius_mm      机器人环半径（mm，跟踪无效时为 0）
 *   40   1   u8    valid          1=有可打目标（位姿有效）
 *   41   1   u8    flags          bit0=反小陀螺跟踪中；bit1=选中板朝向相机
 *   42   1   u8    plate_slot     选中的板序号（0..3）
 *   43   1   u8    reserved       = 0
 *   44   4   u32   reserved2      = 0
 */

#include <cstdint>
#include <string>
#include <vector>

namespace rm27 {

#pragma pack(push, 1)
struct AimPacket {
  uint32_t magic;        // 0x524D3732
  uint32_t seq;
  float aim_yaw_deg;
  float aim_pitch_deg;
  float dist_m;
  float spin_rate_dps;
  int32_t center_x_mm;
  int32_t center_y_mm;
  int32_t center_z_mm;
  int32_t radius_mm;
  uint8_t valid;
  uint8_t flags;
  uint8_t plate_slot;
  uint8_t reserved;
  uint32_t reserved2;
};
#pragma pack(pop)

static_assert(sizeof(AimPacket) == 48, "AimPacket 必须是 48 字节");

constexpr uint32_t kAimMagic = 0x524D3732u;  // "27RM"（小端字节序）

class AimUdpSender {
 public:
  AimUdpSender() = default;
  ~AimUdpSender();

  AimUdpSender(const AimUdpSender&) = delete;
  AimUdpSender& operator=(const AimUdpSender&) = delete;

  /**
   * 启动发送端。**支持逗号分隔的多个目标**，一条报文同时发到所有目标：
   *   有线网段广播 + 热点网段广播（两套线都要能用时的正确姿势）：
   *     --aim-udp "192.168.2.255:40002,10.42.0.255:40002"
   *
   * @param target "host:port[,host:port...]"（host 可为 IP 或 x.x.x.255 广播）；
   *               空串 / "off" / "0" 表示关闭
   * @return 至少解析出一个合法目标时返回 true
   */
  bool Start(const std::string& target);
  void Stop();
  bool enabled() const { return fd_ >= 0; }

  void Send(AimPacket* pkt);  // 自动填 magic/seq 并发送

 private:
  struct Target {
    uint32_t ip_be   = 0;  // 网络字节序
    uint16_t port_be = 0;  // 网络字节序
  };

  int                 fd_ = -1;
  std::vector<Target> targets_;   ///< 逗号分隔支持多目标（有线 + 热点）
};

}  // namespace rm27

#endif  // RM27_AIM_UDP_HPP_
