// ===========================================================================
//  AimView —— 27RM 自瞄数据（UDP 40002）接收与解析
//
//  27RM（~/stmproject/27RM_AUTO_AIM）按 50 Hz 把瞄准结果以 **48 字节小端报文**
//  广播出去（默认 192.168.2.255:40002，无线用 10.42.0.255:40002）。
//  本类只负责：绑端口、收包、校验 magic、统计频率/丢包、只读解析。
//
//  ⚠️ 报文结构体**不复制**：直接复用 27RM 工程里的 inc/aim_udp.hpp（唯一权威来源）。
//     那个头文件缺失时（比如只 clone 了 gimbal_ctl），本类会退化成"永远没数据"，
//     编译照常通过 —— 见 CTL_HAVE_AIM_LINK。
//
//  字段（48 字节，全小端，无 CRC）：
//    off size type   field
//     0   4   u32   magic = 0x524D3732 ("27RM")
//     4   4   u32   seq
//     8   4   f32   aim_yaw_deg    预测瞄准 yaw（°，向右为正，相机系）
//    12   4   f32   aim_pitch_deg  预测瞄准 pitch（°，向上为正）
//    16   4   f32   dist_m         目标板距离（m）
//    20   4   f32   spin_rate_dps  反小陀螺转速（°/s）
//    24  12   i32x3 center_mm      机器人中心（mm，相机系）
//    36   4   i32   radius_mm      环半径（mm）
//    40   1   u8    valid          1 = 有可打目标
//    41   1   u8    flags          bit0 = 反小陀螺跟踪中；bit1 = 选中板朝向相机
//    42   1   u8    plate_slot     选中的板序号 0..3
//    43   1   u8    reserved
//    44   4   u32   reserved2
// ===========================================================================
#pragma once

#include <cstdint>
#include <mutex>
#include <string>

#if defined(__has_include)
#if __has_include("aim_udp.hpp")
#define CTL_HAVE_AIM_LINK 1
#include "aim_udp.hpp"   // rm27::AimPacket / rm27::kAimMagic
#endif
#endif
#ifndef CTL_HAVE_AIM_LINK
#define CTL_HAVE_AIM_LINK 0
#endif

namespace ctl {

constexpr int kAimDefaultPort = 40002;

/// 解析后的一帧自瞄数据（控制端只读视角，字段名对齐 aim_udp.hpp）
struct AimState {
    uint32_t seq          = 0;
    double   yaw_deg      = 0.0;   ///< 相机系，向右为正
    double   pitch_deg    = 0.0;   ///< 相机系，向上为正
    double   dist_m       = 0.0;
    double   spin_dps     = 0.0;   ///< 反小陀螺转速（°/s，0 = 不转）
    int32_t  center_mm[3] = {0, 0, 0};
    int32_t  radius_mm    = 0;
    bool     valid        = false; ///< 1 = 有可打目标
    bool     spin_tracking = false;///< flags bit0
    bool     facing       = false; ///< flags bit1（选中板朝向相机）
    uint8_t  plate_slot   = 0;     ///< 0..3
    uint8_t  color_id     = 0;     ///< 27RM 当前敌方颜色：1=蓝 2=红（报文 reserved 字节）
    double   t_rx         = 0.0;   ///< 收到时刻（steady clock 秒）
};

class AimView {
public:
    AimView() = default;
    ~AimView() { close(); }
    AimView(const AimView&) = delete;
    AimView& operator=(const AimView&) = delete;

    /// 绑 0.0.0.0:<port>（广播包要求 SO_REUSEADDR）；失败返回 false
    bool open(int port);
    void close();
    bool ok() const { return fd_ >= 0; }

    /// 非阻塞把积压的全收完；返回是否收到过合法包
    bool poll();

    /// 取最新一帧（拷贝）
    bool latest(AimState& out) const;

    /// 最近一帧是否新鲜（秒）
    bool fresh(double now_s, double stale_s = 0.30) const;

    /// 最近一秒的收包频率
    double rateHz() const;

    /// 按 seq 跳号估算的丢包数
    uint32_t lost() const;

    /// 数据来自哪（ip:port），没收到过返回 "-"
    std::string peer() const;

    /// 没编译进自瞄支持时给个说明
    static bool compiledIn() { return CTL_HAVE_AIM_LINK != 0; }

private:
    int                      fd_ = -1;
    mutable std::mutex       mu_;
    AimState                 st_;
    bool                     have_       = false;
    uint32_t                 lost_       = 0;
    uint32_t                 last_seq_   = 0;
    bool                     have_seq_   = false;
    std::string              peer_       = "-";
    uint64_t                 count_      = 0;      ///< 累计包数
    double                   rate_       = 0.0;    ///< Hz
    double                   win_t_      = 0.0;    ///< 频率统计窗口起点
    uint64_t                 win_n_      = 0;      ///< 窗口内包数
};

}  // namespace ctl
