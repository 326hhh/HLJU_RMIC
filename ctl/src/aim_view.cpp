#include "aim_view.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

#if CTL_HAVE_AIM_LINK
#include <cstring>   // std::memcpy（AimPacket 逐字节拷进来）
#endif

namespace ctl {

namespace {
double nowS() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
}  // namespace

bool AimView::open(int port) {
    close();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;

    int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    ::setsockopt(fd_, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(static_cast<uint16_t>(port));
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::fprintf(stderr, "[aim] bind 0.0.0.0:%d 失败: %s\n", port, std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    const int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
    win_t_ = nowS();
    return true;
}

void AimView::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool AimView::poll() {
    if (fd_ < 0) return false;
    bool got = false;
    for (;;) {
        uint8_t buf[256];
        sockaddr_in from{};
        socklen_t   fromlen = sizeof(from);
        const ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), 0,
                                     reinterpret_cast<sockaddr*>(&from), &fromlen);
        if (n <= 0) break;

#if CTL_HAVE_AIM_LINK
        if (static_cast<size_t>(n) < sizeof(rm27::AimPacket)) continue;
        rm27::AimPacket pkt;
        std::memcpy(&pkt, buf, sizeof(pkt));
        if (pkt.magic != rm27::kAimMagic) continue;   // 不是 27RM 的包

        const double t = nowS();
        {
            std::lock_guard<std::mutex> lk(mu_);
            st_.seq           = pkt.seq;
            st_.yaw_deg       = pkt.aim_yaw_deg;
            st_.pitch_deg     = pkt.aim_pitch_deg;
            st_.dist_m        = pkt.dist_m;
            st_.spin_dps      = pkt.spin_rate_dps;
            st_.center_mm[0]  = pkt.center_x_mm;
            st_.center_mm[1]  = pkt.center_y_mm;
            st_.center_mm[2]  = pkt.center_z_mm;
            st_.radius_mm     = pkt.radius_mm;
            st_.valid         = (pkt.valid != 0);
            st_.spin_tracking = (pkt.flags & 0x01) != 0;
            st_.facing        = (pkt.flags & 0x02) != 0;
            st_.plate_slot    = pkt.plate_slot;
            st_.color_id      = pkt.reserved;   ///< 1=蓝 2=红
            st_.t_rx          = t;

            if (have_seq_ && pkt.seq > last_seq_ + 1) lost_ += pkt.seq - last_seq_ - 1;
            last_seq_  = pkt.seq;
            have_seq_  = true;
            have_      = true;
            ++count_;
            ++win_n_;

            char ip[64] = {0};
            ::inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
            peer_ = std::string(ip) + ":" + std::to_string(ntohs(from.sin_port));

            const double dt = t - win_t_;
            if (dt >= 1.0) {
                rate_  = static_cast<double>(win_n_) / dt;
                win_n_ = 0;
                win_t_ = t;
            }
        }
        got = true;
#else
        (void)from;
        (void)fromlen;
        break;   // 没编译进自瞄支持：收也没有意义
#endif
    }
    return got;
}

bool AimView::latest(AimState& out) const {
    std::lock_guard<std::mutex> lk(mu_);
    if (!have_) return false;
    out = st_;
    return true;
}

bool AimView::fresh(double now_s, double stale_s) const {
    std::lock_guard<std::mutex> lk(mu_);
    return have_ && (now_s - st_.t_rx) <= stale_s;
}

double AimView::rateHz() const {
    std::lock_guard<std::mutex> lk(mu_);
    return rate_;
}

uint32_t AimView::lost() const {
    std::lock_guard<std::mutex> lk(mu_);
    return lost_;
}

std::string AimView::peer() const {
    std::lock_guard<std::mutex> lk(mu_);
    return peer_;
}

}  // namespace ctl
