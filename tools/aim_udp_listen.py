#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自瞄 UDP 数据接收调试脚本（给控制端开发者用）

27RM 默认把自瞄数据以 50Hz 发到 UDP 40002（子网广播 192.168.2.255:40002）。
本脚本监听端口并解析 48 字节报文，方便控制端开发者验证链路。

用法：
  python3 aim_udp_listen.py              # 监听 0.0.0.0:40002
  python3 aim_udp_listen.py 40002        # 指定端口
  python3 aim_udp_listen.py 40002 1.0    # 打印间隔 1 秒

报文格式（48 字节，小端）见 27RM_AUTO_AIM/README.md「自瞄输出」一节。
Ctrl+C 退出。
"""

import socket
import struct
import sys
import time

MAGIC = 0x524D3732  # "27RM"
PKT_FMT = "<II ffff iiii BBBBI"  # 48 字节
PKT_LEN = 48


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 40002
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.5)
    print(f"[aim_udp] 监听 UDP {port}（27RM 的 --aim-udp 默认广播到这个端口）")
    print("按 Ctrl+C 退出\n")

    last = time.time()
    last_seq = None
    lost = 0
    got_addr = "-"

    while True:
        try:
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            if time.time() - last > 1.5:
                print("[aim_udp] 没有数据（27RM 没在跑？）")
                last = time.time()
            continue
        if len(data) < PKT_LEN:
            continue
        (magic, seq, yaw, pitch, dist, spin, cx, cy, cz, radius,
         valid, flags, slot, _r1, _r2) = struct.unpack(PKT_FMT, data[:PKT_LEN])
        if magic != MAGIC:
            continue
        got_addr = f"{addr[0]}:{addr[1]}"
        if last_seq is not None and seq > last_seq + 1:
            lost += seq - last_seq - 1
        last_seq = seq

        if time.time() - last >= interval:
            last = time.time()
            mode = "反小陀螺" if (flags & 0x01) else "单板"
            facing = "朝向相机" if (flags & 0x02) else "背对"
            print(
                f"seq={seq:6d} valid={valid} {mode} {facing} 板{slot} | "
                f"瞄准 yaw={yaw:+.2f}° pitch={pitch:+.2f}° | "
                f"距离 {dist:.3f}m | 转速 {spin:+.0f}°/s | "
                f"中心({cx},{cy},{cz})mm R={radius}mm | 丢包≈{lost} | {got_addr}"
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[aim_udp] 退出")
