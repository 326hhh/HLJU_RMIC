#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
27RM 标定棋盘格生成器：直接输出 A4 精确毫米尺寸的 PDF（无需第三方库）。

规格（默认）：
  内角点 10 x 7（= 11 x 8 个方格），方格边长 15.0 mm，A4 纵向。
  图案整体 165 x 120 mm，居中摆放在页面中央。

用法：
  python3 tools/make_chessboard.py                       # 输出 chessboard_8x6_20mm_A4.pdf
  python3 tools/make_chessboard.py --cols 8 --rows 6 --square 20 --out xx.pdf

⚠️ 打印时必须选「100% / 实际大小」，不要「适应页面」。
   打印完用卡尺量一个方格的实际边长，按实测值传给 ./calibrate --square。
"""

import argparse

MM = 72.0 / 25.4  # 1 mm -> pt


def build_pdf(cols_inner, rows_inner, square_mm, page_w_mm=210.0, page_h_mm=297.0):
    n_cols = cols_inner + 1
    n_rows = rows_inner + 1
    board_w = n_cols * square_mm
    board_h = n_rows * square_mm

    page_w = page_w_mm * MM
    page_h = page_h_mm * MM
    x0 = (page_w_mm - board_w) / 2.0 * MM
    y0 = (page_h_mm - board_h) / 2.0 * MM

    ops = []
    # 棋盘格：顶左方格为白，黑格填充
    for r in range(n_rows):
        for c in range(n_cols):
            if (r + c) % 2 == 1:
                x = x0 + c * square_mm * MM
                y = y0 + (n_rows - 1 - r) * square_mm * MM
                ops.append(
                    "0 0 0 rg %.3f %.3f %.3f %.3f re f"
                    % (x, y, square_mm * MM, square_mm * MM)
                )

    # 外框细线
    ops.append("0 0 0 RG 0.75 w %.3f %.3f %.3f %.3f re S" % (x0, y0, board_w * MM, board_h * MM))

    # 100mm 比例尺（用来快速目测打印是否缩放）
    sx = x0
    sy = y0 - 14.0 * MM
    ops.append("0 0 0 RG 0.5 w")
    ops.append("%.3f %.3f m %.3f %.3f l S" % (sx, sy, sx + 100.0 * MM, sy))
    for i in range(0, 101, 10):
        ops.append(
            "%.3f %.3f m %.3f %.3f l S"
            % (sx + i * MM, sy - (3.0 if i % 50 == 0 else 1.5) * MM, sx + i * MM, sy)
        )

    # 文本
    ops.append("0 0 0 rg BT /F1 11 Tf")
    ops.append(
        "%.3f %.3f Td (27RM calib board: %dx%d inner corners, square %.1f mm)"
        % (x0, y0 + board_h * MM + 7.0 * MM, cols_inner, rows_inner, square_mm)
    )
    ops.append("ET")
    ops.append(
        "0 0 0 rg BT /F1 10 Tf %.3f %.3f Td (Print at 100%% scale - verify a square with caliper)"
        " ET" % (x0, sy - 10.0 * MM)
    )

    content = "\n".join(ops).encode("utf-8")

    objs = {}
    objs[1] = b"<< /Type /Catalog /Pages 2 0 R >>"
    objs[2] = b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>"
    objs[3] = (
        "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.3f %.3f] "
        "/Resources << /Font << /F1 4 0 R >> /ProcSet [/PDF /Text] >> "
        "/Contents 5 0 R >>" % (page_w, page_h)
    ).encode()
    objs[4] = b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>"
    objs[5] = (
        b"<< /Length %d >>\nstream\n" % len(content) + content + b"\nendstream"
    )

    out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
    offsets = {}
    for num in sorted(objs):
        offsets[num] = len(out)
        out += ("%d 0 obj\n" % num).encode() + objs[num] + b"\nendobj\n"
    xref_pos = len(out)
    out += ("xref\n0 %d\n" % (len(objs) + 1)).encode()
    out += b"0000000000 65535 f \n"
    for num in sorted(objs):
        out += ("%010d 00000 n \n" % offsets[num]).encode()
    out += (
        "trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n"
        % (len(objs) + 1, xref_pos)
    ).encode()
    return bytes(out)


def main():
    ap = argparse.ArgumentParser(description="Generate a printable chessboard PDF")
    ap.add_argument("--cols", type=int, default=10, help="inner corner columns (default 10)")
    ap.add_argument("--rows", type=int, default=7, help="inner corner rows (default 7)")
    ap.add_argument("--square", type=float, default=15.0, help="square size in mm (default 15.0)")
    ap.add_argument("--out", default=None, help="output pdf path")
    args = ap.parse_args()

    out = args.out or "chessboard_%dx%d_%gmm_A4.pdf" % (args.cols, args.rows, args.square)
    data = build_pdf(args.cols, args.rows, args.square)
    with open(out, "wb") as f:
        f.write(data)
    print("已生成 %s（%d x %d 内角点, %.1fmm/格, A4）" % (out, args.cols, args.rows, args.square))
    print("打印时选 100% 实际大小；打印后用卡尺量方格边长，填进 calibrate --square")


if __name__ == "__main__":
    main()
