/**
 * @file    calibrate.cpp
 * @brief   相机标定计算工具（27RM 项目）
 *
 * 读取 calib_images/ 里的棋盘格照片，用 calibrateCamera 计算内参与畸变，
 * 结果保存为 configs/camera.yaml。
 *
 * 运行：
 *   ./build/linux/x86_64/debug/calibrate                       # 默认 10x7 内角点 15mm
 *   ./build/linux/x86_64/debug/calibrate --cols 11 --rows 8    # 其它规格的板
 *   ./build/linux/x86_64/debug/calibrate --show                # 逐张显示角点与重投影
 *
 * 输出：
 *   - 重投影误差 RMS（<0.3 很好，0.3~0.5 可用，>0.5 建议删掉差图重拍）
 *   - 内参矩阵 K、畸变系数
 *   - 每张图的误差排行（最差的几张优先考虑删除）
 *   - camera_calib.yml（后续去畸变 / PnP 用）
 */

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include <string>
#include <vector>

namespace {

struct Options {
    int board_cols = 10;       // 内角点列数（10x7 对应 11x8 方格板）
    int board_rows = 7;
    float square_mm = 15.0f;   // 方格边长 mm（打印后用卡尺实测！）
    std::string dir = "calib_images";
    std::string out = "configs/camera.yaml";
    bool show = false;
};

void printUsage(const char *prog) {
    std::printf(
        "用法: %s [选项]\n"
        "  --cols N        内角点列数（默认 8）\n"
        "  --rows N        内角点行数（默认 6）\n"
        "  --square F      方格边长 mm（默认 20.0，本工程实测值）\n"
        "  --dir PATH      标定图目录（默认 calib_images）\n"
        "  --out FILE      输出文件（默认 configs/camera.yaml）\n"
        "  --show          逐张显示角点与重投影（调试用）\n",
        prog);
}

bool parseArgs(int argc, char **argv, Options &opt) {
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--cols") && i + 1 < argc) {
            opt.board_cols = std::atoi(argv[++i]);
        } else if (!std::strcmp(argv[i], "--rows") && i + 1 < argc) {
            opt.board_rows = std::atoi(argv[++i]);
        } else if (!std::strcmp(argv[i], "--square") && i + 1 < argc) {
            opt.square_mm = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--dir") && i + 1 < argc) {
            opt.dir = argv[++i];
        } else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) {
            opt.out = argv[++i];
        } else if (!std::strcmp(argv[i], "--show")) {
            opt.show = true;
        } else {
            printUsage(argv[0]);
            return false;
        }
    }
    return true;
}

struct ImageInfo {
    std::string name;
    double error_px;    // 该图平均重投影误差
    int index;
};

} // namespace

int main(int argc, char **argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) {
        return EXIT_FAILURE;
    }

    // ---------------- 读图 ----------------
    std::vector<cv::String> files;
    cv::glob(opt.dir + "/*.png", files);
    {
      std::vector<cv::String> jpgs;
      cv::glob(opt.dir + "/*.jpg", jpgs);
      files.insert(files.end(), jpgs.begin(), jpgs.end());
    }
    if (files.empty()) {
        std::fprintf(stderr, "错误: %s/ 里没有图片（支持 png/jpg）\n",
                     opt.dir.c_str());
        // 子目录里若有图片，给出可直接复制的命令（例如 old_run/）
        DIR *dp = opendir(opt.dir.c_str());
        if (dp != nullptr) {
            std::vector<std::string> hints;
            struct dirent *entry;
            while ((entry = readdir(dp)) != nullptr) {
                if (entry->d_name[0] == '.') {
                    continue;
                }
                std::string sub = opt.dir + "/" + entry->d_name;
                struct stat st;
                if (stat(sub.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
                    continue;
                }
                std::vector<cv::String> sub_files;
                cv::glob(sub + "/*.png", sub_files);
                {
                  std::vector<cv::String> jpgs;
                  cv::glob(sub + "/*.jpg", jpgs);
                  sub_files.insert(sub_files.end(), jpgs.begin(), jpgs.end());
                }
                if (!sub_files.empty()) {
                    hints.push_back(sub + "  (" +
                                    std::to_string(sub_files.size()) + " 张)");
                }
            }
            closedir(dp);
            if (!hints.empty()) {
                std::fprintf(stderr, "提示: 子目录里有图片，可以这样用：\n");
                for (const auto &h : hints) {
                    std::fprintf(stderr, "   ./calibrate --dir %.*s --square %.1f\n",
                                 static_cast<int>(h.find("  (")), h.c_str(),
                                 opt.square_mm);
                }
            }
        }
        std::fprintf(stderr,
                     "或者运行 capture_calib 采集新图（目标 20~30 张）\n");
        return EXIT_FAILURE;
    }
    std::printf("找到 %zu 张图片\n", files.size());

    // ---------------- 逐张检测角点 ----------------
    // 候选尺寸：配置值 ±1（兼容"方格数"与"内角点数"的混淆）
    std::vector<cv::Size> candidates{
        cv::Size(opt.board_cols, opt.board_rows),
        cv::Size(std::max(3, opt.board_cols - 1), std::max(3, opt.board_rows - 1)),
        cv::Size(opt.board_cols + 1, opt.board_rows + 1),
    };

    cv::Size used_size(0, 0);
    cv::Size image_size(0, 0);
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::string> used_names;
    std::vector<std::string> failed_names;

    for (const auto &file : files) {
        cv::Mat img = cv::imread(file);
        if (img.empty()) {
            failed_names.emplace_back(file);
            continue;
        }
        if (image_size.width == 0) {
            image_size = img.size();
        }
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        bool found = false;
        cv::Size detected(0, 0);
        std::vector<cv::Point2f> corners;
        for (const auto &sz : candidates) {
            corners.clear();
            // 经典检测器为主（Jetson OpenCV4.8 的 SB 检测器又慢又会抛
            // flann getNeighbors 断言），SB 只做兜底且吞异常
            found = cv::findChessboardCorners(
                gray, sz, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
            if (!found) {
                corners.clear();
                try {
                    found = cv::findChessboardCornersSB(
                        gray, sz, corners,
                        cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_LARGER);
                } catch (const cv::Exception&) {
                    found = false;
                }
            }
            if (found) {
                detected = sz;
                break;
            }
        }
        if (!found) {
            failed_names.emplace_back(file);
            continue;
        }

        // 第一张成功图决定实际尺寸，后续图尺寸不一致则丢弃
        if (used_size.width == 0) {
            used_size = detected;
            std::printf("标定板实际尺寸: %d x %d 内角点\n", used_size.width,
                        used_size.height);
            if (used_size != candidates.front()) {
                std::printf("  注意: 与配置 %d x %d 不一致（你的板子可能是 %d x "
                            "%d 个方格）\n",
                            opt.board_cols, opt.board_rows,
                            used_size.width + 1, used_size.height + 1);
            }
        }
        if (detected != used_size) {
            failed_names.emplace_back(file + " (尺寸不一致)");
            continue;
        }

        cv::cornerSubPix(
            gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                             30, 0.001));

        // 世界坐标：z=0 平面，方格为实测边长
        std::vector<cv::Point3f> objp;
        for (int r = 0; r < used_size.height; ++r) {
            for (int c = 0; c < used_size.width; ++c) {
                objp.emplace_back(c * opt.square_mm, r * opt.square_mm, 0.f);
            }
        }
        object_points.push_back(objp);
        image_points.push_back(corners);
        used_names.push_back(file);
    }

    std::printf("可用图片: %zu / %zu\n", image_points.size(), files.size());

    // ---------------- 覆盖度 / 多样性报告（防止参数退化） ----------------
    if (image_points.size() >= 4 && !image_size.empty()) {
        float gx0 = 1e9f, gx1 = -1e9f, gy0 = 1e9f, gy1 = -1e9f;
        float min_sz = 1e9f, max_sz = 0.f;
        for (const auto &c : image_points) {
            if (c.empty()) continue;
            float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f;
            for (const auto &pt : c) {
                x0 = std::min(x0, pt.x); x1 = std::max(x1, pt.x);
                y0 = std::min(y0, pt.y); y1 = std::max(y1, pt.y);
            }
            gx0 = std::min(gx0, x0); gx1 = std::max(gx1, x1);
            gy0 = std::min(gy0, y0); gy1 = std::max(gy1, y1);
            min_sz = std::min(min_sz, std::max(x1 - x0, y1 - y0));
            max_sz = std::max(max_sz, std::max(x1 - x0, y1 - y0));
        }
        const float cov_x = (gx1 - gx0) / image_size.width;
        const float cov_y = (gy1 - gy0) / image_size.height;
        const float size_ratio = max_sz / std::max(1.f, min_sz);
        std::printf("\n[覆盖度] 角点横向覆盖 %.0f%% / 纵向覆盖 %.0f%% / "
                    "棋盘格大小比例 %.2fx\n", cov_x * 100.f, cov_y * 100.f,
                    size_ratio);
        bool poor = false;
        if (cov_x < 0.80f || cov_y < 0.80f) {
            std::printf("⚠️ 覆盖不足：棋盘格没摆到画面边缘，主点/畸变会解不准！\n");
            poor = true;
        }
        if (size_ratio < 1.5f) {
            std::printf("⚠️ 距离多样性不足：都在相近距离拍，fx 会解不准！\n");
            poor = true;
        }
        if (poor) {
            std::printf("   建议：板子在画面①中心②四角③边缘都摆一摆，"
                        "距离从 ~20cm 到 ~80cm，各种倾斜都要有，重拍 25~30 张。\n");
        }
    }
    if (!failed_names.empty()) {
        std::printf("未使用的图片:\n");
        for (const auto &n : failed_names) {
            std::printf("   %s\n", n.c_str());
        }
    }
    if (image_points.size() < 8) {
        std::fprintf(stderr,
                     "错误: 可用图片不足 8 张\n"
                     "  · 检查拍摄条件（整板入画/无反光/不模糊）\n"
                     "  · 或确认 --cols/--rows 参数\n");
        return EXIT_FAILURE;
    }

    // ---------------- 标定（自动剔除坏图） ----------------
    // OpenCV 的 calibrateCamera 对个别角点顺序错乱/严重模糊的图很敏感，
    // 一张 40+px 的坏图就能把整个解带偏（fx 发散到几千）。
    // 策略：迭代去掉「单图重投影误差最大」的图，直到 RMS <= 0.45 或不足 15 张。
    {
        std::vector<std::string> dropped;
        while (image_points.size() > 15) {
            cv::Mat Kt, Dt;
            std::vector<cv::Mat> rv, tv;
            const double rms_t = cv::calibrateCamera(
                object_points, image_points, image_size, Kt, Dt, rv, tv);
            if (rms_t <= 0.45) break;
            int worst = 0;
            double worst_e = -1.0;
            for (std::size_t i = 0; i < image_points.size(); ++i) {
                std::vector<cv::Point2f> proj;
                cv::projectPoints(object_points[i], rv[i], tv[i], Kt, Dt, proj);
                double e = 0.0;
                for (std::size_t k = 0; k < proj.size(); ++k) {
                    e += cv::norm(proj[k] - image_points[i][k]);
                }
                e /= static_cast<double>(proj.size());
                if (e > worst_e) { worst_e = e; worst = static_cast<int>(i); }
            }
            dropped.push_back(used_names[worst] + " (" +
                              std::to_string(static_cast<int>(worst_e + 0.5)) +
                              "px)");
            object_points.erase(object_points.begin() + worst);
            image_points.erase(image_points.begin() + worst);
            used_names.erase(used_names.begin() + worst);
        }
        if (!dropped.empty()) {
            std::printf("\n[自动剔除] 去掉 %zu 张坏图后重新标定:\n",
                        dropped.size());
            for (const auto &d : dropped) std::printf("   %s\n", d.c_str());
        }
    }

    cv::Mat K, D;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(object_points, image_points, image_size, K,
                                     D, rvecs, tvecs);
    std::printf("图像尺寸: %d x %d\n", image_size.width, image_size.height);

    std::printf("\n==============================\n");
    std::printf("重投影误差 RMS: %.4f 像素\n", rms);
    if (rms < 0.3)
        std::printf("评价: 很好 ✓\n");
    else if (rms < 0.5)
        std::printf("评价: 可接受\n");
    else
        std::printf("评价: 偏差较大，建议按下面排行删掉最差的几张后重跑\n");

    std::printf("\n内参矩阵 K:\n");
    for (int r = 0; r < 3; ++r) {
        std::printf("  [ ");
        for (int c = 0; c < 3; ++c)
            std::printf("%10.3f ", K.at<double>(r, c));
        std::printf("]\n");
    }
    std::printf("  fx=%.2f  fy=%.2f  cx=%.2f  cy=%.2f\n", K.at<double>(0, 0),
                K.at<double>(1, 1), K.at<double>(0, 2), K.at<double>(1, 2));

    std::printf("\n畸变系数 [k1 k2 p1 p2 k3 ...]:\n  ");
    for (int c = 0; c < D.cols; ++c) {
        std::printf("%.6f ", D.at<double>(c));
    }
    std::printf("\n");

    // ---------------- 每张图的误差排行 ----------------
    std::vector<ImageInfo> infos;
    for (size_t i = 0; i < image_points.size(); ++i) {
        std::vector<cv::Point2f> reproj;
        cv::projectPoints(object_points[i], rvecs[i], tvecs[i], K, D, reproj);
        double err = cv::norm(image_points[i], reproj, cv::NORM_L2) /
                     std::sqrt(static_cast<double>(reproj.size()));
        infos.push_back({used_names[i], err, static_cast<int>(i)});
    }
    std::sort(infos.begin(), infos.end(),
              [](const ImageInfo &a, const ImageInfo &b) {
                  return a.error_px > b.error_px;
              });
    std::printf("\n误差最大的 5 张（可考虑删掉后重跑）:\n");
    for (int i = 0; i < 5 && i < static_cast<int>(infos.size()); ++i) {
        std::printf("   %s  ->  %.3f px\n", infos[i].name.c_str(),
                    infos[i].error_px);
    }

    // ---------------- 保存 ----------------
    cv::FileStorage fs(opt.out, cv::FileStorage::WRITE);
    fs << "camera_matrix" << K;
    fs << "dist_coeffs" << D;
    fs << "image_width" << image_size.width;
    fs << "image_height" << image_size.height;
    fs << "rms" << rms;
    fs << "board_cols" << used_size.width;
    fs << "board_rows" << used_size.height;
    fs << "square_mm" << opt.square_mm;
    fs.release();
    std::printf("\n已保存 -> %s\n", opt.out.c_str());

    // ---------------- 可视化（可选） ----------------
    if (opt.show) {
        for (const auto &info : infos) {
            cv::Mat img = cv::imread(info.name);
            if (img.empty())
                continue;
            std::vector<cv::Point2f> reproj;
            cv::projectPoints(object_points[info.index], rvecs[info.index],
                              tvecs[info.index], K, D, reproj);
            for (const auto &p : image_points[info.index])
                cv::circle(img, p, 3, cv::Scalar(0, 255, 0), -1);
            for (const auto &p : reproj)
                cv::circle(img, p, 5, cv::Scalar(0, 0, 255), 1);
            cv::putText(img, info.name + " err=" + std::to_string(info.error_px),
                        cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 0), 2);
            cv::imshow("calibrate", img);
            if (cv::waitKey(0) == 27)
                break;
        }
    }

    std::printf("\n完成。后续使用：\n");
    std::printf("  cv::FileStorage fs(\"%s\", cv::FileStorage::READ);\n",
                opt.out.c_str());
    std::printf("  fs[\"camera_matrix\"] >> K; fs[\"dist_coeffs\"] >> D;\n");
    return EXIT_SUCCESS;
}
