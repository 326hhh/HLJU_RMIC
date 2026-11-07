cmake_minimum_required(VERSION 3.10)
project(rm27_auto_aim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# --- OpenCV ---
# Jetson 的 OpenCVConfig.cmake 会 find_package(CUDA) 并强制要 CUDA_TOOLKIT_ROOT_DIR，
# 直接编译视觉部分用不着 CUDA —— 加开关改用 pkg-config：
#   cmake -S . -B build -DRM27_USE_PKGCONFIG=ON
option(RM27_USE_PKGCONFIG "Use pkg-config opencv4 (skip find_package)" OFF)

if(NOT RM27_USE_PKGCONFIG)
  find_package(OpenCV QUIET COMPONENTS core imgproc highgui imgcodecs videoio calib3d)
endif()
if(OpenCV_FOUND)
  message(STATUS "OpenCV ${OpenCV_VERSION} via find_package")
  set(RM27_OPENCV_LIBS ${OpenCV_LIBS})
  set(RM27_OPENCV_INCS ${OpenCV_INCLUDE_DIRS})
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(OPENCV4 REQUIRED opencv4)
  message(STATUS "OpenCV via pkg-config: ${OPENCV4_VERSION}")
  set(RM27_OPENCV_LIBS ${OPENCV4_LIBRARIES})
  set(RM27_OPENCV_INCS ${OPENCV4_INCLUDE_DIRS} ${OPENCV4_INCLUDE_DIR})
endif()

add_compile_options(-Wall -Wextra)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc ${RM27_OPENCV_INCS})

# 公共源码
set(RM27_COMMON
  src/camera_usb.cpp
  src/camera_calib.cpp
  src/light_bar.cpp)

# 自瞄主程序（输出名 27RM）
add_executable(rm27 src/main.cpp ${RM27_COMMON})
set_target_properties(rm27 PROPERTIES OUTPUT_NAME 27RM)
target_link_libraries(rm27 ${RM27_OPENCV_LIBS} pthread)

# 标定图采集（USB 摄像头）
add_executable(capture_calib src/capture_calib.cpp src/camera_usb.cpp)
target_link_libraries(capture_calib ${RM27_OPENCV_LIBS} pthread)

# 标定计算（纯 OpenCV，不依赖相机）
add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate ${RM27_OPENCV_LIBS})

# 检测验证/调参
add_executable(detect_test src/detect_test.cpp ${RM27_COMMON})
target_link_libraries(detect_test ${RM27_OPENCV_LIBS} pthread)

# 几何链路验证
add_executable(pose_test src/pose_test.cpp src/camera_usb.cpp src/camera_calib.cpp)
target_link_libraries(pose_test ${RM27_OPENCV_LIBS} pthread)
