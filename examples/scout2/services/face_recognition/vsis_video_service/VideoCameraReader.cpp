/**
 * @file VideoCameraReader.cpp
 * @brief 摄像头采集器实现
 *
 * @author Teate
 * @date 2026-09-10
 * @version 1.0.0
 * @copyright Copyright (c) 2023, VSISLab. All rights reserved.
 */
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "VideoCameraReader.h"
#include "spdlog/spdlog.h"

namespace Vsis {

using namespace std::chrono_literals;

VideoCameraReader::VideoCameraReader(const std::string& source, int reconnectMs,
                                     int requestWidth, int requestHeight)
    : m_source(source),
      m_reconnectMs(reconnectMs > 0 ? reconnectMs : 3000),
      m_requestWidth(requestWidth),
      m_requestHeight(requestHeight) {
}

VideoCameraReader::~VideoCameraReader() {
    stop();  // 兜底: 不许带着活线程析构
}

void VideoCameraReader::start() {
    if (m_running) {
        return;  // 重复 start 无副作用
    }
    {   // 清掉上一轮残留 (stop 后再 start 的场景)
        std::lock_guard<std::mutex> lock(m_mtx);
        m_latest.release();
        m_frameSeq = 0;
        m_lastReadSeq = 0;
    }
    m_actualFps = 0.0;
    m_connected = false;
    m_running = true;
    m_thread = std::thread(&VideoCameraReader::captureLoop, this);
    SPDLOG_INFO("[Camera] 采集线程启动: {}", m_source);
}

void VideoCameraReader::stop() {
    if (!m_running) {
        return;  // 重复 stop 无副作用
    }
    m_running = false;
    m_cv.notify_all();  // 叫醒重连休眠和阻塞中的 read
    if (m_thread.joinable()) {
        m_thread.join();
    }
    {
        std::lock_guard<std::mutex> lock(m_mtx);
        m_latest.release();
    }
    m_connected = false;
    m_actualFps = 0.0;
    SPDLOG_INFO("[Camera] 采集线程停止");
}

bool VideoCameraReader::read(cv::Mat& out, int timeoutMs) {
    std::unique_lock<std::mutex> lock(m_mtx);
    const uint64_t target = m_lastReadSeq + 1;
    // 等新帧入槽: 消费慢→自动跳到最新帧, 消费快→在此等下一帧, 不空转
    const bool got = m_cv.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                                   [this, target] { return m_frameSeq >= target || !m_running.load(); });
    if (!got || m_frameSeq < target) {
        return false;  // 超时无新帧 / 服务已停止
    }
    m_lastReadSeq = m_frameSeq;
    out = m_latest.clone();  // 出槽必须拷贝: m_latest 归采集线程所有
    return true;
}

bool VideoCameraReader::isOpened() const {
    return m_connected.load();
}

double VideoCameraReader::actualFps() const {
    return m_actualFps.load();
}

bool VideoCameraReader::openBackend(cv::VideoCapture& cap) {
    if (m_source.rfind("rtsp://", 0) == 0) {
        // M2 验证结论: TCP 必须在 open 之前经环境变量声明, 晚了静默走 UDP, 网络一抖就花屏
        setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp", 1);
        return cap.open(m_source, cv::CAP_FFMPEG);
    }

    // V4L2: 设备序号 ("0") 或设备路径 ("/dev/video0")
    int devIdx = -1;
    if (m_source.rfind("/dev/video", 0) == 0) {
        devIdx = std::atoi(m_source.c_str() + std::strlen("/dev/video"));
    } else if (!m_source.empty() &&
               std::all_of(m_source.begin(), m_source.end(),
                           [](unsigned char c) { return std::isdigit(c) != 0; })) {
        devIdx = std::atoi(m_source.c_str());
    }

    // Logi 类相机 YUYV@720p 只有 ~10fps, 请求 MJPG 才拿得满 30fps (对齐狗上 30fps)
    std::vector<int> params;
    if (m_requestWidth > 0 && m_requestHeight > 0) {
        params = {cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                  cv::CAP_PROP_FRAME_WIDTH, m_requestWidth,
                  cv::CAP_PROP_FRAME_HEIGHT, m_requestHeight};
    }
    const bool ok = (devIdx >= 0) ? cap.open(devIdx, cv::CAP_V4L2, params)
                                  : cap.open(m_source, cv::CAP_V4L2, params);
    if (!ok && !params.empty()) {
        // 相机不支持请求参数时, 退回默认参数再试一次
        SPDLOG_WARN("[Camera] 带参数打开失败, 改用相机默认参数: {}", m_source);
        return (devIdx >= 0) ? cap.open(devIdx, cv::CAP_V4L2)
                             : cap.open(m_source, cv::CAP_V4L2);
    }
    return ok;
}

void VideoCameraReader::sleepInterruptible(int ms) {
    std::unique_lock<std::mutex> lock(m_mtx);
    m_cv.wait_for(lock, std::chrono::milliseconds(ms),
                  [this] { return !m_running.load(); });
}

void VideoCameraReader::captureLoop() {
    while (m_running) {
        cv::VideoCapture cap;
        if (!openBackend(cap)) {
            SPDLOG_WARN("[Camera] 打开失败, {}ms 后重试: {}", m_reconnectMs, m_source);
            sleepInterruptible(m_reconnectMs);
            continue;
        }
        m_connected = true;
        SPDLOG_INFO("[Camera] 已连接: {} ({}x{} @{}fps)", m_source,
                    static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
                    static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)),
                    cap.get(cv::CAP_PROP_FPS));

        // 实测帧率: 每秒滚动统计 (CAP_PROP_FPS 在 FFmpeg/RTSP 下不可信, 只信自己量的)
        int fpsCount = 0;
        auto fpsT0 = std::chrono::steady_clock::now();
        cv::Mat frame;
        while (m_running) {
            if (!cap.read(frame) || frame.empty()) {
                break;  // 断流/出错 → 走重连
            }
            {   // 入槽: cap.read 复用内部缓冲, 必须 clone, 否则下一帧冲掉槽位内容
                std::lock_guard<std::mutex> lock(m_mtx);
                m_latest = frame.clone();
                ++m_frameSeq;
            }
            m_cv.notify_one();

            ++fpsCount;
            const auto now = std::chrono::steady_clock::now();
            if (now - fpsT0 >= 30s) {
                m_actualFps = fpsCount / std::chrono::duration<double>(now - fpsT0).count();
                fpsCount = 0;
                fpsT0 = now;
                SPDLOG_DEBUG("[Camera] 采集帧率 {:.1f} fps", m_actualFps.load());
            }
        }

        m_connected = false;
        m_actualFps = 0.0;
        cap.release();
        if (m_running) {
            SPDLOG_WARN("[Camera] 流中断, {}ms 后重连", m_reconnectMs);
            sleepInterruptible(m_reconnectMs);
        }
    }
}

} // namespace Vsis
