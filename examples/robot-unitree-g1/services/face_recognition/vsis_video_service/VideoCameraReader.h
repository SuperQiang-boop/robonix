/**
 * @file VideoCameraReader.h
 * @brief 摄像头采集器 —— 独立线程持续拉流, 对外只暴露"最新帧"
 *
 * 核心语义 (M2 实测结论落地):
 *   1. 双后端: RTSP/HTTP/HTTPS 网络流走 FFmpeg, 其余按 V4L2 设备
 *   2. 最新帧槽位: 采集线程覆盖写, read() 永拿最新帧, 处理慢自动跳帧不掉队
 *   3. 断流自动重连; 帧尺寸永远读实际值, 不写死
 *
 * @author Teate
 * @date 2026-09-10
 * @version 1.0.0
 * @copyright Copyright (c) 2023, VSISLab. All rights reserved.
 */
#ifndef VSIS_VIDEO_CAMERA_READER_H
#define VSIS_VIDEO_CAMERA_READER_H

#include <cstdint>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <opencv2/opencv.hpp>

namespace Vsis {

class VideoCameraReader {
public:
    /**
     * @brief 构造即定参, 不做任何打开动作
     * @param source       视频源: RTSP/HTTP/HTTPS URL 或 V4L2 设备 ("0" / "/dev/video0")
     * @param reconnectMs  断流后重连间隔 (毫秒), <=0 时取 3000
     * @param requestWidth V4L2 请求宽, 0=相机默认 (网络流忽略)
     * @param requestHeight V4L2 请求高, 0=相机默认 (网络流忽略)
     */
    VideoCameraReader(const std::string& source, int reconnectMs,
                      int requestWidth = 0, int requestHeight = 0);
    ~VideoCameraReader();

    void start();   // 起采集线程 (重复调用无副作用)
    void stop();    // 停线程并释放采集 (重复调用无副作用)

    // 取一帧新画面: 以内部已消费序号为准, 慢消费者自动跳到最新帧
    // timeoutMs 内无新帧返回 false; 单消费者设计
    bool read(cv::Mat& out, int timeoutMs = 100);

    bool isOpened() const;      // 后端当前是否连着 (断线重连期间为 false)
    double actualFps() const;   // 采集线程实测帧率 (每秒刷新, 0=尚无数据)

    VideoCameraReader(const VideoCameraReader&) = delete;
    VideoCameraReader& operator=(const VideoCameraReader&) = delete;

private:
    void captureLoop();                     // 采集+重连主体 (采集线程)
    bool openBackend(cv::VideoCapture& cap);// 按 source 类型选后端打开
    void sleepInterruptible(int ms);        // 可被 stop() 打断的休眠

    // ---- 配置 (构造注入, 只读) ----
    std::string m_source;
    int m_reconnectMs{3000};
    int m_requestWidth{0};
    int m_requestHeight{0};

    // ---- 最新帧槽位 (m_mtx 保护) ----
    std::mutex m_mtx;
    std::condition_variable m_cv;
    cv::Mat m_latest;           // 最新一帧 (采集线程 clone 覆盖写)
    uint64_t m_frameSeq{0};     // 帧序号, 每入槽 +1
    uint64_t m_lastReadSeq{0};  // 消费者已取走的序号 (read 推进)

    // ---- 线程与状态 ----
    std::thread m_thread;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_connected{false};
    std::atomic<double> m_actualFps{0.0};
};

} // namespace Vsis

#endif // VSIS_VIDEO_CAMERA_READER_H
