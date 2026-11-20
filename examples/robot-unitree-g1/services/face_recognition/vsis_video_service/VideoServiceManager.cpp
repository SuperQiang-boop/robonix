#include "VideoServiceManager.h"
#include "FaceRecognitionEngine.h"
#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"

using json = nlohmann::json;

namespace Vsis {

void VideoServiceManager::init() {
    m_auxer = std::make_unique<VideoServiceAuxer>();
    m_auxer->init();

    // M2: 相机模块 (未配视频源则跳过, 其余模块照常起)
    if (!m_auxer->m_videoSource.empty()) {
        m_camera = std::make_unique<VideoCameraReader>(m_auxer->m_videoSource,
                                                       m_auxer->m_videoReconnectMs,
                                                       m_auxer->m_videoWidth,
                                                       m_auxer->m_videoHeight);
    } else {
        SPDLOG_WARN("未配置 video.source, 跳过摄像头模块");
        exit(exitcode::FAIL_CAMERA);
    }
    m_face = std::make_unique<FaceRecognitionEngine>(*m_camera);
    // TODO(M5): m_person = std::make_unique<VideoPersonStore>();
    // TODO(M5): m_http   = std::make_unique<VideoHttpApi>();

    SPDLOG_INFO("系统初始化");
}

void VideoServiceManager::start() {
    SPDLOG_INFO("系统启动...");

#if 0
    // ---- 临时冒烟: 连取 100 帧验证 read() 通路
    if (m_camera) {
        cv::Mat f; int got = 0;
        auto t0 = std::chrono::steady_clock::now();
        while (got < 100) {
            // 预算耗尽即收手; 单次 read 超时只是暖机/抖动, 继续等下一帧
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > 10.0) break;
            if (m_camera->read(f, 500)) ++got;
        }
        const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (got == 100) SPDLOG_INFO("[冒烟] read() 通路 OK: 100帧/{:.1f}s(含暖机), 尺寸 {}x{}",
                                    sec, f.cols, f.rows);
        else SPDLOG_ERROR("[冒烟] 10s 内只取到 {}/100 帧, 检查视频源", got);
    }
#endif

    if (m_camera) m_camera->start();
    if (m_face) {
        if (!m_face->start()) {
            SPDLOG_ERROR("人脸引擎启动失败");
            if (m_camera) m_camera->stop();
            exit(exitcode::FAIL_ISF);
        }
    }
}

void VideoServiceManager::stop() {
    SPDLOG_INFO("系统终止...");
    if (m_face) m_face->stop();
    if (m_camera) m_camera->stop();
}

} // namespace Vsis