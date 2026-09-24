/**
 * @file VideoServiceManager.h
 * @brief大管家
 * 
 * 该文件定义了 VideoServiceManager 类
 * 
 * @author Teate
 * @date 2026-07-01
 * @version 1.0.0
 * @copyright Copyright (c) 2023, VSISLab. All rights reserved.
 */

#ifndef VSIS_VIDEO_SERVICE_MANAGER_H
#define VSIS_VIDEO_SERVICE_MANAGER_H

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>

#include "VideoDef.h"
#include "VideoServiceAuxer.h"
#include "VideoCameraReader.h"
#include "FaceRecognitionEngine.h"

namespace Vsis {

class VideoServiceManager {

public:
    static VideoServiceManager Instance; 

    void init();

    void start();
    
    void stop();

    inline VideoServiceAuxer* getAuxer() const {return m_auxer.get();}

    // 删除拷贝构造、移动和赋值操作（单例模式的常规操作）
    VideoServiceManager(const VideoServiceManager&) = delete;
    VideoServiceManager& operator=(const VideoServiceManager&) = delete;
    VideoServiceManager(VideoServiceManager&&) = delete;
    VideoServiceManager& operator=(VideoServiceManager&&) = delete;

private:
    VideoServiceManager() = default;
    ~VideoServiceManager() = default;

    // 小助手
    std::unique_ptr<VideoServiceAuxer> m_auxer;

    // ---- 模块挂点 (里程碑逐步放开; Manager 只创建/接线/启停, 不碰业务) ----
    std::unique_ptr<VideoCameraReader> m_camera;   // M2 摄像头采集
    std::unique_ptr<FaceRecognitionEngine> m_face; // M3/M4 人脸引擎
    // std::unique_ptr<VideoPersonStore>  m_person;   // M5 名单库
    // std::unique_ptr<VideoHttpApi>      m_http;     // M5 http 服务出口

};

inline VideoServiceManager VideoServiceManager::Instance; 

} // namespace Vsis

#endif // VSIS_VIDEO_SERVICE_MANAGER_H