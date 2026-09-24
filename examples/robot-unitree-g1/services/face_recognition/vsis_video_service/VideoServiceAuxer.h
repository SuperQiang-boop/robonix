/**
 * @file VideoServiceAuxer.h
 * @brief 管家助手 —— 目录定位、配置读取、日志装配
 *
 * @author Teate
 * @date 2026-07-01
 * @version 1.1.0
 * @copyright Copyright (c) 2023, VSISLab. All rights reserved.
 */
#ifndef VSIS_VIDEO_SERVICE_AUXER_H
#define VSIS_VIDEO_SERVICE_AUXER_H

#include <string>

namespace Vsis {

class VideoServiceAuxer {
public:
    VideoServiceAuxer() = default;
    ~VideoServiceAuxer() = default;

    // 依次执行: 定位程序目录 → 读配置 → 装配日志 (顺序固定, 失败即退出进程)
    void init();

    inline std::string getAppDir() const { return m_appDir; }  // 工程根, 以 / 结尾

    // 取子目录全路径, 如 getDir("config") → "<工程根>/config/"
    inline std::string getDir(const std::string& dirName) const { return m_appDir + dirName + "/"; }

    // 日志配置项 (readConfig 填充, setLogger 消费)
    int m_logMode{0};    // 0=仅控制台 1=仅文件 2=控制台+文件
    int m_logLevel{0};   // 0=trace 1=debug 2=info 3=warn 4=err 5=critical

    // 视频配置项 (readConfig 填充, Manager 消费)
    std::string m_videoSource;      // RTSP/HTTP/HTTPS URL 或 V4L2 设备; 空=跳过相机模块
    int m_videoReconnectMs{3000};   // 断流重连间隔 (毫秒)
    int m_videoWidth{1280};         // V4L2 请求宽 (0=默认; 网络流忽略)
    int m_videoHeight{720};         // V4L2 请求高

    double m_searchThreshold{0.5};
    int m_greetCooldownSec{30};
    int m_maxFaces{3};
    int m_confirmHits{3};
    int m_absentSec{3};
    std::string m_audioHost{"127.0.0.1"};
    int m_audioPort{10086}; 

private:
    std::string m_appDir;   // 程序目录的上一级 (工程根), 以 / 结尾

    void initAppDir();
    void readConfig();
    void setLogger();
};

} // namespace Vsis

#endif // VSIS_VIDEO_SERVICE_AUXER_H
