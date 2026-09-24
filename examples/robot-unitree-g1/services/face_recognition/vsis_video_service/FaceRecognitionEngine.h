/**
 * @file FaceRecognitionEngine.h
 * @brief 人脸识别引擎 —— InspireFace 封装
 *
 * @author Teate
 * @date 2026-09-16
 * @version 1.0.0
 * @copyright Copyright (c) 2023, VSISLab. All rights reserved.
 */
#ifndef VSIS_FACE_RECOGNITION_ENGINE_H
#define VSIS_FACE_RECOGNITION_ENGINE_H

#include <thread>
#include <atomic>
#include <unordered_map>
#include <string>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <inspireface.h>

namespace Vsis {

class VideoCameraReader;

struct PersonInfo {
    std::string name;
    std::string hello;
    std::string photo;
};

class FaceRecognitionEngine {
public:
    explicit FaceRecognitionEngine(VideoCameraReader& cam);
    ~FaceRecognitionEngine();

    bool start();
    void stop();

    FaceRecognitionEngine(const FaceRecognitionEngine&) = delete;
    FaceRecognitionEngine& operator=(const FaceRecognitionEngine&) = delete;

private:
    void workerLoop();
    void hello(const std::string& text);

    // RAII 资源守卫
    struct ImageStreamGuard {
        HFImageStream stream{nullptr};
        explicit ImageStreamGuard() = default;
        ~ImageStreamGuard() { if (stream) HFReleaseImageStream(stream); }
        ImageStreamGuard(const ImageStreamGuard&) = delete;
        ImageStreamGuard& operator=(const ImageStreamGuard&) = delete;
    };

    struct ImageBitmapGuard {
        HFImageBitmap bitmap{nullptr};
        explicit ImageBitmapGuard() = default;
        ~ImageBitmapGuard() { if (bitmap) HFReleaseImageBitmap(bitmap); }
        ImageBitmapGuard(const ImageBitmapGuard&) = delete;
        ImageBitmapGuard& operator=(const ImageBitmapGuard&) = delete;
    };

    struct FeatureGuard {
        HFFaceFeature* feat{nullptr};
        bool owns{true};
        explicit FeatureGuard(HFFaceFeature* f = nullptr, bool own = true) : feat(f), owns(own) {}
        ~FeatureGuard() { if (feat && owns) HFReleaseFaceFeature(feat); }
        FeatureGuard(const FeatureGuard&) = delete;
        FeatureGuard& operator=(const FeatureGuard&) = delete;
        FeatureGuard(FeatureGuard&& other) noexcept : feat(other.feat), owns(other.owns) {
            other.feat = nullptr;
            other.owns = false;
        }
        FeatureGuard& operator=(FeatureGuard&& other) noexcept {
            if (this != &other) {
                if (feat && owns) HFReleaseFaceFeature(feat);
                feat = other.feat;
                owns = other.owns;
                other.feat = nullptr;
                other.owns = false;
            }
            return *this;
        }
        void dismiss() { owns = false; }
        void adopt(HFFaceFeature* f) {
            if (feat && owns) HFReleaseFaceFeature(feat);
            feat = f;
            owns = true;
        }
    };

    // 阶段函数
    enum class PrepareResult { NO_FACE, EXTRACT_FAIL, OK };
    PrepareResult prepareFeature(cv::Mat& frame, HFFaceFeature& outFeat);
    bool searchFeature(HFFaceFeature& feat, HFloat& outConf, HFFaceFeatureIdentity& outMatched);
    void onFaceMatch(int id, float conf);
    void onNoMatch();

    VideoCameraReader& m_cam;
    std::thread m_thread;
    std::atomic<bool> m_running{false};
    
    // InspireFace 相关
    HFSession m_session{nullptr};
    std::unordered_map<int, PersonInfo> m_mapping;
    
    // 配置参数（从 cfg.yaml 读取）
    float m_scoreGate{0.5f};              // 业务判定阈值
    int m_cooldownSec{30};                // 问候冷却时间（秒）
    int m_maxFaces{10};                   // 最大检测人脸数
    int m_confirmHits{3};
    int m_absentSec{3};                   // 离场判定阈值（秒）

    // 日志限流
    std::chrono::steady_clock::time_point m_lastNoFaceLogTime;  // 上次"无人脸"日志时间
    std::chrono::steady_clock::time_point m_lastNoMatchLogTime; // 上次"未命中"日志时间

    // v2 槽位状态机
    struct FaceSlot {
        std::chrono::steady_clock::time_point lastSeen{};
        std::chrono::steady_clock::time_point lastGreet{};
        int consecutiveHits{0};
    };
    std::unordered_map<int, FaceSlot> m_slots;
};

} // namespace Vsis

#endif // VSIS_FACE_RECOGNITION_ENGINE_H