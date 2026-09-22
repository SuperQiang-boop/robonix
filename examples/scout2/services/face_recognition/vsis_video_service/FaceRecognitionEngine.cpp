/**
 * @file FaceRecognitionEngine.cpp
 * @brief 人脸识别引擎实现
 *
 * @author Teate
 * @date 2026-09-16
 * @version 1.0.0
 * @copyright Copyright (c) 2023, VSISLab. All rights reserved.
 */
#include "FaceRecognitionEngine.h"
#include "VideoCameraReader.h"
#include "VideoServiceManager.h"
#include "VideoDef.h"
#include "spdlog/spdlog.h"
#include "nlohmann/json.hpp"
#include "httplib.h"

#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <filesystem>

using json = nlohmann::json;

namespace Vsis {

FaceRecognitionEngine::FaceRecognitionEngine(VideoCameraReader& cam)
    : m_cam(cam) {
}

FaceRecognitionEngine::~FaceRecognitionEngine() {
    stop();
}

bool FaceRecognitionEngine::start() {
    if (m_running) {
        return true;
    }

    auto auxer = VideoServiceManager::Instance.getAuxer();

    // 0) 读取配置参数
    m_scoreGate   = auxer->m_searchThreshold;
    m_cooldownSec = auxer->m_greetCooldownSec;
    m_maxFaces    = auxer->m_maxFaces;  // 会话检测容量，见 HFCreateInspireFaceSession
    m_confirmHits = auxer->m_confirmHits;
    m_absentSec   = auxer->m_absentSec;
    SPDLOG_INFO("[Face] 配置: gate={:.2f}, confirm={}, cooldown={}s, absent={}s, maxFaces={}",
                m_scoreGate, m_confirmHits, m_cooldownSec, m_absentSec, m_maxFaces);

    // a) 路径构建
    std::string appDir = auxer->getAppDir();
    std::string packPath = appDir + "/" + facepath::PACK;
    std::string hubDbPath = appDir + "/" + facepath::DB;
    std::string mappingPath = appDir + "/" + facepath::MAPPING;
    std::string photoDir = appDir + "/" + facepath::PHOTO;

    SPDLOG_INFO("[Face] pack路径: {}", packPath);
    SPDLOG_INFO("[Face] hub数据库: {}", hubDbPath);
    SPDLOG_INFO("[Face] mapping文件: {}", mappingPath);
    SPDLOG_INFO("[Face] 照片目录: {}", photoDir);

    // b) InspireFace 初始化
    struct stat st;
    if (stat(packPath.c_str(), &st) != 0) {
        SPDLOG_ERROR("[Face] pack不存在: {}", packPath);
        return false;
    }

    HResult ret = HFLaunchInspireFace(packPath.c_str());
    if (ret != HSUCCEED) {
        SPDLOG_ERROR("[Face] HFLaunchInspireFace失败: {}", ret);
        return false;
    }

    // 配置并启用 Feature Hub
    HFFeatureHubConfiguration cfg = {};
    cfg.primaryKeyMode = HF_PK_AUTO_INCREMENT;
    cfg.enablePersistence = 1;
    cfg.persistenceDbPath = const_cast<char*>(hubDbPath.c_str());
    cfg.searchMode = HF_SEARCH_MODE_EAGER;
    cfg.searchThreshold = m_scoreGate;

    ret = HFFeatureHubDataEnable(cfg);
    if (ret != HSUCCEED) {
        SPDLOG_ERROR("[Face] HFFeatureHubDataEnable失败: {}", ret);
        HFTerminateInspireFace();
        return false;
    }

    // c) 获取 hub 特征数
    HInt32 hubCount = 0;
    ret = HFFeatureHubGetFaceCount(&hubCount);
    if (ret != HSUCCEED) {
        SPDLOG_ERROR("[Face] HFFeatureHubGetFaceCount失败: {}", ret);
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    // d) 空库检查
    if (hubCount == 0) {
        SPDLOG_ERROR("[Face] 特征库为空, 拒绝启动");
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    // e) mapping.json 加载
    std::ifstream f(mappingPath);
    if (!f.good()) {
        SPDLOG_ERROR("[Face] mapping.json不可读: {}", mappingPath);
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    std::stringstream ss;
    ss << f.rdbuf();
    std::string s = ss.str();
    
    // 检查空文件
    bool blank = true;
    for (char c : s) {
        if (!isspace((unsigned char)c)) {
            blank = false;
            break;
        }
    }
    if (blank) {
        SPDLOG_ERROR("[Face] mapping.json为空: {}", mappingPath);
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    json mappingJson;
    try {
        mappingJson = json::parse(s);
    } catch (...) {
        SPDLOG_ERROR("[Face] mapping.json解析失败: {}", mappingPath);
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    // 解析为 unordered_map
    if (!mappingJson.is_array()) {
        SPDLOG_ERROR("[Face] mapping.json格式错误: 应为数组");
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    for (const auto& item : mappingJson) {
        int id = item.value("id", -1);
        std::string name = item.value("name", "");
        std::string hello = item.value("hello", "");
        std::string photo = item.value("photo", "");
        
        if (id == -1 || name.empty()) {
            SPDLOG_WARN("[Face] 跳过无效条目: id={}, name='{}'", id, name);
            continue;
        }
        
        m_mapping[id] = {name, hello, photo};
    }

    // f) 对账
    if (hubCount == (HInt32)m_mapping.size()) {
        SPDLOG_INFO("[Face] 特征库就绪, 人数={}", hubCount);
    } else {
        SPDLOG_WARN("[Face] 特征库与名单数量不一致 (hub={}, mapping={}), 仅告警继续", 
                    hubCount, m_mapping.size());
    }

    // g) 自检门
    if (m_mapping.empty()) {
        SPDLOG_ERROR("[Face] mapping为空, 无法自检");
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    // 取第一条进行自检
    auto firstIt = m_mapping.begin();
    int testId = firstIt->first;
    std::string testPhoto = firstIt->second.photo;
    std::string testName = firstIt->second.name;
    std::string testPhotoPath = appDir + "/person/" + testPhoto;

    SPDLOG_INFO("[Face] 自检: id={}, name={}, photo={}", testId, testName, testPhotoPath);

    // 自检照片防崩护栏
    if (stat(testPhotoPath.c_str(), &st) != 0) {
        SPDLOG_ERROR("[Face] 自检照片不存在: {}", testPhotoPath);
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    // 创建临时会话
    HFSessionCustomParameter param = {};
    param.enable_recognition = 1;
    ret = HFCreateInspireFaceSession(param, HF_DETECT_MODE_ALWAYS_DETECT, m_maxFaces, -1, -1, &m_session);
    if (ret != HSUCCEED) {
        SPDLOG_ERROR("[Face] HFCreateInspireFaceSession失败: {}", ret);
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    // 加载图片
    HFImageBitmap bitmap = nullptr;
    ret = HFCreateImageBitmapFromFilePath(testPhotoPath.c_str(), 3, &bitmap);
    if (ret != HSUCCEED) {
        SPDLOG_ERROR("[Face] HFCreateImageBitmapFromFilePath失败: {}, 路径: {}", ret, testPhotoPath);
        HFReleaseInspireFaceSession(m_session);
        m_session = nullptr;
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    // 创建图像流
    HFImageStream stream = nullptr;
    ret = HFCreateImageStreamFromImageBitmap(bitmap, HF_CAMERA_ROTATION_0, &stream);
    if (ret != HSUCCEED) {
        SPDLOG_ERROR("[Face] HFCreateImageStreamFromImageBitmap失败: {}", ret);
        HFReleaseImageBitmap(bitmap);
        HFReleaseInspireFaceSession(m_session);
        m_session = nullptr;
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    // 检测人脸
    HFMultipleFaceData faces = {};
    ret = HFExecuteFaceTrack(m_session, stream, &faces);
    if (ret != HSUCCEED || faces.detectedNum < 1) {
        SPDLOG_ERROR("[Face] 自检失败: 未检测到人脸, 路径: {}", testPhotoPath);
        HFReleaseImageStream(stream);
        HFReleaseImageBitmap(bitmap);
        HFReleaseInspireFaceSession(m_session);
        m_session = nullptr;
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    // 提取特征
    HFFaceFeature feat = {};
    ret = HFCreateFaceFeature(&feat);
    if (ret != HSUCCEED) {
        SPDLOG_ERROR("[Face] HFCreateFaceFeature失败: {}", ret);
        HFReleaseImageStream(stream);
        HFReleaseImageBitmap(bitmap);
        HFReleaseInspireFaceSession(m_session);
        m_session = nullptr;
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    ret = HFFaceFeatureExtractCpy(m_session, stream, faces.tokens[0], feat.data);
    if (ret != HSUCCEED) {
        SPDLOG_ERROR("[Face] HFFaceFeatureExtractCpy失败: {}", ret);
        HFReleaseFaceFeature(&feat);
        HFReleaseImageStream(stream);
        HFReleaseImageBitmap(bitmap);
        HFReleaseInspireFaceSession(m_session);
        m_session = nullptr;
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    // 搜索
    HFloat conf = 0;
    HFFaceFeatureIdentity matched = {};
    ret = HFFeatureHubFaceSearch(feat, &conf, &matched);
    
    // 清理资源
    HFReleaseFaceFeature(&feat);
    HFReleaseImageStream(stream);
    HFReleaseImageBitmap(bitmap);

    if (ret != HSUCCEED) {
        SPDLOG_ERROR("[Face] HFFeatureHubFaceSearch失败: {}", ret);
        HFReleaseInspireFaceSession(m_session);
        m_session = nullptr;
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    // 验证自检结果
    const float kSelfCheckScore = 0.9f;
    if (matched.id == testId && conf >= kSelfCheckScore) {
        SPDLOG_INFO("[Face] 自检通过 (得分={:.2f}, id={}, 姓名={})", conf, matched.id, testName);
    } else {
        SPDLOG_ERROR("[Face] 自检失败: 期望id={}, 实际id={}, 得分={:.2f}", 
                     testId, matched.id, conf);
        HFReleaseInspireFaceSession(m_session);
        m_session = nullptr;
        HFFeatureHubDataDisable();
        HFTerminateInspireFace();
        return false;
    }

    // 启动 worker 线程
    m_running = true;
    m_thread = std::thread(&FaceRecognitionEngine::workerLoop, this);
    SPDLOG_INFO("[Face] 人脸识别线程启动");
    
    return true;
}

void FaceRecognitionEngine::stop() {
    if (!m_running) {
        return;
    }
    
    // 先停 worker
    m_running = false;
    if (m_thread.joinable()) {
        m_thread.join();
    }
    SPDLOG_INFO("[Face] 人脸识别线程停止");

    // 再销毁 ISF 资源
    if (m_session) {
        HFReleaseInspireFaceSession(m_session);
        m_session = nullptr;
    }
    HFFeatureHubDataDisable();
    HFTerminateInspireFace();
    SPDLOG_INFO("[Face] 人脸资源已释放");
}

void FaceRecognitionEngine::hello(const std::string& text) {
    auto auxer = VideoServiceManager::Instance.getAuxer();

    // 快照配置：线程里只碰值拷贝，不碰 this / auxer
    std::string host = auxer->m_audioHost;
    int         port = auxer->m_audioPort;

    // 发完就忘：不阻塞视觉线程；问候受冷却限频，线程开销可忽略
    std::thread([host, port, text]() {
        httplib::Client cli(host, port);
        cli.set_connection_timeout(1, 0);   // 连接 1s
        cli.set_read_timeout(3, 0);         // 读取 3s

        json body = {{"text", text}};
        auto res = cli.Post("/speak", body.dump(), "application/json");

        if (!res) {
            SPDLOG_ERROR("[Face] 语音服务不可达: {}:{} (err={})",
                         host, port, static_cast<int>(res.error()));
            return;
        }
        if (res->status != 200) {
            SPDLOG_WARN("[Face] 语音接口异常: status={}, body={}", res->status, res->body);
            return;
        }
        SPDLOG_INFO("[Face] 已送达语音: '{}'", text);
    }).detach();
}

void FaceRecognitionEngine::workerLoop() {
    uint64_t frameCount = 0;
    
    while (m_running) {
        cv::Mat mat;
        if (!m_cam.read(mat, 100)) {
            continue;
        }
        ++frameCount;
        
        // 心跳保留
        if (frameCount % 600 == 0) {
            SPDLOG_DEBUG("[Face] 心跳: frames={}", frameCount);
        }

        HFFaceFeature feat = {};
        auto prepareRes = prepareFeature(mat, feat);

        if (prepareRes == PrepareResult::NO_FACE) {
            continue;   // v2: 短暂丢失由 absentSec 兜底，不重置任何状态
        }

        if (prepareRes == PrepareResult::EXTRACT_FAIL) {
            continue;
        }

        // OK: 守卫接管，本帧无论从哪条路离开循环体都会释放 feature
        FeatureGuard featGuard(&feat);
        HFloat conf = 0;
        HFFaceFeatureIdentity matched = {};
        if (!searchFeature(feat, conf, matched)) {
            continue;    // continue 跳到循环体末尾 → featGuard 析构 → 自动释放
        }
        if (conf >= m_scoreGate) {
            onFaceMatch(matched.id, conf);
        } else {
            onNoMatch();
        }
        // 循环体自然结束 → featGuard 析构 → 释放
    }
}

// 阶段函数实现
FaceRecognitionEngine::PrepareResult FaceRecognitionEngine::prepareFeature(cv::Mat& frame, HFFaceFeature& outFeat) {
    // 帧→SDK 图像
    HFImageBitmapData bitmapData = {};
    bitmapData.data = frame.data;
    bitmapData.width = frame.cols;
    bitmapData.height = frame.rows;
    bitmapData.channels = frame.channels();

    ImageBitmapGuard bitmapGuard;
    HResult ret = HFCreateImageBitmap(&bitmapData, &bitmapGuard.bitmap);
    if (ret != HSUCCEED) {
        SPDLOG_DEBUG("[Face] HFCreateImageBitmap失败: {}", ret);
        return PrepareResult::EXTRACT_FAIL;
    }

    ImageStreamGuard streamGuard;
    ret = HFCreateImageStreamFromImageBitmap(bitmapGuard.bitmap, HF_CAMERA_ROTATION_0, &streamGuard.stream);
    if (ret != HSUCCEED) {
        SPDLOG_DEBUG("[Face] HFCreateImageStreamFromImageBitmap失败: {}", ret);
        return PrepareResult::EXTRACT_FAIL;
    }

    // 检测人脸
    HFMultipleFaceData faces = {};
    ret = HFExecuteFaceTrack(m_session, streamGuard.stream, &faces);

    if (ret != HSUCCEED || faces.detectedNum < 1) {
        // 无人脸：每 5 秒最多一条
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - m_lastNoFaceLogTime).count() >= 5) {
            SPDLOG_TRACE("[Face] 无人脸");
            m_lastNoFaceLogTime = now;
        }
        return PrepareResult::NO_FACE;
    }

    // 取最大脸（按面积）
    int maxFaceIdx = 0;
    int maxArea = 0;
    for (int i = 0; i < faces.detectedNum; ++i) {
        int w = faces.rects[i].width;
        int h = faces.rects[i].height;
        int area = w * h;
        if (area > maxArea) {
            maxArea = area;
            maxFaceIdx = i;
        }
    }

    // 提取特征
    HFFaceFeature* featPtr = &outFeat;
    ret = HFCreateFaceFeature(featPtr);
    if (ret != HSUCCEED) {
        SPDLOG_DEBUG("[Face] HFCreateFaceFeature失败: {}", ret);
        return PrepareResult::EXTRACT_FAIL;
    }

    ret = HFFaceFeatureExtractCpy(m_session, streamGuard.stream, faces.tokens[maxFaceIdx], outFeat.data);

    // stream/bitmap 守卫在此析构，确保覆盖 detect→extract
    if (ret != HSUCCEED) {
        SPDLOG_DEBUG("[Face] HFFaceFeatureExtractCpy失败: {}", ret);
        HFReleaseFaceFeature(&outFeat);
        return PrepareResult::EXTRACT_FAIL;
    }

    return PrepareResult::OK;
}

bool FaceRecognitionEngine::searchFeature(HFFaceFeature& feat, HFloat& outConf, HFFaceFeatureIdentity& outMatched) {
    HFloat conf = 0;
    HFFaceFeatureIdentity matched = {};
    HResult ret = HFFeatureHubFaceSearch(feat, &conf, &matched);

    if (ret != HSUCCEED) {
        SPDLOG_DEBUG("[Face] HFFeatureHubFaceSearch失败: {}", ret);
        return false;
    }

    outConf = conf;
    outMatched = matched;
    return true;
}

void FaceRecognitionEngine::onFaceMatch(int id, float conf) {
    auto now = std::chrono::steady_clock::now();
    FaceSlot& slot = m_slots[id];   // 不存在则默认构造（lastSeen=epoch → 首次必判入场）

    // 离场判定：距上次见到 ≥ absentSec → 视为(重新)入场
    bool reentry = std::chrono::duration_cast<std::chrono::seconds>(
                       now - slot.lastSeen).count() >= m_absentSec;
    slot.lastSeen = now;

    if (reentry) {
        auto it = m_mapping.find(id);
        if (it != m_mapping.end()) {
            SPDLOG_INFO("[Face] 识别到: id={}, 姓名={}, 得分={:.2f}", id, it->second.name, conf);
        }
        slot.consecutiveHits = 1;
    } else {
        slot.consecutiveHits++;
    }

    if (slot.consecutiveHits == m_confirmHits) {
        auto it = m_mapping.find(id);
        if (it != m_mapping.end() &&
            std::chrono::duration_cast<std::chrono::seconds>(
                now - slot.lastGreet).count() >= m_cooldownSec) {
            SPDLOG_INFO("[Face] 问候: {}", it->second.hello);
            slot.lastGreet = now;
            hello(it->second.hello);
        }
    }
}

void FaceRecognitionEngine::onNoMatch() {
    // 未命中：脸还在但认不出（低头/侧脸/得分抖动）
    // 不重置任何状态——抖动不打断 3 帧计数，id 保持不丢
    // 每 2 秒最多一条 debug 日志
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - m_lastNoMatchLogTime).count() >= 2) {
        SPDLOG_DEBUG("[Face] 未命中");
        m_lastNoMatchLogTime = now;
    }
}


} // namespace Vsis
