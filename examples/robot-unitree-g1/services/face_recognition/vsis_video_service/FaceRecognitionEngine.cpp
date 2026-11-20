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

#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <iostream>

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
    m_intimacyUpgradeEncounters = auxer->m_intimacyUpgradeEncounters;
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
    m_mappingPath = mappingPath;
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
        int baseIntimacy = item.value("base_intimacy_level", 1);
        int encounterCount = item.value("encounter_count", 0);
        
        if (id == -1 || name.empty()) {
            SPDLOG_WARN("[Face] 跳过无效条目: id={}, name='{}'", id, name);
            continue;
        }
        
        if (baseIntimacy < 1 || baseIntimacy > 5 || encounterCount < 0) {
            SPDLOG_ERROR("[Face] 亲密度档案无效: id={}, base={}, encounters={}",
                         id, baseIntimacy, encounterCount);
            HFFeatureHubDataDisable();
            HFTerminateInspireFace();
            return false;
        }
        const int autoIntimacy = intimacyForEncounters(encounterCount);
        m_mapping[id] = {name, hello, photo, baseIntimacy, encounterCount,
                         autoIntimacy, std::max(baseIntimacy, autoIntimacy)};
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

    std::cout << "FACE_READY count=" << m_mapping.size() << std::endl;

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

int FaceRecognitionEngine::intimacyForEncounters(int encounters) const {
    for (int index = static_cast<int>(m_intimacyUpgradeEncounters.size()) - 1; index >= 0; --index) {
        if (encounters >= m_intimacyUpgradeEncounters[index]) return index + 2;
    }
    return 1;
}

bool FaceRecognitionEngine::saveMapping() {
    json output = json::array();
    for (const auto& [id, person] : m_mapping) {
        output.push_back({
            {"id", id},
            {"name", person.name},
            {"hello", person.hello},
            {"photo", person.photo},
            {"base_intimacy_level", person.baseIntimacyLevel},
            {"encounter_count", person.encounterCount},
            {"auto_intimacy_level", person.autoIntimacyLevel},
            {"effective_intimacy_level", person.effectiveIntimacyLevel},
        });
    }
    const std::string tempPath = m_mappingPath + ".tmp";
    std::ofstream temp(tempPath, std::ios::trunc);
    if (!temp.good()) {
        SPDLOG_ERROR("[Face] 无法写入临时名单文件: {}", tempPath);
        return false;
    }
    temp << output.dump(2) << '\n';
    temp.close();
    if (std::rename(tempPath.c_str(), m_mappingPath.c_str()) != 0) {
        SPDLOG_ERROR("[Face] 更新名单文件失败: {}", m_mappingPath);
        std::remove(tempPath.c_str());
        return false;
    }
    return true;
}

void FaceRecognitionEngine::emitEncounterEvent(int id, float conf, const PersonInfo& person) {
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto epoch = seconds.time_since_epoch().count();
    std::ostringstream timestamp;
    timestamp << epoch;
    json event = {
        {"event_type", "face_encounter"},
        {"event_id", std::to_string(id) + "-" + std::to_string(epoch)},
        {"person_id", id},
        {"name", person.name},
        {"hello", person.hello.empty() ? person.name + "，您好！" : person.hello},
        {"confidence", conf},
        {"encounter_count", person.encounterCount},
        {"base_intimacy_level", person.baseIntimacyLevel},
        {"intimacy_level", person.effectiveIntimacyLevel},
        {"timestamp", timestamp.str()},
    };
    std::cout << "FACE_EVENT " << event.dump() << std::endl;
    SPDLOG_INFO("[Face] 已输出见面事件: id={}, 姓名={}, 次数={}, 亲密度={}",
                id, person.name, person.encounterCount, person.effectiveIntimacyLevel);
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
        slot.newEncounter = true;
    } else {
        slot.consecutiveHits++;
    }

    if (slot.consecutiveHits == m_confirmHits) {
        auto it = m_mapping.find(id);
        if (it != m_mapping.end()) {
            PersonInfo& person = it->second;
            if (slot.newEncounter) {
                ++person.encounterCount;
                person.autoIntimacyLevel = intimacyForEncounters(person.encounterCount);
                person.effectiveIntimacyLevel =
                    std::max(person.baseIntimacyLevel, person.autoIntimacyLevel);
                if (!saveMapping()) {
                    SPDLOG_WARN("[Face] 见面次数未能持久化: id={}", id);
                }
            }
            slot.newEncounter = false;
            if (std::chrono::duration_cast<std::chrono::seconds>(
                    now - slot.lastEvent).count() >= m_cooldownSec) {
                slot.lastEvent = now;
                SPDLOG_INFO("[Face] 已确认见面: id={}, 姓名={}, 次数={}, 亲密度={}, 得分={:.2f}",
                            id, person.name, person.encounterCount,
                            person.effectiveIntimacyLevel, conf);
                emitEncounterEvent(id, conf, person);
            }
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
